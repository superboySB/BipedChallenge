#include "jump_planner.h"

#include <chrono>
#include <iostream>
#include <drake/multibody/math/spatial_algebra.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    JumpPlanner<T>::JumpPlanner(double time_step, double offset,
                                const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), plant_(plant)
    {
      DRAKE_THROW_UNLESS(time_step > 0);
      nv_ = plant_.num_velocities();
      nq_ = plant_.num_positions();
      plant_context_ = plant_.CreateDefaultContext();

      left_stance_input_port_ = &this->DeclareVectorInputPort("left_stance", 1);
      right_stance_input_port_ = &this->DeclareVectorInputPort("right_stance", 1);
      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", nq_);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", nv_);
      phase_input_port_ = &this->DeclareVectorInputPort("phase", 1);

      gait_epoch_id_ = this->DeclareDiscreteState(1);

      left_stance_id_ = this->DeclareDiscreteState(1);
      right_stance_id_ = this->DeclareDiscreteState(1);
      left_stance_output_port_ =
          &this->DeclareStateOutputPort("left_stance", left_stance_id_);
      right_stance_output_port_ =
          &this->DeclareStateOutputPort("right_stance", right_stance_id_);

      h_ref_id_ = this->DeclareDiscreteState(nh_);
      hd_ref_id_ = this->DeclareDiscreteState(nh_);
      hdd_ref_id_ = this->DeclareDiscreteState(nh_);
      h_ref_output_port_ = &this->DeclareStateOutputPort("h_ref", h_ref_id_);
      hd_ref_output_port_ = &this->DeclareStateOutputPort("hd_ref", hd_ref_id_);
      hdd_ref_output_port_ = &this->DeclareStateOutputPort("hdd_ref", hdd_ref_id_);
      stand_pos = Eigen::VectorXd::Zero(6);

      this->DeclarePeriodicDiscreteUpdateEvent(time_step, offset, &JumpPlanner::Update);
    }

    template <typename T>
    void JumpPlanner<T>::Update(const drake::systems::Context<T> &context,
                                drake::systems::DiscreteValues<T> *next_state) const
    {
      const auto &in_left_stance = get_left_stance_input_port().Eval(context)(0);
      const auto &in_right_stance = get_left_stance_input_port().Eval(context)(0);
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);
      int phase = get_phase_input_port().Eval(context)(0);
      const T T_gait = 0.5;

      const auto &old_left_st = context.get_discrete_state(left_stance_id_).value()(0);
      const auto &old_right_st = context.get_discrete_state(right_stance_id_).value()(0);
      auto &left_stance = next_state->get_mutable_value(left_stance_id_)(0);
      auto &right_stance = next_state->get_mutable_value(right_stance_id_)(0);

      const auto &old_gait_epoch = context.get_discrete_state(gait_epoch_id_).value()(0);
      auto &gait_epoch = next_state->get_mutable_value(gait_epoch_id_)(0);

      auto h_ref = next_state->get_mutable_value(h_ref_id_);
      auto hd_ref = next_state->get_mutable_value(hd_ref_id_);
      auto hdd_ref = next_state->get_mutable_value(hdd_ref_id_);
      if (!is_initialed)
      {
        setinitialPos(q);
        is_initialed = true;
      }

      plant_.SetPositions(plant_context_.get(), q);
      plant_.SetVelocities(plant_context_.get(), v);

      const auto &torso_body = plant_.GetBodyByName("torso");
      const auto &lfoot_body = plant_.GetBodyByName("l_foot_x");
      const auto &rfoot_body = plant_.GetBodyByName("r_foot_x");
      const auto &world_body = plant_.world_body();
      const auto &torso_frame = torso_body.body_frame();
      const auto &lfoot_frame = lfoot_body.body_frame();
      const auto &rfoot_frame = rfoot_body.body_frame();
      const auto &world_frame = plant_.world_frame();

      T old_t_gait = context.get_time() - old_gait_epoch;
      // 事实上planner context中左右脚的状态总是相同的
      if (old_left_st == 0 && old_right_st == 0)
      {
        if (in_left_stance == 1 && in_right_stance == 1 && old_t_gait > 0.1)
        {
          left_stance = 1;
          right_stance = 1;
          gait_epoch = context.get_time();
          std::cout << "t: " << context.get_time() << "; ";
          std::cout << "phase: " << 1 << "\n";
          const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_, lfoot_body);
          const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_, rfoot_body);
          const auto &p_stmid = (X_WLfoot.translation() + X_WRfoot.translation()) / 2;
          std::cout << "p_stmid:" << p_stmid.transpose() << std::endl;
          jump_done = true;
          setinitialPos(q);
        }
        else
        {
          left_stance = old_left_st;
          right_stance = old_right_st;
          gait_epoch = old_gait_epoch;
        }
      }
      else
      {
        if (old_t_gait > T_gait && !jump_done)
        {
          left_stance = 0;
          right_stance = 0;
          gait_epoch = context.get_time();
          std::cout << "t: " << context.get_time() << "; ";
          std::cout << "phase: " << 0 << "\n";
        }
        else
        {
          left_stance = old_left_st;
          right_stance = old_right_st;
          gait_epoch = old_gait_epoch;
        }
      }
      if (phase == 2 && !jump_done && stand_state)
      {
        stand_state = false;
        // gait_epoch = context.get_time();
        std::cout << "jump start...\n";
      }
      if (stand_state && !jump_done)
      {
        gait_epoch = context.get_time();
      }
      // if (context.get_time()<2){
      //   stand_state = true;
      //   gait_epoch = context.get_time();
      // }else if (context.get_time()<2.1)
      // {
      //   stand_state = false;
      // }
      T t_gait = context.get_time() - gait_epoch;
      std::cout << "gait_epoch:" << gait_epoch << " t_gait" << t_gait << std::endl;

      if (left_stance == 1)
      {
        const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_, lfoot_body);
        const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_, rfoot_body);
        const auto &p_WScm = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
        const auto &v_WScm =
            plant_.CalcCenterOfMassTranslationalVelocityInWorld(*plant_context_);
        const auto &p_mid = (X_WLfoot.translation() + X_WRfoot.translation()) / 2;
        T s = t_gait / T_gait;
        const T w = M_PI / T_gait;
        if (jump_done && t_gait > 0.25 && p_WScm(2) < stand_pos[5] + 0.6 && !stand_state)
          stand_state = true;
        if (stand_state)
        {
          h_ref.segment(0, 6) = stand_pos;
          h_ref(3) = p_mid(0);
          h_ref(4) = p_mid(1);
          h_ref(5) = (jump_done) ? stand_pos[5] + 0.5 : stand_pos[5] + 0.35;
          hd_ref.head(6).fill(0);
          hdd_ref.head(6).fill(0);
        }
        else
        {
          h_ref(0) = 0;
          h_ref(1) = 0;
          h_ref(2) = stand_pos[2];
          h_ref(3) = p_mid(0);
          h_ref(4) = p_mid(1);
          h_ref(5) = stand_pos[5] + 0.6 + 0.2 * -sin(M_PI * s);

          hd_ref.head(5).fill(0);
          hd_ref(5) = 0.2 * w * -cos(M_PI * s);

          hdd_ref.head(5).fill(0);
          hdd_ref(5) = 0.2 * w * w * sin(M_PI * s);
        }
      }
      else
      {
        const auto &p_WScm = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
        const auto &v_WScm =
            plant_.CalcCenterOfMassTranslationalVelocityInWorld(*plant_context_);
        T t_sat = std::clamp(t_gait, T(0), T_gait);

        const auto &X_Wbody = plant_.EvalBodyPoseInWorld(*plant_context_, torso_body);
        const auto &R_BW = X_Wbody.rotation();
        const auto &euler_WLbody = X_Wbody.rotation().ToRollPitchYaw();
        // Eigen::Matrix3d R_BW = body_quat.normalized().toRotationMatrix();
        drake::Vector3<T> l_offset = {0, 0.1, 0};
        drake::Vector3<T> r_offset = {0, -0.1, 0};
        const auto &p_lfoffset_W = R_BW * l_offset;
        const auto &p_rfoffset_W = R_BW * r_offset;

        // h_ref.tail(10)->lf_pitch,lf_yaw,lf.tran,rf_pitch,rf_yaw,rf.tran,
        h_ref(6) = 0;
        h_ref(7) = stand_pos[2];
        h_ref(8) = p_WScm[0] + p_lfoffset_W(0);
        h_ref(9) = p_lfoffset_W(1) + p_WScm(1);
        h_ref(10) = p_WScm(2) - 0.6;
        h_ref(11) = 0;
        h_ref(12) = stand_pos[2];
        h_ref(13) = p_WScm[0] + p_rfoffset_W(0);
        h_ref(14) = p_rfoffset_W(1) + p_WScm(1);
        h_ref(15) = p_WScm(2) - 0.6;

        hd_ref.fill(0);
        hd_ref(10) = v_WScm(2);
        hd_ref(15) = v_WScm(2);

        hdd_ref.fill(0);
        hdd_ref(10) = -g;
        hdd_ref(15) = -g;
      }
    }
    template class ::my_drake_examples::biped::JumpPlanner<double>;

  } // namespace biped
} // namespace my_drake_examples
