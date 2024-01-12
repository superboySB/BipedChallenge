#include "alip_planner.h"

#include <drake/multibody/math/spatial_algebra.h>

#include <chrono>
#include <iostream>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    AlipPlanner<T>::AlipPlanner(double time_step, double offset,
                                const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), plant_(plant)
    {
      DRAKE_THROW_UNLESS(time_step >= 0);
      // 后面用的是discrete state，即普通VectorX类型。这里用Abstract只是演示怎么用？
      stanceLeg_input_port_ = &this->DeclareAbstractInputPort("EstStates.stanceLeg",
                                                              drake::Value<int>());
      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", 17);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", 16);
      vtarget_input_port_ = &this->DeclareVectorInputPort("CMD.vtarget", 2);
      H_input_port_ = &this->DeclareVectorInputPort("CMD.H", 1);
      stand_flag_input_port_ = &this->DeclareVectorInputPort("phase_input", 1);
      stepWidth_input_port_ = &this->DeclareVectorInputPort("CMD.stepWidth", 1);
      gait_period_input_port_ = &this->DeclareVectorInputPort("CMD.gait_period", 1);
      turn_rps_input_port_ = &this->DeclareVectorInputPort("CMD.turn_rps", 1);
      phase_output_port = &this->DeclareVectorOutputPort("phase_output", 1, &AlipPlanner::phaseOutput);

      gait_epoch_id_ = this->DeclareDiscreteState(1);
      p0_WSw_id_ = this->DeclareDiscreteState(3);

      // 它既用于输出，也用于状态，检测切换支撑脚
      stanceLeg_id_ = this->DeclareDiscreteState(1);
      stanceLeg_output_port_ =
          &this->DeclareStateOutputPort("stanceLeg", stanceLeg_id_);

      h_ref_id_ = this->DeclareDiscreteState(10);
      hd_ref_id_ = this->DeclareDiscreteState(10);
      hdd_ref_id_ = this->DeclareDiscreteState(10);
      vf_WScm_id_ = this->DeclareDiscreteState(3);
      vnextf_id_ = this->DeclareDiscreteState(3);
      pf_WSw_id_ = this->DeclareDiscreteState(3);
      calc_elapsed_id_ = this->DeclareDiscreteState(1);
      begin_calc_id_ = this->DeclareDiscreteState(1);
      end_calc_id_ = this->DeclareDiscreteState(1);

      h_ref_output_port_ = &this->DeclareStateOutputPort("h_ref", h_ref_id_);
      hd_ref_output_port_ = &this->DeclareStateOutputPort("hd_ref", hd_ref_id_);
      hdd_ref_output_port_ = &this->DeclareStateOutputPort("hdd_ref", hdd_ref_id_);
      vf_WScm_output_port_ =
          &this->DeclareStateOutputPort("vf_StScm_W", vf_WScm_id_);
      vnextf_output_port_ = &this->DeclareStateOutputPort("vnextf", vnextf_id_);
      pf_WSw_output_port_ = &this->DeclareStateOutputPort("pf_WSw", pf_WSw_id_);
      calc_elapsed_output_port_ =
          &this->DeclareStateOutputPort("calc_elapsed", calc_elapsed_id_);
      begin_calc_output_port_ =
          &this->DeclareStateOutputPort("begin_calc", begin_calc_id_);
      end_calc_output_port_ =
          &this->DeclareStateOutputPort("end_calc", end_calc_id_);

      this->DeclarePeriodicDiscreteUpdateEvent(time_step, offset,
                                               &AlipPlanner::Update);

      plant_context_ = plant_.CreateDefaultContext();
      total_mass_ = plant_.CalcTotalMass(*plant_context_);
      stand_pose.resize(6);
    }

    template <typename T>
    void AlipPlanner<T>::Update(
        const drake::systems::Context<T> &context,
        drake::systems::DiscreteValues<T> *next_state) const
    {
      const auto start = std::chrono::steady_clock::now();
      int input_stanceLeg = get_stanceLeg_input_port().template Eval<int>(context);
      int phase_input = get_phase_input_port().Eval(context)(0);
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);
      const auto &vtarget = get_vtarget_input_port().Eval(context);
      const auto &H = get_H_input_port().Eval(context)(0);
      const auto &stepWidth = get_stepWidth_input_port().Eval(context)(0);
      const auto &Tgait = get_gait_period_input_port().Eval(context)(0);
      const auto &turn_rps = get_turn_rps_input_port().Eval(context)(0);

      const auto &old_p0_WSw = context.get_discrete_state(p0_WSw_id_).value();

      int old_stanceLeg =
          context.get_discrete_state(stanceLeg_id_).value()(0);
      auto &stanceLeg = next_state->get_mutable_value(stanceLeg_id_)(0);

      auto &gait_epoch = next_state->get_mutable_value(gait_epoch_id_)(0);
      auto p0_WSw = next_state->get_mutable_value(p0_WSw_id_);

      auto h_ref = next_state->get_mutable_value(h_ref_id_);
      auto hd_ref = next_state->get_mutable_value(hd_ref_id_);
      auto hdd_ref = next_state->get_mutable_value(hdd_ref_id_);
      auto vf_WScm = next_state->get_mutable_value(vf_WScm_id_);
      auto vnextf = next_state->get_mutable_value(vnextf_id_);
      auto pf_WSw = next_state->get_mutable_value(pf_WSw_id_);
      auto &calc_elapsed = next_state->get_mutable_value(calc_elapsed_id_)(0);
      auto &begin_calc = next_state->get_mutable_value(begin_calc_id_)(0);
      auto &end_calc = next_state->get_mutable_value(end_calc_id_)(0);

      if (phase_input == 1 && stand_phase == -1)
      {
        stand_phase = 0;
        std::cout << "changing to stand,t:" << context.get_time() << std::endl;
      }
      else if (phase_input == 0 && stand_phase == 2)
      {
        split_time = context.get_time();
        gait_epoch = context.get_time() - 0.3;

        std::cout << "changing to walk ...,t:" << context.get_time() << std::endl;
      }
      int gait_count = (context.get_time() + 0.3 - split_time) / Tgait;
      if (gait_count % 2)
      {
        input_stanceLeg = 1;
      }
      else
      {
        input_stanceLeg = -1;
      }
      if (phase_input == 0 && stand_phase == 2)
      {
        old_stanceLeg = (input_stanceLeg == 1) ? -1 : 1;
      }

      plant_.SetPositions(plant_context_.get(), q);
      plant_.SetVelocities(plant_context_.get(), v);

      /*来自 One-Step Ahead Prediction of Angular Momentum about the Contact Point
      for Control of Bipedal Locomotion: Validation in a LIP-inspired Controller
      以及对应的MATLAB代码Cassie_Controller_4.m 的落脚点选择部分。
      空间代数遵循drake的monogram notation命名。*/

      // 计算LIP位置
      const auto &p_WScm = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
      const auto &st_footbody =
          plant_.GetBodyByName(input_stanceLeg > 0 ? "l_foot_x" : "r_foot_x");
      const auto &X_WSt = plant_.EvalBodyPoseInWorld(*plant_context_, st_footbody);
      const auto &p_WSt = X_WSt.translation();
      const auto &p_StScm_W = p_WScm - p_WSt;
      const auto yaw_WSt = X_WSt.rotation().ToRollPitchYaw().yaw_angle();
      const auto &p_StScm_Fst =
          drake::math::RotationMatrix<T>::MakeZRotation(-yaw_WSt) * p_StScm_W;
      // 计算LIP速度
      const auto &L_WSt =
          plant_.CalcSpatialMomentumInWorldAboutPoint(*plant_context_, p_WSt);
      const auto &Lrot_WSt = L_WSt.rotational();
      const auto &vx_pseudo_WScm = Lrot_WSt(1) / total_mass_ / p_StScm_W(2);
      const auto &vy_pseudo_WScm = -Lrot_WSt(0) / total_mass_ / p_StScm_W(2);
      const auto &v_WScm = plant_.CalcCenterOfMassTranslationalVelocityInWorld(
          *plant_context_); // 用不到，如果用角动量的话
      drake::Vector3<T> v_pseudo_WScm;
      v_pseudo_WScm(0) = vx_pseudo_WScm;
      v_pseudo_WScm(1) = vy_pseudo_WScm;
      v_pseudo_WScm(2) = 0;
      drake::Vector3<T> v_pseudo_WScm_Fst =
          drake::math::RotationMatrix<T>::MakeZRotation(-yaw_WSt) * v_pseudo_WScm;

      // 处理切换脚时状态记录
      if (old_stanceLeg != 0 && old_stanceLeg != input_stanceLeg)
      {
        stepcount++;
        if (stand_phase == 0)
          stand_phase = 1;

        std::cout << "t: " << context.get_time();
        std::cout << "; old_stanceLeg: " << old_stanceLeg;
        std::cout << "; stanceLeg: " << input_stanceLeg;
        std::cout << "; stepcount: " << stepcount << "\n";
        gait_epoch = context.get_time();
        const auto &sw_footbody =
            plant_.GetBodyByName(input_stanceLeg > 0 ? "r_foot_x" : "l_foot_x");
        const auto &X_WSw =
            plant_.EvalBodyPoseInWorld(*plant_context_, sw_footbody);
        const auto &p_WSw = X_WSw.translation();
        p0_WSw = p_WSw;
      }
      else
      { // 如果你知道next_state内的值会先和context同步，这步是不必要的。但我认为代码不应依赖于文档没说的行为
        gait_epoch = (stand_phase == 2) ? context.get_time() - 0.3 : context.get_discrete_state(gait_epoch_id_).value()(0);
        p0_WSw = old_p0_WSw;
      }
      if (phase_input == 0 && stand_phase == 2)
      {
        stand_phase = -1;
        gait_epoch = context.get_time() - 0.28;
        std::cout << "changed to walking!\n";
      }
      stanceLeg = input_stanceLeg;

      T t_gait = context.get_time() - gait_epoch; // gait elapsed
      // eq. (5)
      T w = sqrt(g / p_StScm_W(2));
      vf_WScm(0) = p_StScm_W(0) * w * sinh(w * (Tgait - t_gait)) +
                   vx_pseudo_WScm * cosh(w * (Tgait - t_gait));
      vf_WScm(1) = p_StScm_W(1) * w * sinh(w * (Tgait - t_gait)) +
                   vy_pseudo_WScm * cosh(w * (Tgait - t_gait));
      vf_WScm(2) = 0;
      drake::Vector3<T> pf_StScm_W = drake::Vector3<T>::Zero();
      pf_StScm_W(0) = p_StScm_W(0) * cosh(w * (Tgait - t_gait)) +
                      1 / w * vx_pseudo_WScm * sinh(w * (Tgait - t_gait));
      pf_StScm_W(1) = p_StScm_W(1) * cosh(w * (Tgait - t_gait)) +
                      1 / w * vy_pseudo_WScm * sinh(w * (Tgait - t_gait));
      pf_StScm_W(2) = 0;
      // eq. (13)
      // drake::Vector2<T> vnextf;
      vnextf(0) = vtarget(0);
      vnextf(1) = vtarget(1) + -stanceLeg * (-stepWidth * sqrt(g / H) *
                                             sinh(sqrt(g / H) * Tgait) /
                                             (1 + cosh(sqrt(g / H) * Tgait)));
      vnextf(2) = 0;
      const auto &torso_body = plant_.GetBodyByName("torso");
      const auto &X_WTorso =
          plant_.EvalBodyPoseInWorld(*plant_context_, torso_body);
      T yaw_WTorso = X_WTorso.rotation().ToRollPitchYaw().yaw_angle();
      drake::Vector3<T> vnextf_W = drake::math::RotationMatrix<T>::MakeZRotation(
                                       yaw_WTorso + turn_rps * (Tgait - t_gait)) *
                                   vnextf;
      // eq. (12)
      drake::Vector3<T> pf_SwScm_W = drake::Vector3<T>::Zero();
      pf_SwScm_W.head(2) = (vnextf_W.head(2) - cosh(w * Tgait) * vf_WScm.head(2)) /
                           (w * sinh(w * Tgait));
      drake::Vector3<T> pf_WScm = p_WSt + pf_StScm_W;
      pf_WSw = -pf_SwScm_W + pf_WScm;
      pf_WSw(2) = 0; // 只是好看
      // eq. (16)
      T s = t_gait / Tgait;
      T s_sat = std::clamp(s, T(0), T(1));

      if (stand_phase == 1 && s_sat > 0.6)
      {
        stand_phase = 2;
        const auto &torso_body = plant_.GetBodyByName("torso");
        const auto &lfoot_body = plant_.GetBodyByName("l_foot_x");
        const auto &rfoot_body = plant_.GetBodyByName("r_foot_x");

        const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_, lfoot_body);
        const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_, rfoot_body);
        drake::Vector3<T> p_Wmid = (X_WLfoot.translation() + X_WRfoot.translation()) / 2;

        drake::Vector3<T> p_Bmid;
        p_Bmid = X_WSt.inverse().rotation() * (p_Wmid - X_WSt.translation());
        //   drake::Vector3<T> vec = {xf_WSw,0,0};
        // p_Bmid(0) +=  0.01;
        // p_Bmid(1) +=  0.02;
        p_Wmid = X_WSt.rotation() * p_Bmid + X_WSt.translation();

        const auto &rot_WSt = X_WSt.rotation().ToRollPitchYaw();
        stand_pose << 0, 10 * M_PI / 180, rot_WSt.yaw_angle(), p_Wmid(0), p_Wmid(1), p0_WSw(2);
      }
      if (stand_phase != 2)
      {
        h_ref(0) = 0;
        h_ref(1) = 10 * M_PI / 180;
        h_ref(2) = yaw_WTorso; // euler_WTorso
        h_ref(3) = p_StScm_Fst(0);
        h_ref(4) = H; // p_WScm_Fst[x,z]
        h_ref(5) = 0;
        h_ref(6) = yaw_WTorso; // euler_WSw
        h_ref(7) = p0_WSw(0)   // p_WSw
                   + (pf_WSw(0) - p0_WSw(0)) * 0.5 * (1 - cos(M_PI * s_sat));
        h_ref(8) =
            p0_WSw(1) + (pf_WSw(1) - p0_WSw(1)) * 0.5 * (1 - cos(M_PI * s_sat));
        h_ref(9) = p0_WSw(2) + 4 * s * (1 - s) * (z_cl);

        hd_ref(0) = 0;
        hd_ref(1) = 0;
        hd_ref(2) = turn_rps;
        hd_ref(3) = v_pseudo_WScm_Fst(0);
        hd_ref(4) = 0;
        hd_ref(5) = 0;
        hd_ref(6) = turn_rps;
        hd_ref(7) =
            (pf_WSw(0) - p0_WSw(0)) * 0.5 * (M_PI / Tgait) * sin(M_PI * s_sat);
        hd_ref(8) =
            (pf_WSw(1) - p0_WSw(1)) * 0.5 * (M_PI / Tgait) * sin(M_PI * s_sat);
        hd_ref(9) = 8 * (0.5 - s) / Tgait * (z_cl);

        hdd_ref.head(3).fill(0);
        hdd_ref(3) = w * w * p_StScm_Fst(0);
        hdd_ref(4) = 0;
        hdd_ref(5) = 0;
        hdd_ref(6) = 0;
        hdd_ref(7) = (pf_WSw(0) - p0_WSw(0)) * 0.5 * (M_PI / Tgait) * (M_PI / Tgait) *
                     cos(M_PI * s_sat);
        hdd_ref(8) = (pf_WSw(1) - p0_WSw(1)) * 0.5 * (M_PI / Tgait) * (M_PI / Tgait) *
                     cos(M_PI * s_sat);
        hdd_ref(9) = -8 / Tgait / Tgait * (z_cl);

        if (stand_phase == 1)
        {
          h_ref(9) = p0_WSw(2);
          hd_ref(9) = 0;
          hdd_ref(9) = 0;
        }
      }
      else
      {
        stanceLeg = 0;

        const auto &torso_body = plant_.GetBodyByName("torso");
        const auto &lfoot_body = plant_.GetBodyByName("l_foot_x");
        const auto &rfoot_body = plant_.GetBodyByName("r_foot_x");

        const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_, lfoot_body);
        const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_, rfoot_body);
        const auto &p_mid = (X_WLfoot.translation() + X_WRfoot.translation()) / 2;

        h_ref.fill(0);
        // h_ref(0) = stand_pose[0]; h_ref(1) = 0; h_ref(2) = 0;
        // h_ref(3) = p_mid(0); h_ref(4) = p_mid(1);
        h_ref.segment(0, 5) << stand_pose.segment(0, 5);
        // h_ref(4) = stand_pose[4] + 0.02;
        h_ref(5) = stand_pose[5] + 0.56;

        hd_ref.fill(0);
        hdd_ref.fill(0);
      }

      const auto end = std::chrono::steady_clock::now();
      const std::chrono::duration calc_elapsed_ns = end - start;
      using namespace std::chrono_literals;
      calc_elapsed = calc_elapsed_ns / 1us;
      begin_calc = start.time_since_epoch() / 1us;
      end_calc = end.time_since_epoch() / 1us;
    }
    template <typename T>
    void AlipPlanner<T>::phaseOutput(const drake::systems::Context<double> &context,
                                     drake::systems::BasicVector<double> *output) const
    {
      auto output_vector = output->get_mutable_value();
      output_vector << planner_phase;
    }
    template class ::my_drake_examples::biped::AlipPlanner<double>;

  } // namespace biped
} // namespace my_drake_examples
