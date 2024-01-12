#include "swing_arm_planner.h"

#include <drake/multibody/math/spatial_algebra.h>

#include <chrono>
#include <iostream>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    SwingArmPlanner<T>::SwingArmPlanner(
        double time_step, double offset,
        const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), plant_(plant)
    {
      DRAKE_THROW_UNLESS(time_step > 0);
      nv_ = plant_.num_velocities();
      nq_ = plant_.num_positions();
      plant_context_ = plant_.CreateDefaultContext();

      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", nq_);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", nv_);

      gait_epoch_id_ = this->DeclareDiscreteState(1);

      stance_id_ = this->DeclareDiscreteState(2);
      stance_output_port_ = &this->DeclareStateOutputPort("stance", stance_id_);

      h_ref_id_ = this->DeclareDiscreteState(nh_);
      hd_ref_id_ = this->DeclareDiscreteState(nh_);
      hdd_ref_id_ = this->DeclareDiscreteState(nh_);
      h_ref_output_port_ = &this->DeclareStateOutputPort("h_ref", h_ref_id_);
      hd_ref_output_port_ = &this->DeclareStateOutputPort("hd_ref", hd_ref_id_);
      hdd_ref_output_port_ = &this->DeclareStateOutputPort("hdd_ref", hdd_ref_id_);

      this->DeclarePeriodicDiscreteUpdateEvent(time_step, offset,
                                               &SwingArmPlanner::Update);
      stand_pos.resize(6);
      stand_pos.setZero();
    }

    template <typename T>
    void SwingArmPlanner<T>::Update(
        const drake::systems::Context<T> &context,
        drake::systems::DiscreteValues<T> *next_state) const
    {
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);
      const T T_gait = 1.0;

      auto &left_stance = next_state->get_mutable_value(stance_id_)(0);
      auto &right_stance = next_state->get_mutable_value(stance_id_)(1);

      const auto &old_gait_epoch =
          context.get_discrete_state(gait_epoch_id_).value()(0);
      auto &gait_epoch = next_state->get_mutable_value(gait_epoch_id_)(0);

      auto h_ref = next_state->get_mutable_value(h_ref_id_);
      auto hd_ref = next_state->get_mutable_value(hd_ref_id_);
      auto hdd_ref = next_state->get_mutable_value(hdd_ref_id_);

      plant_.SetPositions(plant_context_.get(), q);
      plant_.SetVelocities(plant_context_.get(), v);

      T old_t_gait = context.get_time() - old_gait_epoch;
      // 事实上planner context中左右脚的状态总是相同的
      if (old_t_gait > T_gait)
      {
        gait_epoch = context.get_time();
      }
      else
      {
        gait_epoch = old_gait_epoch;
      }
      left_stance = 1;
      right_stance = 1;
      T t_gait = context.get_time() - gait_epoch;
      const T A = 0;
      T s = t_gait / T_gait;
      const T w = 2 * M_PI / T_gait;
      h_ref.setZero();
      h_ref.segment(0, 6) = stand_pos;
      // h_ref(1) = 10*M_PI/180;
      // h_ref(3) = stand_pos[0];
      // h_ref(4) = stand_pos[1];
      // h_ref(5) = stand_pos[2];
      // h_ref.segment(6,18)<<-0.005795, -0.03286, -1.492, 2.075, -0.7569,
      //       0.005606, 0.03178, -1.492, 2.074, -0.7569,
      //       0, 0, 0, 0, 0, 0, 0, 0;
      // h_ref(5) = 0.6;
      // h_ref(16) = A * sin(2 * M_PI * s);
      // h_ref(20) = -A * sin(2 * M_PI * s);

      hd_ref.setZero();
      hd_ref(16) = A * w * cos(2 * M_PI * s);
      hd_ref(20) = -A * w * cos(2 * M_PI * s);

      hdd_ref.setZero();
      hdd_ref(16) = A * w * w * -sin(2 * M_PI * s);
      hdd_ref(20) = -A * w * w * -sin(2 * M_PI * s);
    }

    DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::SwingArmPlanner)

  } // namespace biped
} // namespace my_drake_examples
