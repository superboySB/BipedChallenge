#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>
#include "utils.h"
namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class StairPlanner final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(StairPlanner)

      StairPlanner(double time_step, double offset, const drake::multibody::MultibodyPlant<T> &);

      template <typename U>
      explicit StairPlanner(const StairPlanner<U> &);

      double time_step() const { return time_step_; }

      const drake::systems::InputPort<T> &get_stanceLeg_input_port() const
      {
        return *stanceLeg_input_port_;
      }

      const drake::systems::InputPort<T> &get_q_input_port() const
      {
        return *q_input_port_;
      }

      const drake::systems::InputPort<T> &get_v_input_port() const
      {
        return *v_input_port_;
      }

      const drake::systems::InputPort<T> &get_vtarget_input_port() const
      {
        return *vtarget_input_port_;
      }

      const drake::systems::InputPort<T> &get_H_input_port() const
      {
        return *H_input_port_;
      }

      const drake::systems::InputPort<T> &get_stepWidth_input_port() const
      {
        return *stepWidth_input_port_;
      }

      const drake::systems::InputPort<T> &get_gait_period_input_port() const
      {
        return *gait_period_input_port_;
      }

      const drake::systems::InputPort<T> &get_turn_rps_input_port() const
      {
        return *turn_rps_input_port_;
      }

      const drake::systems::OutputPort<T> &get_h_ref_output_port() const
      {
        return *h_ref_output_port_;
      }

      const drake::systems::OutputPort<T> &get_hd_ref_output_port() const
      {
        return *hd_ref_output_port_;
      }

      const drake::systems::OutputPort<T> &get_hdd_ref_output_port() const
      {
        return *hdd_ref_output_port_;
      }
      const drake::systems::InputPort<T> &get_phase_input_port() const
      {
        return *phase_input_port_;
      }
      const drake::systems::OutputPort<T> &get_phase_output_port() const
      {
        return *phase_output_port;
      }

      const drake::systems::OutputPort<T> &get_stanceLeg_output_port() const
      {
        return *stanceLeg_output_port_;
      }
      void setinitialPos(Eigen::VectorXd &q0)
      {
        Eigen::Ref<const Eigen::VectorXd> q0_ref = q0;
        plant_.SetPositions(plant_context_.get(), q0_ref);
        const auto &sw_footbody =
            plant_.GetBodyByName("l_foot_x");
        const auto &X_WSw =
            plant_.EvalBodyPoseInWorld(*plant_context_, sw_footbody);
        const auto &p_WSw0 = X_WSw.translation();
        const auto &r_WSw0 = X_WSw.rotation();
        std::cout << "p_WSw0:" << p_WSw0.transpose() << std::endl;

        Eigen::VectorXd initial_pose = q0.segment(0, 7);
        Eigen::Quaterniond body_quat(initial_pose[0], initial_pose[1], initial_pose[2], initial_pose[3]);
        Eigen::Vector3d euler_body = body_quat.toRotationMatrix().eulerAngles(2, 0, 1);
        stableQuatToEuler(body_quat, euler_body);
        stand_pose << 0, 10 * M_PI / 180, euler_body[2], initial_pose.segment(4, 2), p_WSw0[2];
        std::cout << "set stand pos " << stand_pose.segment(0, 3).transpose() * 180 / M_PI << stand_pose.segment(3, 3).transpose() << std::endl;
      }

    private:
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::DiscreteValues<T> *next_state) const;
      void phaseOutput(const drake::systems::Context<double> &context,
                       drake::systems::BasicVector<double> *output) const;

      const double time_step_{0};
      const double g{9.81};
      const double z_cl{0.15};
      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      T total_mass_{0};
      const drake::systems::InputPort<T> *stanceLeg_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::InputPort<T> *vtarget_input_port_{nullptr};
      const drake::systems::InputPort<T> *H_input_port_{nullptr};
      const drake::systems::InputPort<T> *stepWidth_input_port_{nullptr};
      const drake::systems::InputPort<T> *gait_period_input_port_{nullptr};
      const drake::systems::InputPort<T> *turn_rps_input_port_{nullptr};
      const drake::systems::InputPort<T> *phase_input_port_{nullptr};
      const drake::systems::OutputPort<T> *phase_output_port{nullptr};
      const drake::systems::OutputPort<T> *h_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *hd_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *hdd_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *stanceLeg_output_port_{nullptr};
      drake::systems::DiscreteStateIndex gait_epoch_id_;
      drake::systems::DiscreteStateIndex p0_WSw_id_;
      drake::systems::DiscreteStateIndex x_zmp_id_;
      drake::systems::DiscreteStateIndex x0_com_id_;
      drake::systems::DiscreteStateIndex vx0_com_id_;
      drake::systems::DiscreteStateIndex xf_WSw_id_;
      drake::systems::DiscreteStateIndex stanceLeg_id_;
      drake::systems::DiscreteStateIndex h_ref_id_;
      drake::systems::DiscreteStateIndex hd_ref_id_;
      drake::systems::DiscreteStateIndex hdd_ref_id_;
      mutable Eigen::VectorXd stand_pose;
      mutable bool is_initialed{false};
      mutable double gain_time{0};
      mutable int gait_count{0};
      mutable uint32_t step_n{0};
      mutable uint32_t stair_max{3};
      mutable int planner_phase{2};
      mutable bool stair_done{false};
      mutable int stand_phase = -1;
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::StairPlanner)

  } // namespace biped
} // namespace my_drake_examples
