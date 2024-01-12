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
    class JumpPlanner final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JumpPlanner)

      JumpPlanner(double time_step, double offset,
                  const drake::multibody::MultibodyPlant<T> &);

      const drake::systems::InputPort<T> &get_left_stance_input_port() const
      {
        return *left_stance_input_port_;
      }

      const drake::systems::InputPort<T> &get_phase_input_port() const
      {
        return *phase_input_port_;
      }

      const drake::systems::InputPort<T> &get_right_stance_input_port() const
      {
        return *right_stance_input_port_;
      }

      const drake::systems::InputPort<T> &get_q_input_port() const
      {
        return *q_input_port_;
      }

      const drake::systems::InputPort<T> &get_v_input_port() const
      {
        return *v_input_port_;
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

      const drake::systems::OutputPort<T> &get_left_stance_output_port() const
      {
        return *left_stance_output_port_;
      }

      const drake::systems::OutputPort<T> &get_right_stance_output_port() const
      {
        return *right_stance_output_port_;
      }
      void setinitialPos(const Eigen::VectorXd &q0) const
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
        stand_pos << 0, 10 * M_PI / 180, euler_body[2], initial_pose.segment(4, 2), p_WSw0[2];
        std::cout << "set stand pos " << stand_pos.segment(0, 3).transpose() * 180 / M_PI << stand_pos.segment(3, 3).transpose() << std::endl;
      }

    private:
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::DiscreteValues<T> *next_state) const;

      const double time_step_{0};
      int nh_{16};
      int nq_;
      int nv_;
      const double g{9.81};
      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *left_stance_input_port_{nullptr};
      const drake::systems::InputPort<T> *right_stance_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::InputPort<T> *phase_input_port_{nullptr};
      const drake::systems::OutputPort<T> *h_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *hd_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *hdd_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *left_stance_output_port_{nullptr};
      const drake::systems::OutputPort<T> *right_stance_output_port_{nullptr};
      drake::systems::DiscreteStateIndex gait_epoch_id_;
      drake::systems::DiscreteStateIndex left_stance_id_;
      drake::systems::DiscreteStateIndex right_stance_id_;
      drake::systems::DiscreteStateIndex h_ref_id_;
      drake::systems::DiscreteStateIndex hd_ref_id_;
      drake::systems::DiscreteStateIndex hdd_ref_id_;
      mutable Eigen::VectorXd stand_pos;
      mutable bool jump_done{false};
      mutable bool stand_state{true};
      mutable bool is_initialed{false};
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::JumpPlanner)

  } // namespace biped
} // namespace my_drake_examples
