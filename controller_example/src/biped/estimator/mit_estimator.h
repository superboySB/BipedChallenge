#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class MitEstimator final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MitEstimator)

      MitEstimator(double time_step, double offset,
                   const drake::multibody::MultibodyPlant<T> &);

      const drake::systems::InputPort<T> &get_q_joint_input_port() const
      {
        return *q_joint_input_port_;
      }

      const drake::systems::InputPort<T> &get_v_joint_input_port() const
      {
        return *v_joint_input_port_;
      }

      const drake::systems::InputPort<T> &get_acc_imu_input_port() const
      {
        return *acc_imu_input_port_;
      }

      const drake::systems::InputPort<T> &get_quat_imu_input_port() const
      {
        return *quat_imu_input_port_;
      }

      // gyro 是角速度
      const drake::systems::InputPort<T> &get_gyro_imu_input_port() const
      {
        return *gyro_imu_input_port_;
      }

      // (left_stance, right_stance)
      const drake::systems::InputPort<T> &get_stance_phase_input_port() const
      {
        return *stance_phase_input_port_;
      }

      const drake::systems::OutputPort<T> &get_state_output_port() const
      {
        return *state_output_port_;
      }

      const drake::systems::OutputPort<T> &get_xhat_output_port() const
      {
        return *xhat_output_port_;
      }

    private:
      drake::systems::EventStatus Init(const drake::systems::Context<T> &,
                                       drake::systems::State<T> *) const;
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::State<T> *state) const;

      double time_step_;
      double offset_;
      int na_{0};
      int nq_{0};
      int nv_{0};
      drake::Vector3<T> g_;
      Eigen::Matrix<T, 12, 12> A_;
      Eigen::Matrix<T, 12, 3> B_;
      Eigen::Matrix<T, 12, 12> C_;
      Eigen::Matrix<T, 12, 12> Q_;
      Eigen::Matrix<T, 12, 12> R_;
      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *q_joint_input_port_;
      const drake::systems::InputPort<T> *v_joint_input_port_;
      const drake::systems::InputPort<T> *acc_imu_input_port_;
      const drake::systems::InputPort<T> *quat_imu_input_port_;
      const drake::systems::InputPort<T> *gyro_imu_input_port_;
      const drake::systems::InputPort<T> *stance_phase_input_port_;
      const drake::systems::OutputPort<T> *state_output_port_;
      const drake::systems::OutputPort<T> *xhat_output_port_;
      drake::systems::DiscreteStateIndex state_id_;
      drake::systems::DiscreteStateIndex xhat_id_;
      drake::systems::AbstractStateIndex Phat_id_;
    };

    extern template class ::my_drake_examples::biped::MitEstimator<double>;

  } // namespace biped
} // namespace my_drake_examples
