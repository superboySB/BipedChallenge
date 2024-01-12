#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <iostream>
namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class JumpEstimator final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JumpEstimator)

      JumpEstimator(const drake::multibody::MultibodyPlant<T> &);

      template <typename U>
      explicit JumpEstimator(const JumpEstimator<U> &);

      const drake::systems::InputPort<T> &get_state_input_port() const
      {
        return *state_input_port_;
      }

      const drake::systems::OutputPort<T> &get_q_output_port() const
      {
        return *q_output_port_;
      }

      const drake::systems::OutputPort<T> &get_v_output_port() const
      {
        return *v_output_port_;
      }

      const drake::systems::OutputPort<T> &get_left_stance_output_port() const
      {
        return *left_stance_output_port_;
      }

      const drake::systems::OutputPort<T> &get_right_stance_output_port() const
      {
        return *right_stance_output_port_;
      }
      void getGroundHeight() const
      {
        if (!is_initial)
        {
          const auto &sw_footbody =
              plant_.GetBodyByName("l_foot_x");
          const auto &X_WSw =
              plant_.EvalBodyPoseInWorld(*plant_context_, sw_footbody);
          const auto &p_WSw0 = X_WSw.translation();
          std::cout << "ground_height:" << p_WSw0[2] << std::endl;
          ground_height = p_WSw0[2];
        }
        is_initial = true;
      }

    private:
      void CalcQ(const drake::systems::Context<T> &context,
                 drake::systems::BasicVector<T> *output) const;
      void CalcV(const drake::systems::Context<T> &context,
                 drake::systems::BasicVector<T> *output) const;
      void CalcLeftStance(const drake::systems::Context<T> &context,
                          drake::systems::BasicVector<T> *output) const;
      void CalcRightStance(const drake::systems::Context<T> &context,
                           drake::systems::BasicVector<T> *output) const;

      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *state_input_port_;
      const drake::systems::OutputPort<T> *q_output_port_;
      const drake::systems::OutputPort<T> *v_output_port_;
      const drake::systems::OutputPort<T> *left_stance_output_port_;
      const drake::systems::OutputPort<T> *right_stance_output_port_;
      mutable double ground_height{0};
      mutable bool is_initial{false};
    };
    // DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    //     class ::my_drake_examples::biped::JumpEstimator)
    extern template class ::my_drake_examples::biped::JumpEstimator<double>;

  } // namespace biped
} // namespace my_drake_examples
