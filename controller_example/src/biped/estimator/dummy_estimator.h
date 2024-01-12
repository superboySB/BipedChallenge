#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class DummyEstimator final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(DummyEstimator)

      DummyEstimator(const drake::multibody::MultibodyPlant<T> &);

      template <typename U>
      explicit DummyEstimator(const DummyEstimator<U> &);

      const drake::systems::InputPort<T> &get_state_input_port() const
      {
        return *state_input_port_;
      }

      const drake::systems::InputPort<T> &get_a_input_port() const
      {
        return *a_input_port_;
      }

      const drake::systems::InputPort<T> &get_torque_input_port() const
      {
        return *torque_input_port_;
      }

      const drake::systems::OutputPort<T> &get_q_output_port() const
      {
        return *q_output_port_;
      }

      const drake::systems::OutputPort<T> &get_v_output_port() const
      {
        return *v_output_port_;
      }

      const drake::systems::OutputPort<T> &get_stanceLeg_output_port() const
      {
        return *stanceLeg_output_port_;
      }

      const drake::systems::OutputPort<T> &get_GRF_output_port() const
      {
        return *GRF_output_port_;
      }

    private:
      void CalcQ(const drake::systems::Context<T> &context,
                 drake::systems::BasicVector<T> *output) const;
      void CalcV(const drake::systems::Context<T> &context,
                 drake::systems::BasicVector<T> *output) const;
      void CalcStanceLeg(const drake::systems::Context<T> &context,
                         int *output) const;
      void CalcGRF(const drake::systems::Context<T> &context,
                   drake::systems::BasicVector<T> *output) const;

      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *state_input_port_;
      const drake::systems::InputPort<T> *a_input_port_;
      const drake::systems::InputPort<T> *torque_input_port_;
      const drake::systems::OutputPort<T> *q_output_port_;
      const drake::systems::OutputPort<T> *v_output_port_;
      const drake::systems::OutputPort<T> *stanceLeg_output_port_;
      const drake::systems::OutputPort<T> *GRF_output_port_;
    };
    // DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    //     class ::my_drake_examples::biped::DummyEstimator)
    extern template class ::my_drake_examples::biped::DummyEstimator<double>;

  } // namespace biped
} // namespace my_drake_examples
