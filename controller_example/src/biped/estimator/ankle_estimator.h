#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class AnkleEstimator final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AnkleEstimator)

      AnkleEstimator(const drake::multibody::MultibodyPlant<T> &);

      template <typename U>
      explicit AnkleEstimator(const AnkleEstimator<U> &);

      const drake::systems::InputPort<T> &get_state_input_port() const
      {
        return *state_input_port_;
      }

      const drake::systems::InputPort<T> &get_reaction_force_input_port() const
      {
        return *reaction_force_input_port_;
      }

      const drake::systems::OutputPort<T> &get_ankle_force_port() const
      {
        return *ankle_force_port_;
      }

    private:
      void CalcAnkleForce(const drake::systems::Context<T> &context,
                          drake::systems::BasicVector<T> *output) const;

      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *state_input_port_;
      const drake::systems::InputPort<T> *reaction_force_input_port_;
      const drake::systems::OutputPort<T> *ankle_force_port_;
    };

    extern template class ::my_drake_examples::biped::AnkleEstimator<double>;

  } // namespace biped
} // namespace my_drake_examples
