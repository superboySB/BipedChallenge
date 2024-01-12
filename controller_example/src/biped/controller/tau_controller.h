#pragma once

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/framework/context.h>
#include <drake/systems/framework/leaf_system.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class TauController final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TauController)

      TauController(double time_step, double offset,
                    const drake::multibody::MultibodyPlant<T> &);

      const drake::systems::InputPort<T> &get_h_ref_input_port() const
      {
        return *h_ref_input_port_;
      }

      const drake::systems::InputPort<T> &get_hd_ref_input_port() const
      {
        return *hd_ref_input_port_;
      }

      const drake::systems::InputPort<T> &get_hdd_ref_input_port() const
      {
        return *hdd_ref_input_port_;
      }

      const drake::systems::InputPort<T> &get_left_stance_input_port() const
      {
        return *left_stance_input_port_;
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

      const drake::systems::OutputPort<T> &get_torque_output_port() const
      {
        return *torque_output_port_;
      }

    private:
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::DiscreteValues<T> *next_state) const;

      const double time_step_{0};
      int nh_{16};
      int nq_;
      int nv_;
      int na_;
      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *h_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hdd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *left_stance_input_port_{nullptr};
      const drake::systems::InputPort<T> *right_stance_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::OutputPort<T> *torque_output_port_{nullptr};
      drake::systems::DiscreteStateIndex torque_id_;
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
        class ::my_drake_examples::biped::TauController)

  } // namespace biped
} // namespace my_drake_examples
