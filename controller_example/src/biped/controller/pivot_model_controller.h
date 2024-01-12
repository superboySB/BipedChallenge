#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class PivotModelController : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(PivotModelController)

      double time_step() const { return time_step_; }

      PivotModelController(double time_step, double offset, const drake::multibody::MultibodyPlant<T> &);

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

      const drake::systems::OutputPort<T> &get_torque_output_port() const
      {
        return *torque_output_port_;
      }

    private:
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::DiscreteValues<T> *next_state) const;

      const double time_step_{0};
      const drake::multibody::MultibodyPlant<T> &float_plant_;
      std::unique_ptr<drake::systems::Context<T>> float_context_;
      const drake::multibody::MultibodyPlant<T> &left_stance_plant_;
      std::unique_ptr<drake::systems::Context<T>> left_stance_context_;
      const drake::multibody::MultibodyPlant<T> &right_stance_plant_;
      std::unique_ptr<drake::systems::Context<T>> right_stance_context_;
      const drake::systems::InputPort<T> *h_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hdd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *stanceLeg_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::OutputPort<T> *torque_output_port_{nullptr};

      int torque_id_{0};
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::PivotModelController)

  } // namespace biped
} // namespace my_drake_examples
