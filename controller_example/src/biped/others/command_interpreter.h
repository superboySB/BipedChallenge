#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>

namespace my_drake_examples
{
  namespace biped
  {

    class CommandInterpreter final : public drake::systems::LeafSystem<double>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(CommandInterpreter)

      CommandInterpreter();

      const drake::systems::InputPort<double> &get_axes_input_port() const
      {
        return *axes_input_port_;
      }

      const drake::systems::InputPort<double> &get_buttons_input_port() const
      {
        return *buttons_input_port_;
      }

      const drake::systems::InputPort<double> &get_hats_input_port() const
      {
        return *hats_input_port_;
      }

      const drake::systems::OutputPort<double> &get_vtarget_output_port() const
      {
        return *vtarget_output_port_;
      }

      const drake::systems::OutputPort<double> &get_H_output_port() const
      {
        return *H_output_port_;
      }

      const drake::systems::OutputPort<double> &get_stepWidth_output_port() const
      {
        return *stepWidth_output_port_;
      }

      const drake::systems::OutputPort<double> &get_gait_period_output_port() const
      {
        return *gait_period_output_port_;
      }

      const drake::systems::OutputPort<double> &get_turn_rps_output_port() const
      {
        return *turn_rps_output_port_;
      }

    private:
      void CalcVtarget(const drake::systems::Context<double> &context,
                       drake::systems::BasicVector<double> *output) const;
      void CalcH(const drake::systems::Context<double> &context,
                 drake::systems::BasicVector<double> *output) const;
      void CalcStepWidth(const drake::systems::Context<double> &context,
                         drake::systems::BasicVector<double> *output) const;
      void CalcGaitPeriod(const drake::systems::Context<double> &context,
                          drake::systems::BasicVector<double> *output) const;
      void CalcTurnRps(const drake::systems::Context<double> &context,
                       drake::systems::BasicVector<double> *output) const;

      const drake::systems::InputPort<double> *axes_input_port_{nullptr};
      const drake::systems::InputPort<double> *buttons_input_port_{nullptr};
      const drake::systems::InputPort<double> *hats_input_port_{nullptr};

      const drake::systems::OutputPort<double> *vtarget_output_port_{nullptr};
      const drake::systems::OutputPort<double> *H_output_port_{nullptr};
      const drake::systems::OutputPort<double> *stepWidth_output_port_{nullptr};
      const drake::systems::OutputPort<double> *gait_period_output_port_{nullptr};
      const drake::systems::OutputPort<double> *turn_rps_output_port_{nullptr};
    };

  } // namespace biped
} // namespace my_drake_examples
