#include "command_interpreter.h"

// #include <drake/xxx/xxx.h>

namespace my_drake_examples
{
  namespace biped
  {

    CommandInterpreter::CommandInterpreter()
    {
      axes_input_port_ = &this->DeclareVectorInputPort("axes", 6);
      buttons_input_port_ = &this->DeclareVectorInputPort("buttons", 11);
      hats_input_port_ = &this->DeclareVectorInputPort("hats", 1);

      vtarget_output_port_ = &this->DeclareVectorOutputPort(
          "vtarget", 2, &CommandInterpreter::CalcVtarget);
      H_output_port_ = &this->DeclareVectorOutputPort(
          "H", 1, &CommandInterpreter::CalcH);
      stepWidth_output_port_ = &this->DeclareVectorOutputPort(
          "stepWidth", 1, &CommandInterpreter::CalcStepWidth);
      gait_period_output_port_ = &this->DeclareVectorOutputPort(
          "gait_period", 1, &CommandInterpreter::CalcGaitPeriod);
      turn_rps_output_port_ = &this->DeclareVectorOutputPort(
          "turn_rps", 1, &CommandInterpreter::CalcTurnRps);
    }

    void CommandInterpreter::CalcVtarget(const drake::systems::Context<double> &context,
                                         drake::systems::BasicVector<double> *output) const
    {
      const auto &axes = get_axes_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector(1) = 0.5 * -axes(0) / 32768;
      output_vector(0) = 0.5 * -axes(1) / 32768;
    }

    void CommandInterpreter::CalcH(const drake::systems::Context<double> &context,
                                   drake::systems::BasicVector<double> *output) const
    {
      const auto &axes = get_axes_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector(0) = 0.55 + 0.1 * -axes(4) / 32768; // 设计为带矩的更好，后续容易改
    }

    void CommandInterpreter::CalcStepWidth(const drake::systems::Context<double> &context,
                                           drake::systems::BasicVector<double> *output) const
    {
      const auto &axes = get_axes_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector(0) = 0.1; // 开摆！
    }

    void CommandInterpreter::CalcGaitPeriod(const drake::systems::Context<double> &context,
                                            drake::systems::BasicVector<double> *output) const
    {
      const auto &axes = get_axes_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector(0) = 0.4;
    }

    void CommandInterpreter::CalcTurnRps(const drake::systems::Context<double> &context,
                                         drake::systems::BasicVector<double> *output) const
    {
      const auto &axes = get_axes_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector(0) = M_PI_4 * -axes(3) / 32768;
    }

  } // namespace biped
} // namespace my_drake_examples
