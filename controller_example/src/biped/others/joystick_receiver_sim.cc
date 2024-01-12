#include "joystick_receiver.h"
#include "command_interpreter.h"

#include <iostream>
#include <fstream>

#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>

namespace my_drake_examples
{
  namespace biped
  {
    namespace
    {

      int do_main()
      {
        drake::systems::DiagramBuilder<double> builder;
        auto receiver = builder.AddSystem<JoystickReceiver>(0.04);
        auto interpreter = builder.AddSystem<CommandInterpreter>();
        builder.Connect(receiver->get_axes_output_port(), interpreter->get_axes_input_port());
        builder.Connect(receiver->get_buttons_output_port(), interpreter->get_buttons_input_port());
        builder.Connect(receiver->get_hats_output_port(), interpreter->get_hats_input_port());
        auto vtarget_logger = drake::systems::LogVectorOutput(interpreter->get_vtarget_output_port(), &builder, 0);
        auto diagram = builder.Build();
        drake::systems::Simulator<double> simulator(*diagram);
        simulator.set_target_realtime_rate(1.0);
        simulator.Initialize();
        simulator.AdvanceTo(10.0);

        const auto &log = vtarget_logger->FindLog(simulator.get_context());
        for (int n = 0; n < log.sample_times().size(); n++)
        {
          const double t = log.sample_times()[n];
          std::cout << n << ": "
                    << "p0 out: " << log.data().col(n).transpose()
                    << " (" << t << ")\n";
        }
        return 0;
      }

    } // namespace
  }   // namespace biped
} // namespace my_drake_examples

int main(int argc, char **argv)
{
  return my_drake_examples::biped::do_main();
}
