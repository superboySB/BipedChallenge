#include <drake/common/eigen_types.h>
#include <drake/common/proto/call_python.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/lcm/drake_lcm_log.h>
#include <drake/multibody/meshcat/contact_visualizer.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/systems/lcm/lcm_scope_system.h>
#include <drake/systems/primitives/affine_system.h>
#include <drake/systems/primitives/saturation.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/systems/sensors/rotary_encoders.h>
#include <gflags/gflags.h>
#include <sched.h>

#include <iostream>
#include <string>

#include "controller/feedback_controller.h"
#include "controller/feedback_controller_async.h"
#include "estimator/dummy_estimator.h"
#include "estimator/mit_estimator.h"
#include "others/command_interpreter.h"
#include "others/imu.h"
#include "others/joystick_receiver.h"
#include "planner/alip_planner.h"
#include "tcp/include/tcp_server.hpp"
#include "tcp/include/data_converter.hpp"
#include "tcp/include/json.hpp"

namespace my_drake_examples
{
    namespace biped
    {
        namespace
        {

            class BipedCtrlServer : public TcpServer
            {
            public:
                /**
                 * Constructor
                 *
                 * @param host host IP
                 * @param port port number
                 */
                BipedCtrlServer(const std::string &host_ip, const unsigned int port) : TcpServer(host_ip, port)
                {
                }

            private:
                /**
                 * Message processing callback function
                 *
                 * This function will be called when client sending a request
                 *
                 * @param msg Message received from client, i.e., controller input
                 */
                std::string processMsg_(const char *msg)
                {
                    // parse input message to JSON format
                    nlohmann::json jmsg = bin2json(msg);

                    // print message, i.e., controller input
                    // std::cout << jmsg.dump(4) << std::endl;
                    std::cout << " processMsg_\n";

                    nlohmann::json resp;
                    resp["content"] = "Hello from TCP Server";

                    // return controller output
                    // the output will be automatically send back to the client
                    return json2binstr(resp);
                }
            };

        } // namespace
    }     // namespace biped
} // namespace my_drake_examples

using my_drake_examples::biped::BipedCtrlServer;

int main(int argc, char **argv)
{
    BipedCtrlServer server("0.0.0.0", 8800);
    server.launch();

    return 0;
}
