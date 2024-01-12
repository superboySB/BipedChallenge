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
#include <drake/systems/primitives/demultiplexer.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <gflags/gflags.h>
#include "tcp/include/isaac_sim_system.hpp"

#include <iostream>
#include <string>

#include "controller/whole_body_controller.h"
#include "planner/swing_arm_planner.h"

namespace my_drake_examples
{
  namespace biped
  {
    namespace
    {

      DEFINE_bool(drake_vis, false, "connect DrakeVisualizer to defualt lcm.");

      using drake::common::CallPython;
      using drake::common::ToPythonKwargs;
      using drake::common::ToPythonTuple;
      using drake::systems::LogVectorOutput;
      IsaacSimulator<double> *IsaacSimulator_ptr;
      void signalHandler(int signal)
      {
        std::cout << "Signal " << signal << std::endl;
        IsaacSimulator_ptr->close_server();
        exit(signal);
      }

      int do_main()
      {
        drake::systems::DiagramBuilder<double> builder;
        // auto lcm_log = drake::lcm::DrakeLcmLog("file_name", true);
        drake::systems::lcm::LcmInterfaceSystem *lcm;
        auto [plant, scene_graph] =
            drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0005);
        drake::multibody::Parser parser(&plant, &scene_graph);
        const std::string relative_name =
            "models/biped_v2/urdf/biped_v2_contact.urdf";
        parser.AddModelFromFile(relative_name);
        const double static_friction = 1.0;
        const drake::Vector4<double> gray(0.8, 0.8, 0.8, 0.8);
        plant.RegisterVisualGeometry(
            plant.world_body(), drake::math::RigidTransformd(),
            drake::geometry::HalfSpace(), "GroundVisualGeometry", gray);
        const drake::multibody::CoulombFriction<double> ground_friction(
            static_friction, static_friction);
        plant.RegisterCollisionGeometry(
            plant.world_body(), drake::math::RigidTransformd(),
            drake::geometry::HalfSpace(), "GroundCollisionGeometry", ground_friction);
        plant.Finalize();
        if (FLAGS_drake_vis)
        {
          lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();
          drake::geometry::DrakeVisualizerd::AddToBuilder(&builder, scene_graph, lcm);
        }
        auto meshcat = std::make_shared<drake::geometry::Meshcat>();
        auto &meshcat_vis = drake::geometry::MeshcatVisualizerd::AddToBuilder(
            &builder, scene_graph, meshcat);
        drake::multibody::meshcat::ContactVisualizerParams cparams;
        cparams.newtons_per_meter = 300.0;
        drake::multibody::meshcat::ContactVisualizerd::AddToBuilder(
            &builder, plant, meshcat, std::move(cparams));

        std::vector<int> out_sizes{25, 24};
        auto demuxer = builder.AddSystem<drake::systems::Demultiplexer>(out_sizes);
        auto planner = builder.AddSystem<SwingArmPlanner>(0.001, 0.001, plant);
        auto controller = builder.AddSystem<WholeBodyController>(0.001, 0.002, plant);
        auto isaac_simulator = builder.AddSystem<IsaacSimulator>(0.001, 0.000, plant);
        IsaacSimulator_ptr = isaac_simulator;
        builder.Connect(planner->get_h_ref_output_port(),
                        controller->get_h_ref_input_port());
        builder.Connect(planner->get_hd_ref_output_port(),
                        controller->get_hd_ref_input_port());
        builder.Connect(planner->get_hdd_ref_output_port(),
                        controller->get_hdd_ref_input_port());
        builder.Connect(planner->get_stance_output_port(),
                        controller->get_stance_input_port());

        builder.Connect(controller->get_torque_output_port(),
                        plant.get_actuation_input_port());
        builder.Connect(controller->get_torque_output_port(),
                        isaac_simulator->get_actuation_input_port());

        builder.Connect(isaac_simulator->get_state_output_port(), demuxer->get_input_port());

        builder.Connect(demuxer->get_output_port(0), planner->get_q_input_port());
        builder.Connect(demuxer->get_output_port(1), planner->get_v_input_port());
        builder.Connect(demuxer->get_output_port(0), controller->get_q_input_port());
        builder.Connect(demuxer->get_output_port(1), controller->get_v_input_port());

        auto q_logger = LogVectorOutput(demuxer->get_output_port(0), &builder);
        auto v_logger = LogVectorOutput(demuxer->get_output_port(0), &builder);

        auto diagram = builder.Build();
        Eigen::VectorXd initial_pose(7);
        isaac_simulator->get_initial_pose(initial_pose);

        drake::systems::Simulator<double> simulator(*diagram);
        drake::VectorX<double> q0 = drake::VectorX<double>::Zero(25);
        q0 << initial_pose.segment(0, 7),
            -0.005795, -0.03286, -1.492, 2.075, -0.7569,
            0.005606, 0.03178, -1.492, 2.074, -0.7569,
            0, 0, 0, 0, 0, 0, 0, 0; // clang-format on

        auto &plant_context = diagram->GetMutableSubsystemContext(
            plant, &simulator.get_mutable_context());
        plant.SetPositions(&plant_context, q0);
        planner->setinitialPos(q0);
        meshcat_vis.StartRecording();
        simulator.set_target_realtime_rate(1.0);
        simulator.Initialize();
        try
        {
          simulator.AdvanceTo(10.0);
        }
        catch (const std::runtime_error &e)
        {
          std::cerr << e.what() << std::endl;
        }
        meshcat_vis.StopRecording();
        meshcat_vis.PublishRecording();

        const auto &q_log = q_logger->FindLog(simulator.get_context());
        const auto &v_log = v_logger->FindLog(simulator.get_context());

        CallPython("figure", 1);
        CallPython("clf");
        CallPython("subplots_adjust", 0.05, 0.05, 0.95, 0.95,
                   ToPythonKwargs("hspace", 0.3));

        CallPython("subplot", 1, 1, 1);
        for (int i = 17; i < 25; i++)
        {
          CallPython("plot", q_log.sample_times(), q_log.data().row(i).transpose(),
                     ToPythonKwargs("label", fmt::format("q({})", i)));
        }

        CallPython("title", "joint q");
        CallPython("legend");
        CallPython("wait");

        return 0;
      }

    } // namespace
  }   // namespace biped
} // namespace my_drake_examples

int main(int argc, char **argv)
{
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  return my_drake_examples::biped::do_main();
}
