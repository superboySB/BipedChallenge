#include "planner/jump_planner.h"
#include "controller/tau_controller.h"
#include "estimator/jump_estimator.h"

#include <sched.h>
#include <gflags/gflags.h>

#include <iostream>
#include <string>
#include <drake/common/eigen_types.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/primitives/vector_log_sink.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/geometry/scene_graph.h>
#include <drake/systems/lcm/lcm_interface_system.h>
#include <drake/geometry/drake_visualizer.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/multibody/meshcat/contact_visualizer.h>
#include <drake/systems/lcm/lcm_scope_system.h>
#include <drake/lcm/drake_lcm_log.h>
#include <drake/common/proto/call_python.h>
#include "tcp/include/isaac_sim_system.hpp"

namespace my_drake_examples
{
  namespace biped
  {
    namespace
    {

      DEFINE_bool(drake_vis, false,
                  "connect DrakeVisualizer to defualt lcm.");

      using drake::common::CallPython;
      using drake::common::ToPythonKwargs;
      using drake::common::ToPythonTuple;
      using drake::systems::LogVectorOutput;

      int do_main()
      {

        drake::systems::DiagramBuilder<double> builder;
        // auto lcm_log = drake::lcm::DrakeLcmLog("file_name", true);
        drake::systems::lcm::LcmInterfaceSystem *lcm;
        auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0005);
        drake::multibody::Parser parser(&plant, &scene_graph);
        const std::string relative_name = "models/biped_v2/urdf/biped_v2_no_arm.urdf";
        parser.AddModelFromFile(relative_name);
        const double static_friction = 1.0;
        const drake::Vector4<double> gray(0.8, 0.8, 0.8, 0.8);
        plant.RegisterVisualGeometry(plant.world_body(), drake::math::RigidTransformd(),
                                     drake::geometry::HalfSpace(), "GroundVisualGeometry",
                                     gray);
        const drake::multibody::CoulombFriction<double> ground_friction(static_friction,
                                                                        static_friction);
        plant.RegisterCollisionGeometry(plant.world_body(), drake::math::RigidTransformd(),
                                        drake::geometry::HalfSpace(),
                                        "GroundCollisionGeometry", ground_friction);
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

        auto planner = builder.AddSystem<JumpPlanner>(0.001, 0.001, plant);
        auto controller = builder.AddSystem<TauController>(0.001, 0.002, plant);
        auto isaac_simulator = builder.AddSystem<IsaacSimulator>(0.001, 0.000, plant);

        builder.Connect(planner->get_h_ref_output_port(), controller->get_h_ref_input_port());
        builder.Connect(planner->get_hd_ref_output_port(), controller->get_hd_ref_input_port());
        builder.Connect(planner->get_hdd_ref_output_port(), controller->get_hdd_ref_input_port());
        builder.Connect(planner->get_left_stance_output_port(), controller->get_left_stance_input_port());
        builder.Connect(planner->get_right_stance_output_port(), controller->get_right_stance_input_port());
        builder.Connect(isaac_simulator->get_phase_output_port(),
                        planner->get_phase_input_port());
        builder.Connect(controller->get_torque_output_port(), plant.get_actuation_input_port());
        builder.Connect(controller->get_torque_output_port(), isaac_simulator->get_actuation_input_port());
        auto estimator = builder.AddSystem<JumpEstimator>(plant);
        builder.Connect(isaac_simulator->get_state_output_port(), estimator->get_state_input_port());

        builder.Connect(estimator->get_q_output_port(), planner->get_q_input_port());
        builder.Connect(estimator->get_v_output_port(), planner->get_v_input_port());
        builder.Connect(estimator->get_left_stance_output_port(), planner->get_left_stance_input_port());
        builder.Connect(estimator->get_right_stance_output_port(), planner->get_right_stance_input_port());
        builder.Connect(estimator->get_q_output_port(), controller->get_q_input_port());
        builder.Connect(estimator->get_v_output_port(), controller->get_v_input_port());

        // drake::systems::lcm::LcmScopeSystem::AddToBuilder(
        //     &builder, &lcm_log, estimator->get_q_output_port(), "estimator.q", 0.0);
        // drake::systems::lcm::LcmScopeSystem::AddToBuilder(
        //     &builder, &lcm_log, estimator->get_v_output_port(), "estimator.v", 0.0);

        auto torque_logger = LogVectorOutput(controller->get_torque_output_port(), &builder);
        auto q_logger = LogVectorOutput(estimator->get_q_output_port(), &builder);
        auto v_logger = LogVectorOutput(estimator->get_v_output_port(), &builder);
        auto a_logger = LogVectorOutput(isaac_simulator->get_generalized_acceleration_output_port(), &builder);
        auto h_ref_logger = LogVectorOutput(planner->get_h_ref_output_port(), &builder);
        auto hd_ref_logger = LogVectorOutput(planner->get_hd_ref_output_port(), &builder);
        auto hdd_ref_logger = LogVectorOutput(planner->get_hdd_ref_output_port(), &builder);

        auto diagram = builder.Build();

        Eigen::VectorXd initial_pose(7);
        // initial_pose<<0, 0, 0.471,0.9962, -8.284e-07, 0.08716, -5.303e-08;
        isaac_simulator->get_initial_pose(initial_pose);

        drake::systems::Simulator<double> simulator(*diagram);
        drake::VectorX<double> q0 = drake::VectorX<double>::Zero(17);
        q0 << initial_pose.segment(0, 7),
            -0.005795, -0.03286, -1.492, 2.075, -0.7569,
            0.005606, 0.03178, -1.492, 2.074, -0.7569;
        planner->setinitialPos(q0);
        auto &plant_context = diagram->GetMutableSubsystemContext(plant, &simulator.get_mutable_context());
        plant.SetPositions(&plant_context, q0);
        auto &planner_context = diagram->GetMutableSubsystemContext(*planner, &simulator.get_mutable_context());
        auto left_stance = planner_context.get_mutable_discrete_state(1).get_mutable_value();
        auto right_stance = planner_context.get_mutable_discrete_state(2).get_mutable_value();
        left_stance(0) = 1;
        right_stance(0) = 1;

        meshcat_vis.StartRecording();
        simulator.set_target_realtime_rate(0.1);
        simulator.Initialize();
        try
        {
          simulator.AdvanceTo(2000.0);
        }
        catch (const std::runtime_error &e)
        {
          std::cerr << e.what() << std::endl;
        }
        meshcat_vis.StopRecording();
        meshcat_vis.PublishRecording();

        const auto &torque_log = torque_logger->FindLog(simulator.get_context());
        const auto &q_log = q_logger->FindLog(simulator.get_context());
        const auto &v_log = v_logger->FindLog(simulator.get_context());
        const auto &a_log = a_logger->FindLog(simulator.get_context());
        const auto &h_ref_log = h_ref_logger->FindLog(simulator.get_context());
        const auto &hd_ref_log = hd_ref_logger->FindLog(simulator.get_context());
        const auto &hdd_ref_log = hdd_ref_logger->FindLog(simulator.get_context());
        CallPython("figure", 1);
        CallPython("clf");
        CallPython("subplots_adjust", 0.05, 0.05, 0.95, 0.95, ToPythonKwargs("hspace", 0.3));

        CallPython("subplot", 1, 1, 1);
        CallPython("plot", torque_log.sample_times(), torque_log.data().bottomRows(10).transpose());
        CallPython("title", "joint torque");
        CallPython("legend", "0123456789");
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
