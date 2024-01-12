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
#include "planner/stair_planner.h"

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
            using drake::systems::sensors::RotaryEncoders;
            using Eigen::MatrixXd;
            using Eigen::VectorXd;

            int do_main()
            {
                sched_param param;
                param.sched_priority = 80;
                if (sched_setscheduler(0, SCHED_FIFO, &param) == -1)
                {
                    perror("sched_setscheduler");
                    // exit(1);
                }

                drake::systems::DiagramBuilder<double> builder;
                // auto lcm_log = drake::lcm::DrakeLcmLog("file_name", true);
                drake::systems::lcm::LcmInterfaceSystem *lcm;
                auto [plant, scene_graph] =
                    drake::multibody::AddMultibodyPlantSceneGraph(&builder, 0.0005);
                drake::multibody::Parser parser(&plant, &scene_graph);
                const std::string relative_name = "models/biped_v2/urdf/biped_v2_no_arm.urdf";
                parser.AddModels(relative_name);
                parser.AddModels("models/biped_v2/urdf/stair.urdf");
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
                plant.WeldFrames(plant.world_frame(), plant.GetBodyByName("boxes").body_frame(),
                                 drake::math::RigidTransformd());
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

                auto receiver = builder.AddSystem<JoystickReceiver>(0.04);
                auto interpreter = builder.AddSystem<CommandInterpreter>();
                builder.Connect(receiver->get_axes_output_port(),
                                interpreter->get_axes_input_port());
                builder.Connect(receiver->get_buttons_output_port(),
                                interpreter->get_buttons_input_port());
                builder.Connect(receiver->get_hats_output_port(),
                                interpreter->get_hats_input_port());
                auto planner = builder.AddSystem<StairPlanner>(0.001, 0.001, plant);
                builder.Connect(interpreter->get_vtarget_output_port(),
                                planner->get_vtarget_input_port());
                builder.Connect(interpreter->get_H_output_port(),
                                planner->get_H_input_port());
                builder.Connect(interpreter->get_stepWidth_output_port(),
                                planner->get_stepWidth_input_port());
                builder.Connect(interpreter->get_gait_period_output_port(),
                                planner->get_gait_period_input_port());
                builder.Connect(interpreter->get_turn_rps_output_port(),
                                planner->get_turn_rps_input_port());
                auto controller = builder.AddSystem<FeedbackController>(0.001, 0.002, plant);
                builder.Connect(planner->get_h_ref_output_port(),
                                controller->get_h_ref_input_port());
                builder.Connect(planner->get_hd_ref_output_port(),
                                controller->get_hd_ref_input_port());
                builder.Connect(planner->get_hdd_ref_output_port(),
                                controller->get_hdd_ref_input_port());
                builder.Connect(planner->get_stanceLeg_output_port(),
                                controller->get_stanceLeg_input_port());

                builder.Connect(controller->get_torque_output_port(),
                                plant.get_actuation_input_port());
                auto estimator = builder.AddSystem<DummyEstimator>(plant);
                builder.Connect(plant.get_state_output_port(),
                                estimator->get_state_input_port());
                builder.Connect(plant.get_generalized_acceleration_output_port(),
                                estimator->get_a_input_port());
                builder.Connect(controller->get_torque_output_port(),
                                estimator->get_torque_input_port());

                std::vector<int> joint_pos_ids{7, 8, 9, 10, 11, 12, 13, 14, 15, 16};
                auto joint_pos_encoder = builder.AddSystem<RotaryEncoders>(33, joint_pos_ids);
                builder.Connect(plant.get_state_output_port(),
                                joint_pos_encoder->get_input_port());
                std::vector<int> joint_vel_ids{23, 24, 25, 26, 27, 28, 29, 30, 31, 32};
                auto joint_vel_encoder = builder.AddSystem<RotaryEncoders>(33, joint_vel_ids);
                builder.Connect(plant.get_state_output_port(),
                                joint_vel_encoder->get_input_port());
                auto imu = builder.AddSystem<IMU>(
                    plant.GetBodyByName("torso"),
                    plant.GetFrameByName("Mti").GetFixedPoseInBodyFrame(),
                    plant.gravity_field().gravity_vector());
                builder.Connect(plant.get_body_poses_output_port(),
                                imu->get_input_port_body_poses());
                builder.Connect(plant.get_body_spatial_velocities_output_port(),
                                imu->get_input_port_body_velocities());
                builder.Connect(plant.get_body_spatial_accelerations_output_port(),
                                imu->get_input_port_body_accelerations());
                auto mit_esti = builder.AddSystem<MitEstimator>(0.001, 0.001, plant);
                builder.Connect(joint_pos_encoder->get_output_port(),
                                mit_esti->get_q_joint_input_port());
                builder.Connect(joint_vel_encoder->get_output_port(),
                                mit_esti->get_v_joint_input_port());
                builder.Connect(imu->get_output_port_measurement_acceleration(),
                                mit_esti->get_acc_imu_input_port());
                builder.Connect(imu->get_output_port_measurement_angular_velocity(),
                                mit_esti->get_gyro_imu_input_port());
                builder.Connect(imu->get_output_port_measurement_quaternion_orientation(),
                                mit_esti->get_quat_imu_input_port());
                MatrixXd A = MatrixXd::Zero(0, 0);
                MatrixXd B = MatrixXd::Zero(0, 1);
                VectorXd f0 = VectorXd::Zero(2);
                MatrixXd C = MatrixXd::Zero(2, 0);
                MatrixXd D(2, 1);
                D << 0.5, -0.5;
                VectorXd y0(2);
                y0 << 0.5, 0.5;
                auto affine =
                    builder.AddSystem<drake::systems::AffineSystem>(A, B, f0, C, D, y0);
                builder.Connect(planner->get_stanceLeg_output_port(),
                                affine->get_input_port());
                builder.Connect(affine->get_output_port(),
                                mit_esti->get_stance_phase_input_port());

                builder.Connect(estimator->get_q_output_port(), planner->get_q_input_port());
                builder.Connect(estimator->get_v_output_port(), planner->get_v_input_port());
                builder.Connect(estimator->get_stanceLeg_output_port(),
                                planner->get_stanceLeg_input_port());
                builder.Connect(estimator->get_q_output_port(),
                                controller->get_q_input_port());
                builder.Connect(estimator->get_v_output_port(),
                                controller->get_v_input_port());

                // drake::systems::lcm::LcmScopeSystem::AddToBuilder(
                //     &builder, &lcm_log, estimator->get_q_output_port(), "estimator.q",
                //     0.0);
                // drake::systems::lcm::LcmScopeSystem::AddToBuilder(
                //     &builder, &lcm_log, estimator->get_v_output_port(), "estimator.v",
                //     0.0);

                auto state_logger = LogVectorOutput(plant.get_state_output_port(), &builder);
                auto state_est_logger =
                    LogVectorOutput(mit_esti->get_state_output_port(), &builder);
                auto xhat_logger =
                    LogVectorOutput(mit_esti->get_xhat_output_port(), &builder);
                auto acc_logger = LogVectorOutput(
                    imu->get_output_port_measurement_acceleration(), &builder);
                auto torque_logger =
                    LogVectorOutput(controller->get_torque_output_port(), &builder);

                auto diagram = builder.Build();
                drake::systems::Simulator<double> simulator(*diagram);
                drake::VectorX<double> q0 = drake::VectorX<double>::Zero(17);
                q0 << 0.9962, -8.284e-07, 0.08716, -5.303e-08, 0, 0, 0.471, -0.005795,
                    -0.03286, -1.492, 2.075, -0.7569, 0.005606, 0.03178, -1.492, 2.074,
                    -0.7569;
                auto &plant_context = diagram->GetMutableSubsystemContext(
                    plant, &simulator.get_mutable_context());
                plant.SetPositions(&plant_context, q0);
                auto &planner_context = diagram->GetMutableSubsystemContext(
                    *planner, &simulator.get_mutable_context());
                auto gait_epoch = planner_context.get_mutable_discrete_state(0)
                                      .get_mutable_value(); // 用眼睛看的
                auto p0_WSw =
                    planner_context.get_mutable_discrete_state(1).get_mutable_value();
                auto stanceLeg =
                    planner_context.get_mutable_discrete_state(6).get_mutable_value();
                gait_epoch(0) = -0.35;
                p0_WSw << 0, 0.1, 0;
                stanceLeg(0) = -1;
                auto &esti_context = diagram->GetMutableSubsystemContext(
                    *mit_esti, &simulator.get_mutable_context());
                auto xhat = esti_context.get_mutable_discrete_state(0).get_mutable_value();
                xhat << 0, 0, 0.471, 0, 0, 0, 0, 0.1, 0, 0, -0.1, 0;

                meshcat_vis.StartRecording();
                simulator.set_target_realtime_rate(1.0);
                simulator.Initialize();
                gait_epoch(0) = -0.3;
                // simulator.AdvanceTo(300);
                try
                {
                    simulator.AdvanceTo(300);
                }
                catch (const std::runtime_error &e)
                {
                    std::cerr << e.what() << std::endl;
                }
                meshcat_vis.StopRecording();
                meshcat_vis.PublishRecording();

                const auto &state_log = state_logger->FindLog(simulator.get_context());
                const auto &state_est_log =
                    state_est_logger->FindLog(simulator.get_context());
                const auto &xhat_log = xhat_logger->FindLog(simulator.get_context());
                const auto &acc_log = acc_logger->FindLog(simulator.get_context());
                const auto &torque_log = torque_logger->FindLog(simulator.get_context());

                CallPython("figure", 1);
                CallPython("clf");
                CallPython("subplots_adjust", 0.05, 0.05, 0.95, 0.95,
                           ToPythonKwargs("hspace", 0.3));

                CallPython("subplot", 2, 2, 1);
                CallPython("plot", state_est_log.sample_times(),
                           state_est_log.data().middleRows<3>(4).transpose(),
                           ToPythonKwargs("label", ToPythonTuple("est0", "est1", "est2")));
                CallPython("plot", state_log.sample_times(),
                           state_log.data().middleRows<3>(4).transpose(),
                           ToPythonKwargs("label", ToPythonTuple("real0", "real1", "real2")));
                CallPython("title", "floating base pos");
                CallPython("legend");

                CallPython("subplot", 222);
                CallPython("plot", xhat_log.sample_times(),
                           xhat_log.data().middleRows<3>(3).transpose(),
                           ToPythonKwargs("label", ToPythonTuple("est0", "est1", "est2")));
                CallPython("plot", state_log.sample_times(),
                           state_log.data().middleRows<3>(20).transpose(),
                           ToPythonKwargs("label", ToPythonTuple("real0", "real1", "real2")));
                CallPython("title", "floating base vel");
                CallPython("legend");

                CallPython("subplot", 223);
                //   CallPython("plot", acc_log.sample_times(), acc_log.data().transpose(),
                //              ToPythonKwargs("label", ToPythonTuple("0", "1", "2")));
                //   CallPython("title", "imu acc");
                for (int i = 0; i < 10; i++)
                {
                    CallPython("plot", state_log.sample_times(),
                               state_log.data().row(i + 23).transpose(),
                               ToPythonKwargs("label", fmt::format("v_joint({})", i)));
                }
                CallPython("legend");
                CallPython("title", "v joint");

                CallPython("subplot", 224);
                for (int i = 0; i < 10; i++)
                {
                    CallPython("plot", state_log.sample_times(),
                               state_log.data().row(i + 7).transpose(),
                               ToPythonKwargs("label", fmt::format("q_joint({})", i)));
                }
                CallPython("legend");
                CallPython("title", "q joint");

                CallPython("wait");
                return 0;
            }

        } // namespace
    }     // namespace biped
} // namespace my_drake_examples

int main(int argc, char **argv)
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    return my_drake_examples::biped::do_main();
}
