#include <iostream>
#include <chrono>

#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/weld_joint.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/visualization/visualization_config_functions.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/common/eigen_types.h>

namespace my_drake_examples
{
  namespace biped
  {
    namespace
    {

      int do_main()
      {
        drake::multibody::MultibodyPlant<double> plant(0.001);
        drake::multibody::Parser parser(&plant);
        const std::string relative_name = "models/biped_v2/urdf/biped_v2_no_arm.urdf";
        parser.AddModelFromFile(relative_name);
        plant.Finalize();
        auto context = plant.CreateDefaultContext();
        int num_bodies = plant.num_bodies();
        int num_joints = plant.num_joints();

        // for (int i=0; i<num_bodies; i++) {
        //   std::cout << "body("<<i<<"): ";
        //   const auto& body = plant.get_body(drake::multibody::BodyIndex{i});
        //   std::cout << " name: " << body.name() << "\n";
        //   // std::cout << "; spatial_inertia:\n" << body.CalcSpatialInertiaInBodyFrame(*context2);
        // }
        // for (int i=0; i<num_joints; i++) {
        //   const auto& joint = plant.get_joint(drake::multibody::JointIndex{i});
        //   std::cout << "joint("<<i<<"):"<<joint.type_name()<<"; ";
        //   std::cout << "name: " << joint.name() << "\n";
        // }
        // std::cout << "nq: " << plant.num_positions() << "; ";
        // std::cout << "nv: " << plant.num_velocities() << "\n";
        drake::VectorX<double> q0 = drake::VectorX<double>::Zero(17);
        q0 << 0.9962, -8.284e-07, 0.08716, -5.303e-08,
            0, 0, 0.471,
            -0.005795, -0.03286, -1.492, 2.075, -0.7569,
            0.005606, 0.03178, -1.492, 2.074, -0.7569;
        plant.SetPositions(context.get(), q0);
        for (int i = 0; i < num_bodies; i++)
        {
          // std::cout << "body("<<i<<"): ";
          const auto &body = plant.get_body(drake::multibody::BodyIndex{i});
          // std::cout << " name: " << body.name() << "; ";
          const auto &X_WB = plant.EvalBodyPoseInWorld(*context, body);
          std::cout << X_WB << std::endl;
        }

        // plant2 是调转了左腿关节父子方向的版本
        // std::cout << "plant2:\n";
        drake::multibody::MultibodyPlant<double> plant2(0.001);
        for (int i = 1; i < num_bodies; i++)
        {
          const auto &body = plant.get_body(drake::multibody::BodyIndex{i});
          auto spatial_inertia = body.CalcSpatialInertiaInBodyFrame(*context);
          plant2.AddRigidBody(body.name(), spatial_inertia);
        }
        for (int i = 0; i < 6; i++)
        {
          if (i == 5 || i == 11)
            continue;
          const auto &joint = plant.get_joint(drake::multibody::JointIndex{i});
          const auto &parent_body = joint.parent_body();
          const auto &frame_X_PF = joint.frame_on_parent();
          const auto &child_body = joint.child_body();
          const auto &frame_X_BM = joint.frame_on_child();
          const auto &revolute_axis = dynamic_cast<const drake::multibody::RevoluteJoint<double> &>(joint).revolute_axis();
          plant2.AddJoint<drake::multibody::RevoluteJoint>(joint.name(),
                                                           plant2.GetBodyByName(child_body.name()),
                                                           frame_X_BM.CalcPoseInBodyFrame(*context),
                                                           plant2.GetBodyByName(parent_body.name()),
                                                           frame_X_PF.CalcPoseInBodyFrame(*context),
                                                           -revolute_axis);
        }
        {
          const auto &joint = plant.get_joint(drake::multibody::JointIndex{5});
          const auto &parent_body = joint.parent_body();
          const auto &frame_X_PF = joint.frame_on_parent();
          const auto &child_body = joint.child_body();
          const auto &frame_X_BM = joint.frame_on_child();
          plant2.AddJoint<drake::multibody::WeldJoint>(joint.name(),
                                                       plant2.GetBodyByName(child_body.name()),
                                                       frame_X_BM.CalcPoseInBodyFrame(*context),
                                                       plant2.GetBodyByName(parent_body.name()),
                                                       frame_X_PF.CalcPoseInBodyFrame(*context),
                                                       drake::math::RigidTransformd());
        }
        for (int i = 6; i < 12; i++)
        {
          if (i == 5 || i == 11)
            continue;
          const auto &joint = plant.get_joint(drake::multibody::JointIndex{i});
          const auto &parent_body = joint.parent_body();
          const auto &frame_X_PF = joint.frame_on_parent();
          const auto &child_body = joint.child_body();
          const auto &frame_X_BM = joint.frame_on_child();
          const auto &revolute_axis = dynamic_cast<const drake::multibody::RevoluteJoint<double> &>(joint).revolute_axis();
          plant2.AddJoint<drake::multibody::RevoluteJoint>(joint.name(),
                                                           plant2.GetBodyByName(parent_body.name()),
                                                           frame_X_PF.CalcPoseInBodyFrame(*context),
                                                           plant2.GetBodyByName(child_body.name()),
                                                           frame_X_BM.CalcPoseInBodyFrame(*context),
                                                           revolute_axis);
        }
        {
          const auto &joint = plant.get_joint(drake::multibody::JointIndex{11});
          const auto &parent_body = joint.parent_body();
          const auto &frame_X_PF = joint.frame_on_parent();
          const auto &child_body = joint.child_body();
          const auto &frame_X_BM = joint.frame_on_child();
          plant2.AddJoint<drake::multibody::WeldJoint>(joint.name(),
                                                       plant2.GetBodyByName(parent_body.name()),
                                                       frame_X_PF.CalcPoseInBodyFrame(*context),
                                                       plant2.GetBodyByName(child_body.name()),
                                                       frame_X_BM.CalcPoseInBodyFrame(*context),
                                                       drake::math::RigidTransformd());
        }
        plant2.AddJoint<drake::multibody::RevoluteJoint>("constraint",
                                                         plant2.world_body(),
                                                         drake::math::RigidTransformd(),
                                                         plant2.GetBodyByName("l_foot_x"),
                                                         drake::math::RigidTransformd(),
                                                         -drake::Vector3<double>::UnitX());
        plant2.Finalize();
        auto context2 = plant2.CreateDefaultContext();
        // for (int i=0; i<num_bodies; i++) {
        //   std::cout << "body("<<i<<"): ";
        //   const auto& body = plant2.get_body(drake::multibody::BodyIndex{i});
        //   std::cout << " name: " << body.name() << "\n";
        //   // std::cout << "; spatial_inertia:\n" << body.CalcSpatialInertiaInBodyFrame(*context2);
        // }
        // for (int i=0; i<num_joints; i++) {
        //   const auto& joint = plant2.get_joint(drake::multibody::JointIndex{i});
        //   std::cout << "joint("<<i<<"):"<<joint.type_name()<<"; ";
        //   std::cout << "name: " << joint.name() << "; ";
        //   std::cout << "position_start: " << joint.position_start() << "; ";
        //   std::cout << "velocity_start: " << joint.velocity_start() << "\n";
        // }
        // std::cout << "nq: " << plant2.num_positions() << "; ";
        // std::cout << "nv: " << plant2.num_velocities() << "\n";
        drake::VectorX<double> q00 = drake::VectorX<double>::Zero(11);
        q00 << 0.03286,
            -0.005795, -0.03286, -1.492, 2.075, -0.7569,
            0.005606, 0.03178, -1.492, 2.074, -0.7569;
        q00.segment(1, 5).colwise().reverseInPlace();
        plant2.SetPositions(context2.get(), q00);
        for (int i = 0; i < num_bodies; i++)
        {
          // std::cout << "body("<<i<<"): ";
          const auto &body = plant2.get_body(drake::multibody::BodyIndex{i});
          // std::cout << " name: " << body.name() << "; ";
          const auto &X_WB = plant2.EvalBodyPoseInWorld(*context2, body);
          std::cout << X_WB << std::endl;
        }
        const auto start = std::chrono::steady_clock::now();
        drake::VectorX<double> vdot = drake::VectorX<double>::Zero(11);
        vdot(0) = 1;
        drake::multibody::MultibodyForces<double> external_forces(plant2);
        drake::MatrixX<double> Mu = drake::MatrixX<double>::Zero(1, 11);
        const auto start_id = std::chrono::steady_clock::now();
        Mu.row(0) = plant2.CalcInverseDynamics(*context2, vdot, external_forces);
        const auto end_id = std::chrono::steady_clock::now();
        // std::cout << "Mu: " << Mu << "\n";
        const auto start_g = std::chrono::steady_clock::now();
        drake::VectorX<double> G = -plant2.CalcGravityGeneralizedForces(*context2);
        const auto end_g = std::chrono::steady_clock::now();
        const auto start_cv = std::chrono::steady_clock::now();
        drake::VectorX<double> Cv = drake::VectorX<double>::Zero(11);
        const auto end_cv = std::chrono::steady_clock::now();
        plant2.CalcBiasTerm(*context2, &Cv);
        // std::cout << "G: " << G.transpose() << "\n";
        // std::cout << "Cv: " << Cv.transpose() << "\n";
        drake::MatrixX<double> A = drake::MatrixX<double>::Zero(11, 11);
        drake::VectorX<double> vdot_des = drake::VectorX<double>::Zero(11);
        drake::VectorX<double> b = drake::VectorX<double>::Zero(11);
        drake::MatrixX<double> Jv_hd = drake::MatrixX<double>::Zero(10, 11);
        drake::MatrixX<double> Jv_w_WTorso_Torso = drake::MatrixX<double>::Zero(3, 11);
        const auto &torso_frame = plant2.GetBodyByName("torso").body_frame();
        const auto start_jw = std::chrono::steady_clock::now();
        plant2.CalcJacobianAngularVelocity(*context2,
                                           drake::multibody::JacobianWrtVariable::kV,
                                           torso_frame,
                                           plant2.world_frame(),
                                           torso_frame,
                                           &Jv_w_WTorso_Torso);
        const auto end_jw = std::chrono::steady_clock::now();
        Jv_hd.topRows(3) = Jv_w_WTorso_Torso;

        drake::MatrixX<double> Jv_v_WCcm = drake::MatrixX<double>::Zero(3, 11);
        plant2.CalcJacobianCenterOfMassTranslationalVelocity(
            *context2, drake::multibody::JacobianWrtVariable::kV,
            plant2.world_frame(), plant2.world_frame(), &Jv_v_WCcm);
        Jv_hd.row(3) = Jv_v_WCcm.row(0);
        Jv_hd.row(4) = Jv_v_WCcm.row(2);

        const auto &swing_frame =
            plant2.GetBodyByName("r_foot_x").body_frame();
        drake::MatrixX<double> Jv_w_WSw_Sw = drake::MatrixX<double>::Zero(3, 11);
        plant2.CalcJacobianAngularVelocity(
            *context2, drake::multibody::JacobianWrtVariable::kV,
            swing_frame, plant2.world_frame(), swing_frame, &Jv_w_WSw_Sw);
        Jv_hd.middleRows(5, 2) = Jv_w_WSw_Sw.bottomRows(2);
        drake::MatrixX<double> Jv_v_WSw = drake::MatrixX<double>::Zero(3, 11);
        plant2.CalcJacobianTranslationalVelocity(
            *context2, drake::multibody::JacobianWrtVariable::kV,
            swing_frame, drake::Vector3<double>::Zero(), plant2.world_frame(), plant2.world_frame(), &Jv_v_WSw);
        Jv_hd.bottomRows(3) = Jv_v_WSw;
        A.row(0) = Mu;
        A.bottomRows(10) = Jv_hd;
        b(0) = -(G + Cv)(0);
        b.tail(10).fill(0);
        // std::cout << "A:\n" << A << std::endl;
        const auto start_ls = std::chrono::steady_clock::now();
        vdot_des = A.inverse() * b;
        const auto end_ls = std::chrono::steady_clock::now();
        // std::cout << "vdot des: " << vdot_des.transpose() << "\n";
        drake::VectorX<double> torque =
            plant2.CalcInverseDynamics(*context2, vdot_des, external_forces);
        drake::VectorX<double> u = (torque + G).tail(10);
        u.head(5).reverseInPlace();
        // std::cout << "torque: " << u.transpose() << "\n";
        const auto end = std::chrono::steady_clock::now();
        const std::chrono::duration elapse = end - start;
        std::cout << "elapse: " << elapse.count() << "\n";
        const std::chrono::duration elapse_id = end_id - start_id;
        std::cout << "id elapse: " << elapse_id.count() << "\n";
        const std::chrono::duration elapse_g = end_g - start_g;
        std::cout << "elapse_g: " << elapse_g.count() << "\n";
        const std::chrono::duration elapse_cv = end_cv - start_cv;
        std::cout << "elapse_cv: " << elapse_cv.count() << "\n";
        const std::chrono::duration elapse_jw = end_jw - start_jw;
        std::cout << "elapse_jw: " << elapse_jw.count() << "\n";
        const std::chrono::duration elapse_ls = end_ls - start_ls;
        std::cout << "elapse_ls: " << elapse_ls.count() << "\n";
        return 0;
      }

    } // namespace
  }   // namespace biped
} // namespace my_drake_examples

int main(int argc, char **argv)
{
  return my_drake_examples::biped::do_main();
}
