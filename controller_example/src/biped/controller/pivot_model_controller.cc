#include "pivot_model_controller.h"

#include <iostream>
#include <iomanip>

#include <drake/common/eigen_types.h>
#include <drake/math/wrap_to.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/weld_joint.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    PivotModelController<T>::PivotModelController(double time_step, double offset, const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), float_plant_(plant)
    {
      DRAKE_THROW_UNLESS(time_step >= 0);
      h_ref_input_port_ = &this->DeclareVectorInputPort("h_ref", 10);
      hd_ref_input_port_ = &this->DeclareVectorInputPort("hd_ref", 10);
      hdd_ref_input_port_ = &this->DeclareVectorInputPort("hdd_ref", 10);
      stanceLeg_input_port_ = &this->DeclareVectorInputPort("stanceLeg", 1);
      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", 17);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", 16);

      const auto &torque = this->DeclareDiscreteState(10);
      torque_id_ = torque;

      torque_output_port_ = &this->DeclareStateOutputPort("torque", torque);

      this->DeclarePeriodicDiscreteUpdateEvent(time_step, offset, &PivotModelController::Update);

      float_context_ = plant.CreateDefaultContext();
      // 构建左支撑multibodyplant
      left_stance_plant_ = drake::multibody::MultibodyPlant<T>(plant.time_step());
      right_stance_plant_ = drake::multibody::MultibodyPlant<T>(plant.time_step());
      for (int i = 1; i < plant.num_bodies(); i++)
      {
        const auto &body = plant.get_body(drake::multibody::BodyIndex{i});
        auto spatial_inertia = body.CalcSpatialInertiaInBodyFrame(*float_context_);
        left_stance_plant_.AddRigidBody(body.name(), spatial_inertia);
        right_stance_plant_.AddRigidBody(body.name(), spatial_inertia);
      }
      for (int i = 0; i < 6; i++)
      {
        const auto &joint = plant.get_joint(drake::multibody::JointIndex{i});
        const auto &parent_body = joint.parent_body();
        const auto &frame_X_PF = joint.frame_on_parent();
        const auto &child_body = joint.child_body();
        const auto &frame_X_BM = joint.frame_on_child();
        if (i < 6)
        {
          const auto &revolute_axis =
              dynamic_cast<const drake::multibody::RevoluteJoint<T> &>(joint).revolute_axis();
          left_stance_plant_.template AddJoint<drake::multibody::RevoluteJoint>(
              joint.name(),
              left_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              left_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              -revolute_axis);
          right_stance_plant_.template AddJoint<drake::multibody::RevoluteJoint>(
              joint.name(),
              right_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              right_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              revolute_axis);
        }
        else
        {
          const auto &X_FM =
              dynamic_cast<const drake::multibody::WeldJoint<T> &>(joint).X_FM();
          left_stance_plant_.template AddJoint<drake::multibody::WeldJoint>(
              joint.name(),
              left_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              left_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              X_FM.inverse());
          right_stance_plant_.template AddJoint<drake::multibody::WeldJoint>(
              joint.name(),
              right_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              right_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              X_FM);
        }
      }
      for (int i = 6; i < 12; i++)
      {
        const auto &joint = plant.get_joint(drake::multibody::JointIndex{i});
        const auto &parent_body = joint.parent_body();
        const auto &frame_X_PF = joint.frame_on_parent();
        const auto &child_body = joint.child_body();
        const auto &frame_X_BM = joint.frame_on_child();
        if (i < 6)
        {
          const auto &revolute_axis =
              dynamic_cast<const drake::multibody::RevoluteJoint<T> &>(joint).revolute_axis();
          left_stance_plant_.template AddJoint<drake::multibody::RevoluteJoint>(
              joint.name(),
              left_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              left_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              revolute_axis);
          right_stance_plant_.template AddJoint<drake::multibody::RevoluteJoint>(
              joint.name(),
              right_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              right_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              -revolute_axis);
        }
        else
        {
          const auto &X_FM =
              dynamic_cast<const drake::multibody::WeldJoint<T> &>(joint).X_FM();
          left_stance_plant_.template AddJoint<drake::multibody::WeldJoint>(
              joint.name(),
              left_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              left_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              X_FM);
          right_stance_plant_.template AddJoint<drake::multibody::WeldJoint>(
              joint.name(),
              right_stance_plant_.GetBodyByName(child_body.name()),
              frame_X_BM.CalcPoseInBodyFrame(*float_context_),
              right_stance_plant_.GetBodyByName(parent_body.name()),
              frame_X_PF.CalcPoseInBodyFrame(*float_context_),
              X_FM.inverse());
        }
      }
      left_stance_plant_.template AddJoint<drake::multibody::RevoluteJoint>(
          "constraint",
          left_stance_plant_.world_body(),
          drake::math::RigidTransformd(),
          left_stance_plant_.GetBodyByName("l_foot_x"),
          drake::math::RigidTransformd(),
          -drake::Vector3<T>::UnitX());
      right_stance_plant_.template AddJoint<drake::multibody::RevoluteJoint>(
          "constraint",
          right_stance_plant_.world_body(),
          drake::math::RigidTransformd(),
          right_stance_plant_.GetBodyByName("r_foot_x"),
          drake::math::RigidTransformd(),
          -drake::Vector3<T>::UnitX());
      left_stance_plant_.Finalize();
      right_stance_plant_.Finalize();
      left_stance_context_ = left_stance_plant_.CreateDefaultContext();
      right_stance_context_ = right_stance_plant_.CreateDefaultContext();
    }

    template <typename T>
    void PivotModelController<T>::Update(const drake::systems::Context<T> &context,
                                         drake::systems::DiscreteValues<T> *next_state) const
    {
      const auto start = std::chrono::steady_clock::now();
      const auto &h_ref = get_h_ref_input_port().Eval(context);
      const auto &hd_ref = get_hd_ref_input_port().Eval(context);
      const auto &hdd_ref = get_hdd_ref_input_port().Eval(context);
      const auto &stanceLeg = get_stanceLeg_input_port().Eval(context)(0);
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);

      auto torque = next_state->get_mutable_value(torque_id_);

      auto &plant_ = stanceLeg == 1 ? left_stance_plant_ : right_stance_plant_;
      auto plant_context_ = stanceLeg == 1 ? left_stance_context_ : right_stance_context_;
      const int nq = plant_.num_positions();
      const int nv = plant_.num_velocities();
      drake::VectorX<T> q_pivot(nq);
      drake::VectorX<T> v_pivot(nv);
      if (stanceLeg == 1)
      {
        // q.segment(1,5).colwise.reverseInPlace();
        // v.segment(6,5).colwise.reverseInPlace();
        q_pivot.segment(1, 5) = q.segment(7, 5).reverse();
        q_pivot.tail(5) = q.tail(5);
        v_pivot.segment(1, 5) = v.segment(6, 5).reverse();
        v_pivot.tail(5) = v.tail(5);
      }
      else
      {
        // q_pivot.segment()
        // TOFO
      }
      plant_.SetPositions(plant_context_.get(), q_pivot);

      plant_.SetVelocities(plant_context_.get(), v_pivot);

      const auto &torso_frame = plant_.GetBodyByName("torso").body_frame();
      const auto &X_WTorso = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                        plant_.GetBodyByName("torso"));
      const auto &euler_WTorso = X_WTorso.rotation().ToRollPitchYaw();

      const auto &p_WScm = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);

      const auto &sw_footbody = plant_.GetBodyByName(stanceLeg == 1 ? "r_foot_x" : "l_foot_x");
      const auto &X_WSw = plant_.EvalBodyPoseInWorld(*plant_context_, sw_footbody);

      const auto &p_WSw = X_WSw.translation();
      const auto &euler_WSw = X_WSw.rotation().ToRollPitchYaw();
      drake::VectorX<double> h_est(10);
      h_est << euler_WTorso.roll_angle(), euler_WTorso.pitch_angle(), euler_WTorso.yaw_angle(),
          p_WScm(0), p_WScm(2),
          euler_WSw.pitch_angle(), euler_WSw.yaw_angle(),
          p_WSw;
      drake::MatrixX<T> Jv_hd = drake::MatrixX<T>::Zero(10, nv);
      drake::MatrixX<T> Jv_w_WTorso_Torso = drake::MatrixX<T>::Zero(3, nv);
      plant_.CalcJacobianAngularVelocity(*plant_context_,
                                         drake::multibody::JacobianWrtVariable::kV,
                                         torso_frame,
                                         plant_.world_frame(),
                                         torso_frame,
                                         &Jv_w_WTorso_Torso);
      const auto end_jw = std::chrono::steady_clock::now();
      Jv_hd.topRows(3) = Jv_w_WTorso_Torso;

      drake::MatrixX<T> Jv_v_WCcm = drake::MatrixX<T>::Zero(3, 11);
      plant_.CalcJacobianCenterOfMassTranslationalVelocity(
          *plant_context_, drake::multibody::JacobianWrtVariable::kV,
          plant_.world_frame(), plant_.world_frame(), &Jv_v_WCcm);
      Jv_hd.row(3) = Jv_v_WCcm.row(0);
      Jv_hd.row(4) = Jv_v_WCcm.row(2);

      const auto &swing_frame =
          plant_.GetBodyByName("r_foot_x").body_frame();
      drake::MatrixX<T> Jv_w_WSw_Sw = drake::MatrixX<T>::Zero(3, 11);
      plant_.CalcJacobianAngularVelocity(
          *plant_context_, drake::multibody::JacobianWrtVariable::kV,
          swing_frame, plant_.world_frame(), swing_frame, &Jv_w_WSw_Sw);
      Jv_hd.middleRows(5, 2) = Jv_w_WSw_Sw.bottomRows(2);
      drake::MatrixX<T> Jv_v_WSw = drake::MatrixX<T>::Zero(3, 11);
      plant_.CalcJacobianTranslationalVelocity(
          *plant_context_, drake::multibody::JacobianWrtVariable::kV,
          swing_frame, drake::Vector3<T>::Zero(), plant_.world_frame(), plant_.world_frame(), &Jv_v_WSw);
      Jv_hd.bottomRows(3) = Jv_v_WSw;
      drake::VectorX<double> hd_est = Jv_hd * v;

      drake::VectorX<T> y = h_est - h_ref;
      y(2) = drake::math::wrap_to(y(2), -M_PI, M_PI);
      y(6) = drake::math::wrap_to(y(6), -M_PI, M_PI);
      drake::VectorX<T> ydot = hd_est - hd_ref;
      ydot(3) = 0;

      drake::VectorX<T> vdot = drake::VectorX<T>::Unit(nv, 0);
      drake::MatrixX<T> Mu = drake::MatrixX<T>::Zero(1, nv);
      drake::multibody::MultibodyForces<T> external_forces(plant_);
      plant_.SetVelocities(plant_context_.get(), drake::VectorX<T>::Zero(nv));
      Mu.row(0) = plant_.CalcInverseDynamics(*plant_context_, vdot, external_forces);
      plant_.SetVelocities(plant_context_.get(), v_pivot);
      drake::VectorX<T> G = -plant_.CalcGravityGeneralizedForces(*plant_context_);
      drake::VectorX<T> Cv = drake::VectorX<T>::Zero(nv);
      plant_.CalcBiasTerm(*plant_context_, &Cv);
      drake::MatrixX<T> Me = drake::MatrixX<T>::Zero(nv, nv);
      drake::VectorX<T> vdot_des = drake::VectorX<T>::Zero(nv);
      drake::VectorX<T> He = drake::VectorX<T>::Zero(nv);

      Me << Mu, -Jv_hd;
      drake::VectorX<T> kd = drake::VectorX<T>::Zero(10);
      kd << 10, 10, 10, 1, 10, 20, 50, 50, 50, 50;
      drake::VectorX<T> kp = drake::VectorX<T>::Zero(10);
      kp << 25, 25, 25, 0, 25, 100, 625, 625, 625, 625;
      He << (G + Cv)(0), (hdd_ref - kp.cwiseProduct(y) - kd.cwiseProduct(ydot));
      vdot_des = Me.inverse() * (-He);
      drake::VectorX<double> torque_all =
          plant_.CalcInverseDynamics(*plant_context_, vdot_des, external_forces);
      torque = (torque_all + G).tail(10);
      if (stanceLeg == 1)
      {
        torque.head(5).reverseInPlace();
      }
      else
      {
        torque.head(5).swap(torque.tail(5));
        torque.head(5).reverseInPlace();
      }
    }

  } // namespace biped
} // namespace my_drake_examples
