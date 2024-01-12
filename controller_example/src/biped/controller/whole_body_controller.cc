#include "whole_body_controller.h"

#include <drake/common/eigen_types.h>
#include <drake/math/wrap_to.h>

#include <Eigen/QR>
#include <iomanip>
#include <iostream>
#include <thread>

namespace my_drake_examples
{
  namespace biped
  {

    using drake::MatrixX;
    using drake::VectorX;
    using drake::math::wrap_to;
    using drake::multibody::JacobianWrtVariable;
    using drake::systems::Context;
    using drake::systems::DiscreteValues;

    template <typename T>
    WholeBodyController<T>::WholeBodyController(
        double time_step, double offset,
        const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), plant_(plant)
    {
      DRAKE_THROW_UNLESS(time_step >= 0);
      nv_ = plant_.num_velocities();
      nq_ = plant_.num_positions();
      na_ = plant_.num_actuated_dofs();
      plant_context_ = plant_.CreateDefaultContext();

      h_ref_input_port_ = &this->DeclareVectorInputPort("h_ref", nh_);
      hd_ref_input_port_ = &this->DeclareVectorInputPort("hd_ref", nh_);
      hdd_ref_input_port_ = &this->DeclareVectorInputPort("hdd_ref", nh_);
      stance_input_port_ = &this->DeclareVectorInputPort("stance", 2);
      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", nq_);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", nv_);

      torque_id_ = this->DeclareDiscreteState(na_);

      torque_output_port_ = &this->DeclareStateOutputPort("torque", torque_id_);

      this->DeclarePeriodicDiscreteUpdateEvent(time_step, offset,
                                               &WholeBodyController::Update);
    }

    template <typename T>
    void WholeBodyController<T>::Update(const Context<T> &context,
                                        DiscreteValues<T> *next_state) const
    {
      std::cout.setf(std::ios::fixed);
      std::cout.precision(3);
      Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ",", "\n", " ", ";", "[",
                               "];\n");
      Eigen::IOFormat VecFmt(Eigen::StreamPrecision, 0, ",", "", " ", ";", "[",
                             "];");

      const auto &h_ref = get_h_ref_input_port().Eval(context);
      const auto &hd_ref = get_hd_ref_input_port().Eval(context);
      const auto &hdd_ref = get_hdd_ref_input_port().Eval(context);
      const auto &left_stance = get_stance_input_port().Eval(context)(0);
      const auto &right_stance = get_stance_input_port().Eval(context)(1);
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);

      auto torque = next_state->get_mutable_value(torque_id_);

      plant_.SetPositions(plant_context_.get(), q);
      plant_.SetVelocities(plant_context_.get(), v);

      const auto &torso_body = plant_.GetBodyByName("torso");
      const auto &leftfoot_body = plant_.GetBodyByName("l_foot_x");
      const auto &rightfoot_body = plant_.GetBodyByName("r_foot_x");
      const auto &world_body = plant_.world_body();
      const auto &torso_frame = torso_body.body_frame();
      const auto &leftfoot_frame = leftfoot_body.body_frame();
      const auto &rightfoot_frame = rightfoot_body.body_frame();
      const auto &world_frame = plant_.world_frame();

      VectorX<T> y_ref;
      VectorX<T> yd_ref;
      VectorX<T> ydd_ref;
      VectorX<T> y_est;
      VectorX<T> yd_est;
      VectorX<T> y_err;
      VectorX<T> yd_err;
      VectorX<T> ydd_des;
      MatrixX<T> Jv_yd;
      MatrixX<T> Jv_cd;
      VectorX<T> acc_des(nv_);
      // 也许可以加上bias
      int ny;
      int nc;
      int nf = 6;
      if (left_stance == 1 && right_stance == 1)
      {
        ny = 14;
        nc = 10;
      }
      else if (left_stance == 0 && right_stance == 0)
      {
        ny = 18;
        nc = 0;
      }
      else
      { // single stance
        ny = 18;
        nc = 5;
      }
      y_ref = VectorX<T>::Zero(ny);
      yd_ref = VectorX<T>::Zero(ny);
      ydd_ref = VectorX<T>::Zero(ny);
      y_est = VectorX<T>::Zero(ny);
      yd_est = VectorX<T>::Zero(ny);
      y_err = VectorX<T>::Zero(ny);
      yd_err = VectorX<T>::Zero(ny);
      ydd_des = VectorX<T>::Zero(ny);
      Jv_yd = MatrixX<T>::Zero(ny, nv_);
      Jv_cd = MatrixX<T>::Zero(nc, nv_);

      if (left_stance == 1 && right_stance == 1)
      {
        y_ref << h_ref.head(6), h_ref.tail(8);
        yd_ref << hd_ref.head(6), hd_ref.tail(8);
        ydd_ref << hdd_ref.head(6), hdd_ref.tail(8);

        const auto &X_WTorso =
            plant_.EvalBodyPoseInWorld(*plant_context_, torso_body);
        const auto &euler_WTorso = X_WTorso.rotation().ToRollPitchYaw();
        const auto &p_WScm =
            plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
        y_est << euler_WTorso.vector(), p_WScm, q.tail(8);

        MatrixX<T> Jv_w_WTorso_Torso(3, nv_);
        plant_.CalcJacobianAngularVelocity(*plant_context_, JacobianWrtVariable::kV,
                                           torso_frame, world_frame, torso_frame,
                                           &Jv_w_WTorso_Torso);
        // 也不是很严谨 TODO 写的更好些.
        // 如果使用欧拉角旋转的中间坐标系，它就很好了。
        Jv_yd.topRows(3) = Jv_w_WTorso_Torso;

        MatrixX<T> Jv_v_WScm(3, nv_);
        plant_.CalcJacobianCenterOfMassTranslationalVelocity(
            *plant_context_, JacobianWrtVariable::kV, world_frame, world_frame,
            &Jv_v_WScm);
        Jv_yd.middleRows(3, 3) = Jv_v_WScm;

        Jv_yd.template block<8, 8>(6, 16) = MatrixX<T>::Identity(8, 8);

        yd_est = Jv_yd * v;

        y_err = y_est - y_ref;
        y_err(2) = wrap_to(y_err(2), -M_PI, M_PI);
        yd_err = yd_est - yd_ref;

        VectorX<T> kd(ny);
        kd.fill(10);
        // kd(2) = 4;
        VectorX<T> kp(ny);
        kp.fill(25);
        // kp(2) = 4;
        ydd_des = ydd_ref - kp.cwiseProduct(y_err) - kd.cwiseProduct(yd_err);

        MatrixX<T> Jv_V_WLfoot(6, nv_);
        plant_.CalcJacobianSpatialVelocity(
            *plant_context_, JacobianWrtVariable::kV, leftfoot_frame,
            drake::Vector3<T>::Zero(), world_frame, leftfoot_frame, &Jv_V_WLfoot);
        Jv_cd.topRows(5) = Jv_V_WLfoot.bottomRows(5);

        MatrixX<T> Jv_V_WRfoot(6, nv_);
        plant_.CalcJacobianSpatialVelocity(
            *plant_context_, JacobianWrtVariable::kV, rightfoot_frame,
            drake::Vector3<T>::Zero(), world_frame, rightfoot_frame, &Jv_V_WRfoot);
        Jv_cd.bottomRows(5) = Jv_V_WRfoot.bottomRows(5);
      }
      else if (left_stance == 0 && right_stance == 0)
      {
        y_ref = h_ref.tail(18);
        yd_ref = hd_ref.tail(18);
        ydd_ref = hdd_ref.tail(18);

        const auto &X_WLfoot =
            plant_.EvalBodyPoseInWorld(*plant_context_, leftfoot_body);
        const auto &X_WRfoot =
            plant_.EvalBodyPoseInWorld(*plant_context_, rightfoot_body);
        const auto &euler_WLfoot = X_WLfoot.rotation().ToRollPitchYaw();
        const auto &euler_WRfoot = X_WRfoot.rotation().ToRollPitchYaw();
        y_est << euler_WLfoot.pitch_angle(), euler_WLfoot.yaw_angle(),
            X_WLfoot.translation(), euler_WRfoot.pitch_angle(),
            euler_WRfoot.yaw_angle(), X_WRfoot.translation(), q.tail(8);

        MatrixX<T> Jv_w_WLf_Lf(3, nv_);
        plant_.CalcJacobianAngularVelocity(*plant_context_, JacobianWrtVariable::kV,
                                           leftfoot_frame, world_frame,
                                           leftfoot_frame, &Jv_w_WLf_Lf);
        Jv_yd.topRows(2) = Jv_w_WLf_Lf.bottomRows(2);

        MatrixX<T> Jv_v_WLf(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(
            *plant_context_, JacobianWrtVariable::kV, leftfoot_frame,
            drake::Vector3<T>::Zero(), world_frame, world_frame, &Jv_v_WLf);
        Jv_yd.middleRows(2, 3) = Jv_v_WLf;

        MatrixX<T> Jv_w_WRf_Rf(3, nv_);
        plant_.CalcJacobianAngularVelocity(*plant_context_, JacobianWrtVariable::kV,
                                           rightfoot_frame, world_frame,
                                           rightfoot_frame, &Jv_w_WRf_Rf);
        Jv_yd.middleRows(5, 2) = Jv_w_WRf_Rf.bottomRows(2);

        MatrixX<T> Jv_v_WRf(3, nv_);
        plant_.CalcJacobianTranslationalVelocity(
            *plant_context_, JacobianWrtVariable::kV, rightfoot_frame,
            drake::Vector3<T>::Zero(), world_frame, world_frame, &Jv_v_WRf);
        Jv_yd.middleRows(7, 3) = Jv_v_WRf;

        Jv_yd.template block<8, 8>(10, 16) = MatrixX<T>::Identity(8, 8);

        yd_est = Jv_yd * v;

        y_err = y_est - y_ref;
        y_err(2) = wrap_to(y_err(2), -M_PI, M_PI);
        y_err(7) = wrap_to(y_err(7), -M_PI, M_PI);
        yd_err = yd_est - yd_ref;

        VectorX<T> kd(ny);
        kd << 50, 20, 50, 50, 50, 50, 20, 50, 50, 50, 10, 10, 10, 10, 10, 10, 10,
            10;
        VectorX<T> kp(ny);
        kp << 625, 100, 625, 625, 625, 625, 100, 625, 625, 625, 25, 25, 25, 25, 25,
            25, 25, 25;
        ydd_des = ydd_ref - kp.cwiseProduct(y_err) - kd.cwiseProduct(yd_err);
      }
      MatrixX<T> M(nv_, nv_);
      plant_.CalcMassMatrix(*plant_context_, &M);
      VectorX<T> G = -plant_.CalcGravityGeneralizedForces(*plant_context_);
      VectorX<T> Cv(nv_);
      plant_.CalcBiasTerm(*plant_context_, &Cv);

      if (left_stance == 1 && right_stance == 1)
      {
        MatrixX<T> Me(ny + nc, nv_);
        Me << -Jv_yd, -Jv_cd;
        VectorX<T> He(ny + nc);
        He << ydd_des, VectorX<T>::Zero(nc);
        acc_des = -Me.lu().solve(He);
        VectorX<T> GRF_des(nc);
        MatrixX<T> A = Jv_cd.transpose().topRows(nf);
        VectorX<T> b = M.topRows(nf) * acc_des + (G + Cv).topRows(nf);
        GRF_des = A.completeOrthogonalDecomposition().solve(b);
        GRF_des(0) = std::clamp(GRF_des(0), -0.06 * GRF_des(4), 0.06 * GRF_des(4));
        GRF_des(5) = std::clamp(GRF_des(5), -0.06 * GRF_des(9), 0.06 * GRF_des(9));
        // 重载变量是不好的
        Me << M.topRows(nf), -Jv_yd.bottomRows(8), -Jv_cd;
        // 手臂应该没有支撑力补偿？
        He << (G + Cv).head(nf) - Jv_cd.transpose().topRows(6) * GRF_des,
            ydd_des.tail(8), VectorX<T>::Zero(nc);
        acc_des = Me.lu().solve(-He);

        torque = M.bottomRows(na_) * acc_des + (G + Cv).bottomRows(na_) -
                 Jv_cd.transpose().bottomRows(na_) * GRF_des;
        // std::cout << "t= " << context.get_time() << "; ";
        // std::cout << "torque= " << torque.format(VecFmt) << "\n";
      }
      else if (left_stance == 0 && right_stance == 0)
      {
        // 现在还没处理关于手臂的部分，也许运行到这会报错
        MatrixX<T> Me(nf + ny, nv_);
        VectorX<T> He(nf + ny);
        Me << M.topRows(nf), -Jv_yd;
        He << (G + Cv).head(nf), ydd_des;
        acc_des = -Me.lu().solve(He);
        torque = M.bottomRows(na_) * acc_des + (G + Cv).bottomRows(na_);
        // std::cout << "Me= " << Me.format(CleanFmt) << "\n";
        // std::cout << "He= " << He.format(VecFmt) << "\n";
        // std::cout << "torque= " << torque.format(VecFmt) << "\n";
      }
      // torque.fill(0);
    }

    DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
        class ::my_drake_examples::biped::WholeBodyController)

  } // namespace biped
} // namespace my_drake_examples
