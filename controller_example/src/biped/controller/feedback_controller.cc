#include "feedback_controller.h"

#include <iomanip>
#include <iostream>
#include <thread>

// #include<Eigen/Core>
// #include<Eigen/SVD>
// #include <Eigen/QR>
#include <drake/common/eigen_types.h>
#include <drake/math/wrap_to.h>

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
    FeedbackController<T>::FeedbackController(
        double time_step, double offset,
        const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), plant_(plant)
    {
      DRAKE_THROW_UNLESS(time_step >= 0);
      const int nv = plant_.num_velocities();
      const int ng = 5;
      nv_ = plant_.num_velocities();
      nq_ = plant_.num_positions();
      na_ = plant_.num_actuated_dofs();

      h_ref_input_port_ = &this->DeclareVectorInputPort("h_ref", 10);
      hd_ref_input_port_ = &this->DeclareVectorInputPort("hd_ref", 10);
      hdd_ref_input_port_ = &this->DeclareVectorInputPort("hdd_ref", 10);
      stanceLeg_input_port_ = &this->DeclareVectorInputPort("stanceLeg", 1);
      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", 17);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", nv);
      phase_input_port_ = &this->DeclareVectorInputPort("phase_input", 1);

      torque_id_ = this->DeclareDiscreteState(10);
      G_id_ = this->DeclareDiscreteState(nv);
      Cv_id_ = this->DeclareDiscreteState(nv);
      H1_id_ = this->DeclareDiscreteState(nv + ng);
      H2_id_ = this->DeclareDiscreteState(10);
      lambda_id_ = this->DeclareDiscreteState(nv + ng);
      h_est_id_ = this->DeclareDiscreteState(10);
      hd_est_id_ = this->DeclareDiscreteState(10);
      calc_elapsed_id_ = this->DeclareDiscreteState(1);
      begin_calc_id_ = this->DeclareDiscreteState(1);
      end_calc_id_ = this->DeclareDiscreteState(1);

      torque_output_port_ = &this->DeclareStateOutputPort("torque", torque_id_);
      G_output_port_ = &this->DeclareStateOutputPort("G", G_id_);
      Cv_output_port_ = &this->DeclareStateOutputPort("Cv", Cv_id_);
      H1_output_port_ = &this->DeclareStateOutputPort("H1", H1_id_);
      H2_output_port_ = &this->DeclareStateOutputPort("H2", H2_id_);
      lambda_output_port_ = &this->DeclareStateOutputPort("lambda", lambda_id_);
      h_est_output_port_ = &this->DeclareStateOutputPort("h_est", h_est_id_);
      hd_est_output_port_ = &this->DeclareStateOutputPort("hd_est", hd_est_id_);
      calc_elapsed_output_port_ =
          &this->DeclareStateOutputPort("calc_elapsed", calc_elapsed_id_);
      begin_calc_output_port_ =
          &this->DeclareStateOutputPort("begin_calc", begin_calc_id_);
      end_calc_output_port_ =
          &this->DeclareStateOutputPort("end_calc", end_calc_id_);

      this->DeclarePeriodicDiscreteUpdateEvent(time_step, offset,
                                               &FeedbackController::Update);

      plant_context_ = plant_.CreateDefaultContext();
    }

    template <typename T>
    void FeedbackController<T>::Update(
        const drake::systems::Context<T> &context,
        drake::systems::DiscreteValues<T> *next_state) const
    {
      const auto start = std::chrono::steady_clock::now();
      // std::this_thread::sleep_for(std::chrono::microseconds(2000));
      const auto &h_ref = get_h_ref_input_port().Eval(context);
      const auto &hd_ref = get_hd_ref_input_port().Eval(context);
      const auto &hdd_ref = get_hdd_ref_input_port().Eval(context);
      const auto &stanceLeg = get_stanceLeg_input_port().Eval(context)(0);
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);
      int phase_input = get_phase_input_port().Eval(context)(0);
      int left_stance, right_stance;
      if (stanceLeg == 1)
      {
        left_stance = 1;
        right_stance = 0;
      }
      else if (stanceLeg == -1)
      {
        left_stance = 0;
        right_stance = 1;
      }
      else if (stanceLeg == 0)
      {
        left_stance = 1;
        right_stance = 1;
      }
      else if (stanceLeg == 2)
      {
        left_stance = 0;
        right_stance = 0;
      }

      if (stanceLeg == 1 || stanceLeg == -1)
      {

        auto torque = next_state->get_mutable_value(torque_id_);
        auto G = next_state->get_mutable_value(G_id_);
        auto Cv = next_state->get_mutable_value(Cv_id_);
        auto H1 = next_state->get_mutable_value(H1_id_);
        auto H2 = next_state->get_mutable_value(H2_id_);
        auto lambda = next_state->get_mutable_value(lambda_id_);
        auto h_est = next_state->get_mutable_value(h_est_id_);
        auto hd_est = next_state->get_mutable_value(hd_est_id_);
        auto &calc_elapsed = next_state->get_mutable_value(calc_elapsed_id_)(0);
        auto &begin_calc = next_state->get_mutable_value(begin_calc_id_)(0);
        auto &end_calc = next_state->get_mutable_value(end_calc_id_)(0);

        plant_.SetPositions(plant_context_.get(), q);
        plant_.SetVelocities(plant_context_.get(), v);

        const int nv = plant_.num_velocities();
        const int ng = 5;

        const auto &torso_frame = plant_.GetBodyByName("torso").body_frame();
        const auto &X_WTorso = plant_.EvalBodyPoseInWorld(
            *plant_context_, plant_.GetBodyByName("torso"));
        const auto &euler_WTorso = X_WTorso.rotation().ToRollPitchYaw();

        const auto &p_WScm = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
        const auto &st_footbody =
            plant_.GetBodyByName(stanceLeg > 0 ? "l_foot_x" : "r_foot_x");
        const auto &X_WSt = plant_.EvalBodyPoseInWorld(*plant_context_, st_footbody);
        const auto &p_WSt = X_WSt.translation();
        const auto &p_StScm_W = p_WScm - p_WSt;
        const auto yaw_WSt = X_WSt.rotation().ToRollPitchYaw().yaw_angle();
        const auto &p_StScm_Fst =
            drake::math::RotationMatrix<T>::MakeZRotation(-yaw_WSt) * p_StScm_W;

        const auto &sw_footbody =
            plant_.GetBodyByName(stanceLeg == 1 ? "r_foot_x" : "l_foot_x");
        const auto &X_WSw = plant_.EvalBodyPoseInWorld(*plant_context_, sw_footbody);

        const auto &p_WSw = X_WSw.translation();
        const auto &euler_WSw = X_WSw.rotation().ToRollPitchYaw();
        h_est << euler_WTorso.roll_angle(), euler_WTorso.pitch_angle(),
            euler_WTorso.yaw_angle(), p_StScm_Fst(0), p_StScm_Fst(2),
            euler_WSw.pitch_angle(), euler_WSw.yaw_angle(), p_WSw;
        drake::MatrixX<T> Jv_hd = drake::MatrixX<T>::Zero(10, nv);
        drake::MatrixX<T> Jv_w_WTorso_Torso = drake::MatrixX<T>::Zero(3, nv);
        plant_.CalcJacobianAngularVelocity(
            *plant_context_, drake::multibody::JacobianWrtVariable::kV, torso_frame,
            plant_.world_frame(), torso_frame, &Jv_w_WTorso_Torso);
        Jv_hd.topRows(3) = Jv_w_WTorso_Torso;

        drake::MatrixX<T> Jv_v_WCcm = drake::MatrixX<T>::Zero(3, nv);
        plant_.CalcJacobianCenterOfMassTranslationalVelocity(
            *plant_context_, drake::multibody::JacobianWrtVariable::kV,
            plant_.world_frame(), plant_.world_frame(), &Jv_v_WCcm);
        drake::MatrixX<T> Jv_v_WCcm_Fst = drake::MatrixX<T>::Zero(3, nv);
        const auto &stance_frame =
            plant_.GetBodyByName(stanceLeg == 1 ? "l_foot_x" : "r_foot_x")
                .body_frame();
        Jv_v_WCcm_Fst =
            drake::math::RotationMatrix<T>::MakeZRotation(-yaw_WSt) * Jv_v_WCcm;

        Jv_hd.row(3) = Jv_v_WCcm_Fst.row(0);
        Jv_hd.row(4) = Jv_v_WCcm_Fst.row(2);

        const auto &swing_frame =
            plant_.GetBodyByName(stanceLeg == 1 ? "r_foot_x" : "l_foot_x")
                .body_frame();
        drake::MatrixX<T> Jv_w_WSw_Sw = drake::MatrixX<T>::Zero(3, nv);
        plant_.CalcJacobianAngularVelocity(
            *plant_context_, drake::multibody::JacobianWrtVariable::kV, swing_frame,
            plant_.world_frame(), swing_frame, &Jv_w_WSw_Sw);
        Jv_hd.middleRows(5, 2) = Jv_w_WSw_Sw.bottomRows(2);
        drake::MatrixX<T> Jv_v_WSw = drake::MatrixX<T>::Zero(3, nv);
        plant_.CalcJacobianTranslationalVelocity(
            *plant_context_, drake::multibody::JacobianWrtVariable::kV, swing_frame,
            drake::Vector3<T>::Zero(), plant_.world_frame(), plant_.world_frame(),
            &Jv_v_WSw);
        Jv_hd.bottomRows(3) = Jv_v_WSw;

        hd_est = Jv_hd * v;

        // drake::MatrixX<T> Jv_v_BSt_W(3, nv);
        // plant_.CalcJacobianTranslationalVelocity(
        //     *plant_context_, drake::multibody::JacobianWrtVariable::kV,
        //     stance_frame, drake::Vector3<T>::Zero(), torso_frame,
        //     plant_.world_frame(), &Jv_v_BSt_W);
        // std::cout << "from ctrl:" << (-Jv_v_BSt_W * v).transpose() << ";"
        //           << "real:" << v.segment(3, 3).transpose() << ";;";
        // h_->torso.roll,torso.pitch,torso.yaw,dp_com_stfoot.x,dp_com_stfoot.z,swfoot.pitch,swfoot.yaw,p_swfoot

        // std::cout << "h_ref:"<<h_ref.transpose()<<std::endl;
        // std::cout << "h_est:"<<h_est.transpose()<<std::endl;
        drake::VectorX<T> y = h_est - h_ref;
        y(2) = drake::math::wrap_to(y(2), -M_PI, M_PI);
        y(6) = drake::math::wrap_to(y(6), -M_PI, M_PI);
        drake::VectorX<T> ydot = hd_est - hd_ref;
        // ydot(3) = 0;

        drake::MatrixX<T> M = drake::MatrixX<T>::Identity(nv, nv);
        plant_.CalcMassMatrix(*plant_context_, &M);
        // drake::VectorX<T> Cv = drake::VectorX<T>::Zero(nv);
        plant_.CalcBiasTerm(*plant_context_, &Cv);
        // drake::VectorX<T> G =
        // -plant_.CalcGravityGeneralizedForces(*plant_context_);
        G = -plant_.CalcGravityGeneralizedForces(*plant_context_);
        drake::MatrixX<T> Jv_V_WSto = drake::MatrixX<T>::Zero(6, nv);
        plant_.CalcJacobianSpatialVelocity(
            *plant_context_, drake::multibody::JacobianWrtVariable::kV, stance_frame,
            drake::Vector3<T>::Zero(), plant_.world_frame(), stance_frame,
            &Jv_V_WSto);
        drake::multibody::SpatialAcceleration<T> AvBias_WSto =
            plant_.CalcBiasSpatialAcceleration(
                *plant_context_, drake::multibody::JacobianWrtVariable::kV,
                stance_frame, drake::Vector3<T>::Zero(), plant_.world_frame(),
                stance_frame);
        // g表示地面约束，包括roll方向以外所有方向
        drake::MatrixX<T> Jv_g_WSto = Jv_V_WSto.bottomRows(5);
        drake::VectorX<T> gvBias_WSto = drake::VectorX<T>::Zero(5);
        gvBias_WSto << AvBias_WSto.rotational().tail(2), AvBias_WSto.translational();

        drake::MatrixX<T> M11 = drake::MatrixX<T>::Zero(nv + ng, nv + ng);
        M11.block(0, 0, 6, nv) = M.topRows(6);
        M11.block(0, nv, 6, 5) = -Jv_g_WSto.transpose().topRows(6);
        M11.block(6, 0, 5, nv) = Jv_g_WSto;
        M11.block(11, 0, 10, nv) = -Jv_hd;
        drake::MatrixX<T> M21 = drake::MatrixX<T>::Zero(10, nv + ng);
        M21 << M.bottomRows(10), -Jv_g_WSto.transpose().bottomRows(10);
        drake::VectorX<T> kd = drake::VectorX<T>::Zero(10);
        drake::VectorX<T> kp = drake::VectorX<T>::Zero(10);

        if (phase_input == 2) // 走楼梯的
        {
          kd << 10, 10, 10, 10, 10, 20, 50, 50, 50, 20;
          kp << 25, 25, 25, 25, 25, 100, 625, 625, 625, 100;
        }
        else
        {
          kd << 10, 18, 10, 2, 10, 20, 40, 35, 18, 12;
          kp << 25, 80, 25, 1, 25, 100, 400, 400, 200, 50;
        }
        H1 << (G + Cv).head(6), 0 * gvBias_WSto,
            1 * (hdd_ref - kp.cwiseProduct(y) - kd.cwiseProduct(ydot));
        const auto start_pinv = std::chrono::steady_clock::now();
        //   drake::MatrixX<T> pinv =
        //   M11.completeOrthogonalDecomposition().pseudoInverse();
        drake::MatrixX<T> pinv = M11.inverse();
        const auto end_pinv = std::chrono::steady_clock::now();
        lambda = -pinv * H1;
        H2 = (G + Cv).tail(10);
        torque = M21 * lambda + H2;
      }
      else
      {

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
          ny = 6;
          nc = 10;
        }
        else if (left_stance == 0 && right_stance == 0)
        {
          ny = 10;
          nc = 0;
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
          y_ref = h_ref.head(6);
          yd_ref = hd_ref.head(6);
          ydd_ref = hdd_ref.head(6);

          const auto &X_WTorso = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                            torso_body);
          const auto &euler_WTorso = X_WTorso.rotation().ToRollPitchYaw();
          const auto &p_WScm = plant_.CalcCenterOfMassPositionInWorld(*plant_context_);
          y_est << euler_WTorso.vector(), p_WScm;

          MatrixX<T> Jv_w_WTorso_Torso(3, nv_);
          plant_.CalcJacobianAngularVelocity(*plant_context_, JacobianWrtVariable::kV,
                                             torso_frame, world_frame, torso_frame, &Jv_w_WTorso_Torso);
          // 也不是很严谨 TODO 写的更好些. 如果使用欧拉角旋转的中间坐标系，它就很好了。
          Jv_yd.topRows(3) = Jv_w_WTorso_Torso;

          MatrixX<T> Jv_v_WScm(3, nv_);
          plant_.CalcJacobianCenterOfMassTranslationalVelocity(
              *plant_context_, JacobianWrtVariable::kV,
              world_frame, world_frame, &Jv_v_WScm);
          Jv_yd.bottomRows(3) = Jv_v_WScm;

          yd_est = Jv_yd * v;

          // std::cout << "y_est:"<<y_est.transpose()<<std::endl;
          // std::cout << "y_ref:"<<y_ref.transpose()<<std::endl;
          y_err = y_est - y_ref;
          y_err(2) = wrap_to(y_err(2), -M_PI, M_PI);
          yd_err = yd_est - yd_ref;

          VectorX<T> kd(ny);
          kd.fill(10);
          // kd<<10,5,10,  10,10,10;
          VectorX<T> kp(ny);
          kp.fill(25);
          // kp<<25,20,25,  25,25,25;
          ydd_des = ydd_ref - kp.cwiseProduct(y_err) - kd.cwiseProduct(yd_err);

          MatrixX<T> Jv_V_WLfoot(6, nv_);
          plant_.CalcJacobianSpatialVelocity(*plant_context_, JacobianWrtVariable::kV,
                                             leftfoot_frame, drake::Vector3<T>::Zero(),
                                             world_frame, leftfoot_frame, &Jv_V_WLfoot);
          Jv_cd.topRows(5) = Jv_V_WLfoot.bottomRows(5);

          MatrixX<T> Jv_V_WRfoot(6, nv_);
          plant_.CalcJacobianSpatialVelocity(*plant_context_, JacobianWrtVariable::kV,
                                             rightfoot_frame, drake::Vector3<T>::Zero(),
                                             world_frame, rightfoot_frame, &Jv_V_WRfoot);
          Jv_cd.bottomRows(5) = Jv_V_WRfoot.bottomRows(5);
        }
        else if (left_stance == 0 && right_stance == 0)
        {
          y_ref = h_ref.tail(10);
          yd_ref = hd_ref.tail(10);
          ydd_ref = hdd_ref.tail(10);

          const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                            leftfoot_body);
          const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                            rightfoot_body);
          const auto &euler_WLfoot = X_WLfoot.rotation().ToRollPitchYaw();
          const auto &euler_WRfoot = X_WRfoot.rotation().ToRollPitchYaw();
          y_est << euler_WLfoot.pitch_angle(), euler_WLfoot.yaw_angle(),
              X_WLfoot.translation(),
              euler_WRfoot.pitch_angle(), euler_WRfoot.yaw_angle(),
              X_WRfoot.translation();

          MatrixX<T> Jv_w_WLf_Lf(3, nv_);
          plant_.CalcJacobianAngularVelocity(*plant_context_, JacobianWrtVariable::kV,
                                             leftfoot_frame, world_frame, leftfoot_frame, &Jv_w_WLf_Lf);
          Jv_yd.topRows(2) = Jv_w_WLf_Lf.bottomRows(2);

          MatrixX<T> Jv_v_WLf(3, nv_);
          plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                   JacobianWrtVariable::kV,
                                                   leftfoot_frame,
                                                   drake::Vector3<T>::Zero(),
                                                   world_frame,
                                                   world_frame,
                                                   &Jv_v_WLf);
          Jv_yd.middleRows(2, 3) = Jv_v_WLf;

          MatrixX<T> Jv_w_WRf_Rf(3, nv_);
          plant_.CalcJacobianAngularVelocity(*plant_context_, JacobianWrtVariable::kV,
                                             rightfoot_frame, world_frame, rightfoot_frame, &Jv_w_WRf_Rf);
          Jv_yd.middleRows(5, 2) = Jv_w_WRf_Rf.bottomRows(2);

          MatrixX<T> Jv_v_WRf(3, nv_);
          plant_.CalcJacobianTranslationalVelocity(*plant_context_,
                                                   JacobianWrtVariable::kV,
                                                   rightfoot_frame,
                                                   drake::Vector3<T>::Zero(),
                                                   world_frame,
                                                   world_frame,
                                                   &Jv_v_WRf);
          Jv_yd.bottomRows(3) = Jv_v_WRf;

          yd_est = Jv_yd * v;
          // std::cout << "ay_est:"<<y_est.transpose()<<std::endl;
          // std::cout << "ay_ref:"<<y_ref.transpose()<<std::endl;
          y_err = y_est - y_ref;
          y_err(1) = wrap_to(y_err(1), -M_PI, M_PI);
          y_err(6) = wrap_to(y_err(6), -M_PI, M_PI);
          yd_err = yd_est - yd_ref;

          VectorX<T> kd(ny);
          kd << 50, 20, 50, 50, 50, 50, 20, 50, 50, 50;
          VectorX<T> kp(ny);
          kp << 625, 100, 625, 625, 625, 625, 100, 625, 625, 625;
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
          VectorX<T> b = M.topRows(nf) * acc_des + (G).topRows(nf);
          GRF_des = A.completeOrthogonalDecomposition().solve(b);
          GRF_des(0) = std::clamp(GRF_des(0), -0.06 * GRF_des(4), 0.06 * GRF_des(4));
          GRF_des(5) = std::clamp(GRF_des(5), -0.06 * GRF_des(9), 0.06 * GRF_des(9));
          // 重载变量是不好的
          Me << M.topRows(nf), -Jv_cd;
          He << (G).head(nf) - Jv_cd.transpose().topRows(6) * GRF_des, VectorX<T>::Zero(nc);
          acc_des = Me.lu().solve(-He);

          torque = M.bottomRows(na_) * acc_des + (G).bottomRows(na_) - Jv_cd.transpose().bottomRows(na_) * GRF_des;
          // std::cout << "t= " << context.get_time() << "; ";
          // std::cout << "torque= " << torque.format(VecFmt) << "\n";
        }
        else if (left_stance == 0 && right_stance == 0)
        {
          MatrixX<T> Me(nf + ny, nv_);
          VectorX<T> He(nf + ny);
          Me << M.topRows(nf), -Jv_yd;
          He << (G).head(nf), ydd_des;
          acc_des = -Me.lu().solve(He);
          torque = M.bottomRows(na_) * acc_des + (G).bottomRows(na_);
          // std::cout << "Me= " << Me.format(CleanFmt) << "\n";
          // std::cout << "He= " << He.format(VecFmt) << "\n";
          // std::cout << "torque= " << torque.format(VecFmt) << "\n";
        }
      }

      //   const auto end = std::chrono::steady_clock::now();
      //   const std::chrono::duration calc_elapsed_ns = end - start;
      //   using namespace std::chrono_literals;
      //   calc_elapsed = calc_elapsed_ns / 1us;
      //   begin_calc = start.time_since_epoch() / 1us;
      //   end_calc = end.time_since_epoch() / 1us;
      //   //   const std::chrono::duration before_pinv = start_pinv - start;
      //   //   std::cout << "before pinv elapsed(ns): " << before_pinv.count();
      //   //   const std::chrono::duration pinv_elapsed = end_pinv - start_pinv;
      //   //   std::cout << "; pinv elapsed(ns): " << pinv_elapsed.count();
      //   //   const std::chrono::duration after_pinv = end - end_pinv;
      //   //   std::cout << "; after pinv elapsed(ns): " << after_pinv.count() << '\n';
      //   if (torque.minCoeff() < -120 || torque.maxCoeff() > 120 || false) {
      //     std::cout.setf(std::ios::fixed);
      //     std::cout.precision(3);
      //     Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ",", "\n", " ", ";",
      //                              "[", "];\n");
      //     Eigen::IOFormat VecFmt(Eigen::StreamPrecision, 0, ",", "", " ", ";", "[",
      //                            "];");
      //     std::cout << "time: " << context.get_time() << "\n";
      //     std::cout << "v=" << v.format(VecFmt) << "\n";
      //     std::cout << "torque= " << torque.format(VecFmt) << "\n";
      //     std::cout << "G= " << G.format(VecFmt) << "\n";
      //     std::cout << "Cv= " << Cv.format(VecFmt) << "\n";
      //     std::cout << "H_joint= " << H2.format(VecFmt) << "\n";
      //     std::cout << "H_float= " << H1.head(6).format(VecFmt) << "\n";
      //     std::cout << "hdd_des= " << H1.tail(10).format(VecFmt) << "\n";
      //     std::cout << "acc_des= " << lambda.head(nv).format(VecFmt) << "\n";
      //     std::cout << "des_GRF= " << lambda.tail(ng).format(VecFmt) << "\n";
      //     std::cout << "Jst_W= " << Jv_v_WCcm.format(CleanFmt) << "\n";
      //     std::cout << "Jh= " << Jv_hd.format(CleanFmt) << "\n";
      //     std::cout << "Jg= " << Jv_g_WSto.format(CleanFmt) << "\n";
      //     std::cout << "M= " << M.format(CleanFmt) << "\n";
      //     std::cout << "bias_g= " << gvBias_WSto.format(VecFmt) << "\n";
      //     std::cout << "M11= " << M11.format(CleanFmt) << "\n";
      //     std::cout << "hello"
      //               << "\n";
      //   }
    }
    template class ::my_drake_examples::biped::FeedbackController<double>;

    // DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    //     class ::my_drake_examples::biped::FeedbackController)

  } // namespace biped
} // namespace my_drake_examples
