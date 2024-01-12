#include "mit_estimator.h"

#include <drake/multibody/math/spatial_algebra.h>

#include <iostream>

namespace my_drake_examples
{
    namespace biped
    {

        using drake::Matrix3;
        using drake::MatrixX;
        using drake::Quaternion;
        using drake::Vector3;
        using drake::VectorX;
        using drake::math::RollPitchYaw;
        using drake::math::RotationMatrix;
        using drake::multibody::JacobianWrtVariable;
        using drake::multibody::SpatialAcceleration;
        using drake::systems::Context;
        using drake::systems::EventStatus;
        using drake::systems::State;

        template <typename T>
        MitEstimator<T>::MitEstimator(double time_step, double offset,
                                      const drake::multibody::MultibodyPlant<T> &plant)
            : time_step_(time_step), offset_(offset), plant_(plant)
        {
            plant_context_ = plant_.CreateDefaultContext();
            na_ = plant_.num_actuated_dofs();
            nq_ = plant_.num_positions();
            nv_ = plant_.num_velocities();
            g_ = plant_.gravity_field().gravity_vector();
            // set state transition matrix
            A_.setZero();
            A_.topLeftCorner(3, 3) = Matrix3<T>::Identity();
            A_.template block<3, 3>(0, 3) = time_step * Matrix3<T>::Identity();
            A_.template block<3, 3>(3, 3) = Matrix3<T>::Identity();
            A_.template block<6, 6>(6, 6) = MatrixX<T>::Identity(6, 6);
            // set state input matrix
            B_.setZero();
            B_.template block<3, 3>(3, 0) = time_step * Matrix3<T>::Identity();
            // set output matrix
            // B for Torso, Foot for {Lfoot, Rfoot}
            MatrixX<T> JpvB_p_BFoot_W = MatrixX<T>::Zero(3, 6);
            JpvB_p_BFoot_W << -Matrix3<T>::Identity(), Matrix3<T>::Zero();
            MatrixX<T> JpvB_v_BFoot_W = MatrixX<T>::Zero(3, 6);
            JpvB_v_BFoot_W << Matrix3<T>::Zero(), -Matrix3<T>::Identity();
            C_.setZero();
            C_.template block<3, 6>(0, 0) = JpvB_p_BFoot_W;
            C_.template block<3, 6>(3, 0) = JpvB_p_BFoot_W;
            C_.template block<3, 6>(6, 0) = JpvB_v_BFoot_W;
            C_.template block<3, 6>(9, 0) = JpvB_v_BFoot_W;
            // Jp_WFoot_p_BFoot_W
            C_.template block<6, 6>(0, 6) = MatrixX<T>::Identity(6, 6);
            // set input noise factor (在Cheetah-Software的实现中，它需要额外乘参数)
            Q_.setZero();
            Q_.template block<3, 3>(0, 0) = time_step * 1 * Matrix3<T>::Identity();
            Q_.template block<3, 3>(3, 3) = time_step * 1 * Matrix3<T>::Identity();
            Q_.template block<2, 2>(3, 3) *= 9.8;
            Q_.template block<6, 6>(6, 6) = time_step * MatrixX<T>::Identity(6, 6);
            // set measurement noise factor
            R_.setIdentity();

            q_joint_input_port_ = &this->DeclareVectorInputPort("q_joint", na_);
            v_joint_input_port_ = &this->DeclareVectorInputPort("v_joint", na_);
            acc_imu_input_port_ = &this->DeclareVectorInputPort("acc_imu", 3);
            quat_imu_input_port_ = &this->DeclareVectorInputPort("quat_imu", 4);
            gyro_imu_input_port_ = &this->DeclareVectorInputPort("gyro_imu", 3);
            stance_phase_input_port_ = &this->DeclareVectorInputPort("stance_phase", 2);

            xhat_id_ = this->DeclareDiscreteState(3 + 3 + 2 * 3);
            xhat_output_port_ = &this->DeclareStateOutputPort("xhat", xhat_id_);
            Phat_id_ =
                this->DeclareAbstractState(drake::Value<Eigen::Matrix<T, 12, 12>>());

            state_id_ = this->DeclareDiscreteState(nq_ + nv_);
            state_output_port_ = &this->DeclareStateOutputPort("state_output", state_id_);

            this->DeclareInitializationUnrestrictedUpdateEvent(&MitEstimator::Init);
            this->DeclarePeriodicUnrestrictedUpdateEvent(time_step, offset,
                                                         &MitEstimator::Update);
        }

        template <typename T>
        EventStatus MitEstimator<T>::Init(const Context<T> &context,
                                          State<T> *next_state) const
        {
            auto &Phat =
                next_state->template get_mutable_abstract_state<Eigen::Matrix<T, 12, 12>>(
                    Phat_id_);
            Phat = 100 * MatrixX<T>::Identity(12, 12);
            return EventStatus::Succeeded();
        }

        template <typename T>
        void MitEstimator<T>::Update(const Context<T> &context,
                                     State<T> *next_state) const
        {
            std::cout.setf(std::ios::fixed);
            std::cout.precision(3);
            Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ",", "\n", " ", ";", "[",
                                     "];\n");
            Eigen::IOFormat VecFmt(Eigen::StreamPrecision, 0, ",", "", " ", ";", "[",
                                   "];");

            const auto &q_joint = get_q_joint_input_port().Eval(context);
            const auto &v_joint = get_v_joint_input_port().Eval(context);
            const auto &a_IS_S =
                get_acc_imu_input_port().Eval(context); // I是无重力惯性系
            const auto &quat_WS = get_quat_imu_input_port().Eval(context);
            const auto &w_WS_S = get_gyro_imu_input_port().Eval(context);
            const auto &stance_phase = get_stance_phase_input_port().Eval(context);
            const auto &old_state = context.get_discrete_state(state_id_).value();
            const auto &old_q = old_state.head(nq_);
            const auto &old_v = old_state.tail(nv_);
            const auto &old_xhat = context.get_discrete_state(xhat_id_).value();
            const auto &old_Phat =
                context.template get_abstract_state<Eigen::Matrix<T, 12, 12>>(Phat_id_);

            auto xhat =
                next_state->get_mutable_discrete_state(xhat_id_).get_mutable_value();
            auto &Phat =
                next_state->template get_mutable_abstract_state<Eigen::Matrix<T, 12, 12>>(
                    Phat_id_);

            auto state =
                next_state->get_mutable_discrete_state(state_id_).get_mutable_value();
            auto q = state.head(nq_);
            auto v = state.tail(nv_);

            const auto &torso_frame = plant_.GetFrameByName("torso");
            const auto &lfoot_frame = plant_.GetFrameByName("l_foot_x");
            const auto &rfoot_frame = plant_.GetFrameByName("r_foot_x");
            // S为IMU系，B为torso
            const auto &X_BS = plant_.GetFrameByName("Mti").GetFixedPoseInBodyFrame();
            const auto R_WS = RotationMatrix<T>(
                Quaternion<T>(quat_WS(0), quat_WS(1), quat_WS(2), quat_WS(3)));

            auto euler_WS = RollPitchYaw<T>(R_WS);
            // euler_WS.set_yaw_angle(0);
            const auto &R_WS_no_yaw = euler_WS.ToRotationMatrix();

            const auto w_WB = R_WS * w_WS_S; // S fixed in B, so w_WB==w_WS
            const auto R_WB = R_WS * X_BS.rotation().inverse();
            // SpatialAcceleration<T> A_IB_S =
            //     A_IS_S.Shift(X_BS.inverse().translation(), w_WS_S);
            // // order: Quaternion(w,x,y,z)

            const auto a_IS_W = R_WS * a_IS_S;
            const auto a_WS = a_IS_W + g_;
            SpatialAcceleration<T> A_WS(Vector3<T>::Zero(), a_WS);
            const auto p_SB_W = R_WS * X_BS.inverse().translation();
            const auto A_WB = A_WS.Shift(p_SB_W, w_WB); // again w_WB==w_WS
            const auto &a_WB = A_WB.translational();
            // const auto& a_WB = R_WS * A_IB_S.translational() + g_;

            // pass-through joint position and velocity
            q.tail(na_) = q_joint;
            v.tail(na_) = v_joint;

            // 目前直接使用Xsens Mti630估计的姿态进行浮动基姿态计算
            q.head(4) = R_WB.ToQuaternionAsVector4();
            v.head(3) = w_WB;

            // pass-through 接触检测
            VectorX<T> leg_stance = stance_phase;

            // 浮动基预测
            VectorX<T> xcheck = A_ * old_xhat + B_ * a_WB;
            // VectorX<T> xcheck = A_ * old_xhat + B_ * a_IS_S;
            // 浮动基更新
            plant_.SetPositionsAndVelocities(plant_context_.get(), state);
            VectorX<T> y(12);
            MatrixX<T> Q = MatrixX<T>::Identity(12, 12);
            Q.template block<3, 3>(0, 0) = 0.002 * Q_.template block<3, 3>(0, 0);
            Q.template block<3, 3>(3, 3) = 0.002 * Q_.template block<3, 3>(3, 3);
            Q.template block<6, 6>(6, 6) = 0.002 * Q_.template block<6, 6>(6, 6);
            MatrixX<T> R = MatrixX<T>::Identity(12, 12);
            R.template block<6, 6>(0, 0) = 0.01 * R_.template block<6, 6>(0, 0);
            R.template block<6, 6>(6, 6) = 0.1 * R_.template block<6, 6>(6, 6);
            for (int i = 0; i < 2; i++)
            {
                Q.template block<3, 3>(6 + 3 * i, 6 + 3 * i) *=
                    1 + (1 - leg_stance(i)) * 10000;
                R.template block<3, 3>(6 + 3 * i, 6 + 3 * i) *=
                    1 + (1 - leg_stance(i)) * 10000;
            }
            const auto &X_BLfoot =
                plant_.CalcRelativeTransform(*plant_context_, torso_frame, lfoot_frame);
            const auto &p_BLfoot_W = R_WB * X_BLfoot.translation();
            const auto &X_BRfoot =
                plant_.CalcRelativeTransform(*plant_context_, torso_frame, rfoot_frame);
            const auto &p_BRfoot_W = R_WB * X_BRfoot.translation();
            y.head(6) << p_BLfoot_W, p_BRfoot_W;

            MatrixX<T> Jv_v_WB(3, nv_);
            plant_.CalcJacobianTranslationalVelocity(
                *plant_context_, JacobianWrtVariable::kV, torso_frame, Vector3<T>::Zero(),
                plant_.world_frame(), plant_.world_frame(), &Jv_v_WB);
            MatrixX<T> Jv_v_WLfoot(3, nv_);
            plant_.CalcJacobianTranslationalVelocity(
                *plant_context_, JacobianWrtVariable::kV, lfoot_frame, Vector3<T>::Zero(),
                plant_.world_frame(), plant_.world_frame(), &Jv_v_WLfoot);
            MatrixX<T> Jv_v_WRfoot(3, nv_);
            plant_.CalcJacobianTranslationalVelocity(
                *plant_context_, JacobianWrtVariable::kV, rfoot_frame, Vector3<T>::Zero(),
                plant_.world_frame(), plant_.world_frame(), &Jv_v_WRfoot);
            y.tail(6) << (Jv_v_WLfoot - Jv_v_WB) * v, (Jv_v_WRfoot - Jv_v_WB) * v;
            MatrixX<T> Pcheck = A_ * old_Phat * A_.transpose() + Q;
            MatrixX<T> S = C_ * Pcheck * C_.transpose() + R;
            MatrixX<T> K = Pcheck * C_.transpose() * S.inverse();
            // K.setZero();
            xhat = xcheck + K * (y - C_ * xcheck);
            // if (leg_stance(0) > 0.5) {
            //   xhat.template segment<3>(3) = -y.template segment<3>(6);
            //   xhat.head(3) = xhat.template segment<3>(6) - p_BLfoot_W;
            // } else {
            //   xhat.template segment<3>(3) = -y.template segment<3>(9);
            //   xhat.head(3) = xhat.template segment<3>(9) - p_BRfoot_W;
            // }

            Phat = (MatrixX<T>::Identity(12, 12) - K * C_) * Pcheck;

            q.template segment<3>(4) = xhat.head(3);
            v.template segment<3>(3) = xhat.template segment<3>(3);
        }

        template class ::my_drake_examples::biped::MitEstimator<double>;

    } // namespace biped
} // namespace my_drake_examples
