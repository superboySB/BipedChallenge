#include "dummy_estimator.h"

#include <Eigen/QR>
#include <Eigen/SVD>
#include <iostream>

namespace my_drake_examples
{
  namespace biped
  {

    using drake::multibody::SpatialForce;

    template <typename T>
    DummyEstimator<T>::DummyEstimator(
        const drake::multibody::MultibodyPlant<T> &plant)
        : plant_(plant)
    {
      state_input_port_ =
          &this->DeclareVectorInputPort("state", plant_.num_multibody_states());
      a_input_port_ = &this->DeclareVectorInputPort("a", plant_.num_velocities());
      q_output_port_ = &this->DeclareVectorOutputPort("q", plant_.num_positions(),
                                                      &DummyEstimator::CalcQ);
      v_output_port_ = &this->DeclareVectorOutputPort("v", plant_.num_velocities(),
                                                      &DummyEstimator::CalcV);
      torque_input_port_ =
          &this->DeclareVectorInputPort("torque", plant_.num_actuated_dofs());
      stanceLeg_output_port_ = &this->DeclareAbstractOutputPort(
          "stanceLeg", &DummyEstimator::CalcStanceLeg);
      GRF_output_port_ =
          &this->DeclareVectorOutputPort("GRF", 12, &DummyEstimator::CalcGRF);

      plant_context_ = plant_.CreateDefaultContext();
    }

    template <typename T>
    void DummyEstimator<T>::CalcQ(const drake::systems::Context<T> &context,
                                  drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector = state.head(plant_.num_positions());
    }

    template <typename T>
    void DummyEstimator<T>::CalcV(const drake::systems::Context<T> &context,
                                  drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector = state.tail(plant_.num_velocities());
    }

    template <typename T>
    void DummyEstimator<T>::CalcStanceLeg(const drake::systems::Context<T> &context,
                                          int *stanceLeg) const
    {
      const auto &state = get_state_input_port().Eval(context);
      plant_.SetPositionsAndVelocities(plant_context_.get(), state);
      const auto &lfoot = plant_.GetBodyByName("l_foot_x");
      const auto &rfoot = plant_.GetBodyByName("r_foot_x");
      const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_, lfoot);
      const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_, rfoot);
      const auto &V_WLfoot =
          plant_.EvalBodySpatialVelocityInWorld(*plant_context_, lfoot);
      const auto &V_WRfoot =
          plant_.EvalBodySpatialVelocityInWorld(*plant_context_, rfoot);
      // 开摆！
      int gait_count = (context.get_time() + 0.3) / 0.4;
      if (gait_count % 2)
      {
        *stanceLeg = 1;
      }
      else
      {
        *stanceLeg = -1;
      }
      // const auto& a = get_a_input_port().Eval(context);
      // std::cout << "real acc:" << a.segment(3, 3).transpose() << "\n";
    }

    template <typename T>
    void DummyEstimator<T>::CalcGRF(const drake::systems::Context<T> &context,
                                    drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      const auto &a = get_a_input_port().Eval(context);
      const auto &torque = get_torque_input_port().Eval(context);
      auto out_GRF = output->get_mutable_value();

      plant_.SetPositionsAndVelocities(plant_context_.get(), state);
      drake::MatrixX<T> M = drake::MatrixX<T>::Identity(16, 16);
      plant_.CalcMassMatrix(*plant_context_, &M);
      drake::VectorX<T> Cv = drake::VectorX<T>::Zero(16);
      plant_.CalcBiasTerm(*plant_context_, &Cv);
      drake::VectorX<T> G = -plant_.CalcGravityGeneralizedForces(*plant_context_);
      drake::MatrixX<T> B = plant_.MakeActuationMatrix();
      drake::MatrixX<T> Jv_V_WFleft = drake::MatrixX<T>::Zero(6, 16);
      drake::MatrixX<T> Jv_V_WFright(6, 16);
      const auto &Fleft_frame = plant_.GetBodyByName("l_foot_x").body_frame();
      const auto &Fright_frame = plant_.GetBodyByName("r_foot_x").body_frame();
      plant_.CalcJacobianSpatialVelocity(
          *plant_context_, drake::multibody::JacobianWrtVariable::kV, Fleft_frame,
          drake::Vector3<T>::Zero(), plant_.world_frame(), plant_.world_frame(),
          &Jv_V_WFleft);
      plant_.CalcJacobianSpatialVelocity(
          *plant_context_, drake::multibody::JacobianWrtVariable::kV, Fright_frame,
          drake::Vector3<T>::Zero(), plant_.world_frame(), plant_.world_frame(),
          &Jv_V_WFright);
      drake::MatrixX<T> J(12, 16);
      J << Jv_V_WFleft, Jv_V_WFright;
      drake::VectorX<T> b(16);
      b = M * a + G + Cv - B * torque;
      drake::MatrixX<T> A = J.transpose();
      // A.row(9)*=0.01;b(9)*=0.01;
      // A.row(14)*=0.01;b(14)*=0.01;
      // std::cout.setf(std::ios::fixed);
      // std::cout.precision(3);
      Eigen::IOFormat CleanFmt(Eigen::StreamPrecision, 0, ",", "\n", " ", ";", "[",
                               "];\n");
      Eigen::IOFormat VecFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ",", "",
                             " ", ";", "[", "];");
      // std::cout << "J' =\n" << A.format(CleanFmt);
      // std::cout << "b = " << b.format(VecFmt) << "\n\n";
      const auto start = std::chrono::steady_clock::now();
      // out_GRF = J.transpose().completeOrthogonalDecomposition().solve(b);
      // out_GRF = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
      out_GRF = A.colPivHouseholderQr().solve(b);
      // out_GRF = (A.transpose()*A).ldlt().solve(A.transpose()*b);
      const auto end = std::chrono::steady_clock::now();
      const std::chrono::duration calc_elapsed_ns = end - start;
      using namespace std::chrono_literals;
      auto calc_elapsed = calc_elapsed_ns / 1us;
      // std::cout << "calc elapsed: " << calc_elapsed << "\n";
      // std::cout << "residual = " << (A*out_GRF - b).format(VecFmt) << "\n";
    }

    template class ::my_drake_examples::biped::DummyEstimator<double>;

  } // namespace biped
} // namespace my_drake_examples
