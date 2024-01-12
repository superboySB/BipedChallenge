#include "jump_estimator.h"

#include <iostream>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    JumpEstimator<T>::JumpEstimator(
        const drake::multibody::MultibodyPlant<T> &plant)
        : plant_(plant)
    {
      state_input_port_ =
          &this->DeclareVectorInputPort("state", plant_.num_multibody_states());
      q_output_port_ = &this->DeclareVectorOutputPort("q", plant_.num_positions(),
                                                      &JumpEstimator::CalcQ);
      v_output_port_ = &this->DeclareVectorOutputPort("v", plant_.num_velocities(),
                                                      &JumpEstimator::CalcV);
      left_stance_output_port_ = &this->DeclareVectorOutputPort(
          "left_stance", 1, &JumpEstimator::CalcLeftStance);
      right_stance_output_port_ = &this->DeclareVectorOutputPort(
          "right_stance", 1, &JumpEstimator::CalcRightStance);

      plant_context_ = plant_.CreateDefaultContext();
    }

    template <typename T>
    void JumpEstimator<T>::CalcQ(const drake::systems::Context<T> &context,
                                 drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector = state.head(plant_.num_positions());
    }

    template <typename T>
    void JumpEstimator<T>::CalcV(const drake::systems::Context<T> &context,
                                 drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      auto output_vector = output->get_mutable_value();

      output_vector = state.tail(plant_.num_velocities());
    }

    template <typename T>
    void JumpEstimator<T>::CalcLeftStance(
        const drake::systems::Context<T> &context,
        drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      plant_.SetPositionsAndVelocities(plant_context_.get(), state);
      getGroundHeight();
      const auto &lfoot = plant_.GetBodyByName("l_foot_x");
      const auto &X_WLfoot = plant_.EvalBodyPoseInWorld(*plant_context_, lfoot);
      const auto &V_WLfoot =
          plant_.EvalBodySpatialVelocityInWorld(*plant_context_, lfoot);

      auto &out = output->get_mutable_value()(0);
      if (X_WLfoot.translation()(2) < ground_height + 0.005)
      {
        out = 1;
      }
      else
      {
        out = 0;
      }
    }

    template <typename T>
    void JumpEstimator<T>::CalcRightStance(
        const drake::systems::Context<T> &context,
        drake::systems::BasicVector<T> *output) const
    {
      const auto &state = get_state_input_port().Eval(context);
      plant_.SetPositionsAndVelocities(plant_context_.get(), state);
      getGroundHeight();
      const auto &rfoot = plant_.GetBodyByName("r_foot_x");
      const auto &X_WRfoot = plant_.EvalBodyPoseInWorld(*plant_context_, rfoot);
      const auto &V_WRfoot =
          plant_.EvalBodySpatialVelocityInWorld(*plant_context_, rfoot);

      auto &out = output->get_mutable_value()(0);

      if (X_WRfoot.translation()(2) < ground_height + 0.005)
      {
        out = 1;
      }
      else
      {
        out = 0;
      }
    }

    template class ::my_drake_examples::biped::JumpEstimator<double>;

  } // namespace biped
} // namespace my_drake_examples
