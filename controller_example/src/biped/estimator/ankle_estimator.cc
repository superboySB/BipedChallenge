#include "ankle_estimator.h"

#include <iostream>
#include <Eigen/QR>
#include <Eigen/SVD>

namespace my_drake_examples
{
    namespace biped
    {

        using drake::multibody::SpatialForce;

        template <typename T>
        AnkleEstimator<T>::AnkleEstimator(const drake::multibody::MultibodyPlant<T> &plant)
            : plant_(plant)
        {
            state_input_port_ = &this->DeclareVectorInputPort(
                "state", plant_.num_multibody_states());
            reaction_force_input_port_ = &this->DeclareAbstractInputPort(
                "reaction_force", drake::Value<std::vector<SpatialForce<T>>>{});
            ankle_force_port_ = &this->DeclareVectorOutputPort(
                "ankle_force", 12, &AnkleEstimator::CalcAnkleForce);

            plant_context_ = plant_.CreateDefaultContext();
        }

        template <typename T>
        void AnkleEstimator<T>::CalcAnkleForce(const drake::systems::Context<T> &context,
                                               drake::systems::BasicVector<T> *output) const
        {
            const auto &state = get_state_input_port().Eval(context);
            const auto &reaction_forces = get_reaction_force_input_port()
                                              .template Eval<std::vector<SpatialForce<T>>>(context);
            auto ankle_force = output->get_mutable_value();

            plant_.SetPositionsAndVelocities(plant_context_.get(), state);

            // 临时使用常数描述带弹簧plant的index偏移
            const auto &left_force = reaction_forces[plant_.GetJointByName("l_ankle").index()];
            const auto &right_force = reaction_forces[plant_.GetJointByName("r_ankle").index()];

            const auto &X_WLankle = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                               plant_.GetBodyByName("l_foot"));
            const auto &X_WRankle = plant_.EvalBodyPoseInWorld(*plant_context_,
                                                               plant_.GetBodyByName("r_foot"));

            ankle_force.head(3) = X_WLankle * -left_force.rotational();
            ankle_force.template segment<3>(3) = X_WLankle * -left_force.translational();
            ankle_force.template segment<3>(6) = X_WRankle * -right_force.rotational();
            ankle_force.tail(3) = X_WRankle * -right_force.translational();
        }

        template class ::my_drake_examples::biped::AnkleEstimator<double>;

    } // namespace biped
} // namespace my_drake_examples
