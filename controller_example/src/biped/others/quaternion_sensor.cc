// TODO: license?

#include "quaternion_sensor.h"

#include <drake/multibody/math/spatial_algebra.h>

namespace my_drake_examples
{
    namespace biped
    {

        // using drake::xxx::Xxx 似乎是一种偏好，没有绝对的标准
        using drake::math::RigidTransform;
        using drake::math::RotationMatrix;
        using drake::systems::BasicVector;
        using drake::systems::Context;
        using drake::systems::DiagramBuilder;
        using drake::systems::LeafSystem;
        using drake::systems::SystemTypeTag;

        template <typename T>
        QuaternionSensor<T>::QuaternionSensor(const drake::multibody::Body<T> &body,
                                              const RigidTransform<double> &X_BS)
            : QuaternionSensor(body.index(), X_BS) {}

        template <typename T>
        QuaternionSensor<T>::QuaternionSensor(
            const drake::multibody::BodyIndex &body_index,
            const RigidTransform<double> &X_BS)
            : LeafSystem<T>(SystemTypeTag<QuaternionSensor>{}),
              body_index_(body_index),
              X_BS_(X_BS)
        {
            // Declare measurement output port.
            measurement_output_port_ = &this->DeclareVectorOutputPort(
                "measurement", 4, &QuaternionSensor<T>::CalcOutput);

            body_poses_input_port_ = &this->DeclareAbstractInputPort(
                "body_poses", drake::Value<std::vector<RigidTransform<T>>>());
        }

        template <typename T>
        void QuaternionSensor<T>::CalcOutput(const Context<T> &context,
                                             BasicVector<T> *output) const
        {
            const auto &X_WB =
                get_body_poses_input_port().template Eval<std::vector<RigidTransform<T>>>(
                    context)[body_index_];

            // Calculate rotation from world to gyroscope: R_SW = R_SB * R_BW.
            const auto R_SB = X_BS_.rotation().matrix().template cast<T>().transpose();
            const auto R_BW = X_WB.rotation().matrix().transpose();
            const auto R_SW = R_SB * R_BW;

            // Re-express in local frame and return.
            output->SetFromVector(
                RotationMatrix<T>(R_SW.transpose()).ToQuaternionAsVector4());
        }

        template <typename T>
        const QuaternionSensor<T> &QuaternionSensor<T>::AddToDiagram(
            const drake::multibody::Body<T> &body, const RigidTransform<double> &X_BS,
            const drake::multibody::MultibodyPlant<T> &plant,
            DiagramBuilder<T> *builder)
        {
            const auto &quaternion_sensor =
                *builder->template AddSystem<QuaternionSensor<T>>(body, X_BS);
            builder->Connect(plant.get_body_poses_output_port(),
                             quaternion_sensor.get_body_poses_input_port());
            return quaternion_sensor;
        }

        template <typename T>
        template <typename U>
        QuaternionSensor<T>::QuaternionSensor(const QuaternionSensor<U> &other)
            : QuaternionSensor(other.body_index(), other.pose()) {}

        // TODO: figure out what this mean
        DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
            class ::my_drake_examples::biped::QuaternionSensor)

    } // namespace biped
} // namespace my_drake_examples
