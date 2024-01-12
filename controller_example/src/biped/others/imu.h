#pragma once

#include <drake/systems/sensors/accelerometer.h>
#include <drake/systems/sensors/gyroscope.h>
// TODO: maybe include xxx/xxx/quaternion_sensor.h?
#include "quaternion_sensor.h"

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class IMU : public drake::systems::Diagram<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IMU)
      // TODO: figure out what "explicit" mean
      IMU(const drake::multibody::Body<T> &body,
          const drake::math::RigidTransform<double> &X_BS,
          const Eigen::Vector3d &gravity_vector = Eigen::Vector3d::Zero());

      /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
      template <typename U>
      explicit IMU(const IMU<U> &);

      const drake::systems::InputPort<T> &get_input_port_body_poses() const
      {
        return drake::systems::Diagram<T>::get_input_port(input_port_index_body_poses_);
      }

      const drake::systems::InputPort<T> &get_input_port_body_velocities() const
      {
        return drake::systems::Diagram<T>::get_input_port(input_port_index_body_velocities_);
      }

      const drake::systems::InputPort<T> &get_input_port_body_accelerations() const
      {
        return drake::systems::Diagram<T>::get_input_port(input_port_index_body_accelerations_);
      }

      const drake::systems::OutputPort<T> &get_output_port_measurement_acceleration() const
      {
        return drake::systems::Diagram<T>::get_output_port(output_port_index_measurement_acceleration_);
      }

      const drake::systems::OutputPort<T> &get_output_port_measurement_angular_velocity() const
      {
        return drake::systems::Diagram<T>::get_output_port(output_port_index_measurement_angular_velocity_);
      }

      const drake::systems::OutputPort<T> &get_output_port_measurement_quaternion_orientation() const
      {
        return drake::systems::Diagram<T>::get_output_port(output_port_index_measurement_quaternion_orientation_);
      }

      /// Returns the Body that was supplied in the constructor.
      const drake::multibody::Body<T> &body() const { return body_; }

      /// Returns the gravity vector supplied in the constructor, or zero if none.
      const Eigen::Vector3d &gravity_vector() const { return gravity_vector_; }

      /// Gets X_BS, the pose of sensor frame S in body B.
      const drake::math::RigidTransform<double> &pose() const
      {
        return X_BS_;
      }

      static const IMU &AddToDiagram(
          const drake::multibody::Body<T> &body, const drake::math::RigidTransform<double> &X_BS,
          const Eigen::Vector3d &gravity_vector,
          const drake::multibody::MultibodyPlant<T> &plant, drake::systems::DiagramBuilder<T> *builder);

    private:
      const drake::multibody::Body<T> &body_;
      const drake::math::RigidTransform<double> X_BS_;
      const Eigen::Vector3d gravity_vector_;
      drake::systems::InputPortIndex input_port_index_body_poses_;
      drake::systems::InputPortIndex input_port_index_body_velocities_;
      drake::systems::InputPortIndex input_port_index_body_accelerations_;
      drake::systems::OutputPortIndex output_port_index_measurement_acceleration_;
      drake::systems::OutputPortIndex output_port_index_measurement_angular_velocity_;
      drake::systems::OutputPortIndex output_port_index_measurement_quaternion_orientation_;
    };

  } // namespace biped
} // namespace my_drake_examples
