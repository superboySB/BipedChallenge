#pragma once

#include <vector>
// TODO: include memory?

#include <drake/common/drake_copyable.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/multibody_tree_indexes.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/systems/framework/leaf_system.h>

namespace my_drake_examples
{
  namespace biped
  {

    /// Sensor to represent an ideal quaternion sensor(part of a IMU). Currently does not
    /// represent noise or bias, but this could and should be added at a later
    /// date. This sensor measures the quaternion of orientation of a given body B relative
    /// to the world frame. The sensor frame S is rigidly affixed to the given
    /// body B. The rotation matrix of measurement, written R_WS_S, is expressed in the coordinates
    /// of frame S.

    /// There are one input to this sensor (nominally from a MultibodyPlant):
    ///   1. A vector of body poses (e.g. plant.get_body_poses_output_port())
    ///
    /// This class is therefore defined by:
    ///   1. The Body to which this sensor is rigidly affixed,
    ///   2. The pose of the sensor frame in the body frame.
    /// Note that the translational component of the transformation is not strictly
    /// needed by a quaternion sensor, as the position of the sensor does not affect the
    /// measurement. However, as a sensor does have a physical location, it is
    /// included here for completeness and in case it might be useful for display or
    /// other purposes.
    ///
    /// @system
    /// name: QuaternionSensor
    /// input_ports:
    /// - body_poses
    /// output_ports:
    /// - measurement
    /// @endsystem
    ///
    template <typename T>
    class QuaternionSensor final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(QuaternionSensor)

      /// Constructor for %QuaternionSensor using full transform.
      /// @param body the body B to which the sensor is affixed
      /// @param X_BS the pose of sensor frame S in body B
      QuaternionSensor(const drake::multibody::Body<T> &body,
                       const drake::math::RigidTransform<double> &X_BS);

      /// Scalar-converting copy constructor.  See @ref system_scalar_conversion.
      template <typename U>
      explicit QuaternionSensor(const QuaternionSensor<U> &);

      const drake::systems::InputPort<T> &get_body_poses_input_port() const
      {
        return *body_poses_input_port_;
      }

      /// Returns the index of the Body that was supplied in the constructor.
      const drake::multibody::BodyIndex &body_index() const { return body_index_; }

      /// Gets X_BS, the pose of sensor frame S in body B.
      const drake::math::RigidTransform<double> &pose() const
      {
        return X_BS_;
      }

      /// Static factory method that creates a %QuaternionSensor object and connects
      /// it to the given plant. Modifies a Diagram by connecting the input ports
      /// of the new %QuaternionSensor to the appropriate output ports of a
      /// MultibodyPlant. Must be called during Diagram building and given the
      /// appropriate builder. This is a convenience method to simplify some common
      /// boilerplate of Diagram wiring. Specifically, this makes one TODO: 报告gyroscope的注释偷懒写错了哈哈 connection:
      ///
      /// 1. plant.get_body_poses_output_port() to this.get_body_poses_input_port()
      /// @param body the body B to which the sensor is affixed
      /// @param X_BS X_BS the pose of sensor frame S in body B
      /// @param plant the plant to which the sensor will be connected
      /// @param builder a pointer to the DiagramBuilder
      static const QuaternionSensor &AddToDiagram(
          const drake::multibody::Body<T> &body, const drake::math::RigidTransform<double> &X_BS,
          const drake::multibody::MultibodyPlant<T> &plant, drake::systems::DiagramBuilder<T> *builder);

    private:
      QuaternionSensor(const drake::multibody::BodyIndex &body_index,
                       const drake::math::RigidTransform<double> &X_BS);

      // Computes the transformed signal.
      void CalcOutput(const drake::systems::Context<T> &context, drake::systems::BasicVector<T> *output) const;

      const drake::multibody::BodyIndex body_index_;
      const drake::math::RigidTransform<double> X_BS_;
      const drake::systems::InputPort<T> *body_poses_input_port_{nullptr};
      const drake::systems::OutputPort<T> *measurement_output_port_{nullptr};
    };

  } // namespace biped
} // namespace my_drake_examples
