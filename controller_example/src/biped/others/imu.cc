#include "imu.h"

namespace my_drake_examples
{
    namespace biped
    {

        using drake::math::RigidTransform;
        using drake::systems::DiagramBuilder;
        using drake::systems::sensors::Accelerometer;
        using drake::systems::sensors::Gyroscope;

        template <typename T>
        IMU<T>::IMU(const drake::multibody::Body<T> &body,
                    const drake::math::RigidTransform<double> &X_BS,
                    const Eigen::Vector3d &gravity_vector)
            : body_(body),
              X_BS_(X_BS),
              gravity_vector_(gravity_vector)
        {
            drake::systems::DiagramBuilder<T> builder;

            auto quaternion_sensor = builder.template AddSystem<QuaternionSensor<T>>(body, X_BS);
            quaternion_sensor->set_name("quaternion_sensor");
            auto accelerometer = builder.template AddSystem<drake::systems::sensors::Accelerometer<T>>(body, X_BS, gravity_vector_);
            accelerometer->set_name("accelerometer");
            auto gyroscope = builder.template AddSystem<drake::systems::sensors::Gyroscope<T>>(body, X_BS);
            gyroscope->set_name("gyroscope");
            /*
            poses --------------------> | quaternion sensor | ---> quaternion orientation
                               |
                               |  TODO: 找到更好看的表示线绕过去的字符画技巧
            velocities---------|------> |           |
                             | |        | gyroscope | -----------> angular velocity
                             | |------> |           |
                             | |
                             | -------> |               |
                             ---------> | accelerometer | -------> acceleration
            accelerations-------------> |               |

            */
            input_port_index_body_poses_ = builder.ExportInput(
                quaternion_sensor->get_body_poses_input_port(), "body_poses");
            builder.ConnectInput(input_port_index_body_poses_,
                                 accelerometer->get_body_poses_input_port());
            builder.ConnectInput(input_port_index_body_poses_,
                                 gyroscope->get_body_poses_input_port());
            input_port_index_body_velocities_ = builder.ExportInput(
                accelerometer->get_body_velocities_input_port(), "body_velocities");
            builder.ConnectInput(input_port_index_body_velocities_,
                                 gyroscope->get_body_velocities_input_port());
            input_port_index_body_accelerations_ = builder.ExportInput(
                accelerometer->get_body_accelerations_input_port(), "body_accelerations");

            output_port_index_measurement_quaternion_orientation_ =
                builder.ExportOutput(quaternion_sensor->get_output_port(), "measurement_quaternion_orientation");
            output_port_index_measurement_acceleration_ =
                builder.ExportOutput(accelerometer->get_output_port(), "measurement_acceleration");
            output_port_index_measurement_angular_velocity_ =
                builder.ExportOutput(gyroscope->get_output_port(), "measurement_angular_velocity");

            builder.BuildInto(this);
        }

        template <typename T>
        const IMU<T> &IMU<T>::AddToDiagram(
            const drake::multibody::Body<T> &body, const RigidTransform<double> &X_BS,
            const Eigen::Vector3d &gravity_vector,
            const drake::multibody::MultibodyPlant<T> &plant, DiagramBuilder<T> *builder)
        {
            const auto &imu = *builder->template AddSystem<IMU<T>>(body, X_BS, gravity_vector);
            builder->Connect(plant.get_body_poses_output_port(),
                             imu.get_input_port_body_poses());
            builder->Connect(plant.get_body_spatial_velocities_output_port(),
                             imu.get_input_port_body_velocities());
            builder->Connect(plant.get_body_spatial_accelerations_output_port(),
                             imu.get_input_port_body_accelerations());
            return imu;
        }

        template <typename T>
        template <typename U>
        IMU<T>::IMU(const IMU<U> &other) : IMU(other.body_(), other.pose()) {}

        DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
            class ::my_drake_examples::biped::IMU)

    } // namespace biped
} // namespace my_drake_examples
