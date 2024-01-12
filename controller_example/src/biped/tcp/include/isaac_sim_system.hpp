#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>
#include "tcp_server.hpp"
#include "data_converter.hpp"
#include "json.hpp"
#include <thread>

namespace my_drake_examples
{
    namespace biped
    {

        template <typename T>
        class IsaacSimulator final : public drake::systems::LeafSystem<T>, public TcpServer
        {
        public:
            DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(IsaacSimulator)

            IsaacSimulator(double time_step, double offset, const drake::multibody::MultibodyPlant<T> &);

            template <typename U>
            explicit IsaacSimulator(const IsaacSimulator<U> &);

            const drake::systems::InputPort<T> &get_actuation_input_port() const
            {
                return *actuation_input_port_;
            }
            const drake::systems::OutputPort<T> &get_state_output_port() const
            {
                return *state_output_port_;
            }

            const drake::systems::OutputPort<T> &get_generalized_acceleration_output_port() const
            {
                return *generalized_acceleration_output_port_;
            }

            const drake::systems::OutputPort<T> &get_phase_output_port() const
            {
                return *phase_output_port_;
            }

            const drake::systems::OutputPort<T> &get_body_poses_output_port() const
            {
                return *body_poses_output_port_;
            }

            const drake::systems::OutputPort<T> &get_body_spatial_velocities_output_port() const
            {
                return *body_spatial_velocities_output_port_;
            }
            const drake::systems::OutputPort<T> &get_body_spatial_accelerations_output_port() const
            {
                return *body_spatial_accelerations_output_port_;
            }

            const drake::systems::OutputPort<double> &get_turn_rps_output_port() const
            {
                return *turn_rps_output_port_;
            }

            const drake::systems::OutputPort<double> &get_vtarget_output_port() const
            {
                return *vtarget_output_port_;
            }
            void get_initial_pose(Eigen::VectorXd &pose);

        protected:
            std::string processMsg_(const char *msg);

        private:
            void CalcVtarget(const drake::systems::Context<double> &context,
                             drake::systems::BasicVector<double> *output) const;
            void CalcTurnRps(const drake::systems::Context<double> &context,
                             drake::systems::BasicVector<double> *output) const;
            void stateOutput(const drake::systems::Context<double> &context,
                             drake::systems::BasicVector<double> *output) const;
            void accOutput(const drake::systems::Context<double> &context,
                           drake::systems::BasicVector<double> *output) const;
            void phaseOutput(const drake::systems::Context<double> &context,
                           drake::systems::BasicVector<double> *output) const;
            void bodyPoseOutput(const drake::systems::Context<double> &context,
                                std::vector<drake::math::RigidTransform<T>> *output) const;

            void bodyVELOutput(const drake::systems::Context<double> &context,
                               std::vector<drake::multibody::SpatialVelocity<double>> *output) const;
            void bodyACCOutput(const drake::systems::Context<double> &context,
                               std::vector<drake::multibody::SpatialAcceleration<double>> *output) const;

            void Update(const drake::systems::Context<T> &context,
                        drake::systems::State<T> *next_state) const;
            uint16_t nv_, nq_, na_;
            const drake::multibody::MultibodyPlant<T> &plant_;
            std::unique_ptr<drake::systems::Context<T>> plant_context_;
            const drake::systems::InputPort<T> *actuation_input_port_;
            const drake::systems::OutputPort<T> *state_output_port_;
            const drake::systems::OutputPort<T> *generalized_acceleration_output_port_;
            const drake::systems::OutputPort<T> *phase_output_port_;
            const drake::systems::OutputPort<T> *body_poses_output_port_;
            const drake::systems::OutputPort<T> *body_spatial_velocities_output_port_;
            const drake::systems::OutputPort<T> *body_spatial_accelerations_output_port_;
            const drake::systems::OutputPort<double> *vtarget_output_port_{nullptr};
            const drake::systems::OutputPort<double> *turn_rps_output_port_{nullptr};

            drake::systems::DiscreteStateIndex state_id_;
            drake::systems::DiscreteStateIndex acceleration_id_;
            drake::systems::DiscreteStateIndex body_poses_id_;
            drake::systems::DiscreteStateIndex body_v_id_;
            drake::systems::DiscreteStateIndex body_a_id_;
            mutable bool step_updated;
            mutable nlohmann::json actuationJsonArray;
            mutable Eigen::VectorXd mutable_state;
            mutable Eigen::VectorXd prev_mutable_state_v;
            mutable Eigen::VectorXd mutable_state_a,mutable_phase;
            mutable std::vector<drake::math::RigidTransform<T>> mutable_body_p;
            mutable std::vector<drake::multibody::SpatialVelocity<double>> mutable_body_v;
            mutable std::vector<drake::multibody::SpatialVelocity<double>> prev_mutable_body_v;
            mutable Eigen::VectorXd mutable_body_a;
            mutable Eigen::VectorXd mutable_commands;
            double dt;
            std::thread tcp_thread_;
            mutable bool is_initial;
            mutable Eigen::VectorXd initial_pose;
        };
        // DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        //     class ::my_drake_examples::biped::IsaacSimulator)
        extern template class ::my_drake_examples::biped::IsaacSimulator<double>;

    } // namespace biped
}
