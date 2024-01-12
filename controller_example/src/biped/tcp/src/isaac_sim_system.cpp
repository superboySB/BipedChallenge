#include "isaac_sim_system.hpp"

#include <iostream>
#include <drake/systems/framework/output_port.h>
#include <drake/multibody/math/spatial_algebra.h>
#include <drake/math/gradient.h>

namespace my_drake_examples
{
    namespace biped
    {

        template <typename T>
        IsaacSimulator<T>::IsaacSimulator(double time_step, double offset,
                                          const drake::multibody::MultibodyPlant<T> &plant)
            : plant_(plant), TcpServer("0.0.0.0", 8800), dt(time_step)
        {
            nv_ = plant_.num_velocities();
            nq_ = plant_.num_positions();
            na_ = plant_.num_actuated_dofs();
            std::cout << " nq_:" << nq_ << " nv_" << nv_ << " na_" << na_ << std::endl;
            actuation_input_port_ = &this->DeclareVectorInputPort("actuation_input", na_);

            state_id_ = this->DeclareDiscreteState(nq_ + nv_);
            acceleration_id_ = this->DeclareDiscreteState(nv_);
            body_poses_id_ = this->DeclareDiscreteState(7); // 所有body的空间位置
            body_v_id_ = this->DeclareDiscreteState(6);     // 所有body的空间速度
            body_a_id_ = this->DeclareDiscreteState(6);     // 所有body的空间加速度

            mutable_state.resize(nq_ + nv_);
            mutable_state_a.resize(nv_);
            prev_mutable_state_v.resize(nv_);
            mutable_phase = Eigen::VectorXd::Zero(1);
            // mutable_body_p.resize(7);
            mutable_body_v.resize(plant_.num_bodies());
            prev_mutable_body_v.resize(plant_.num_bodies());
            mutable_body_a.resize(6);
            mutable_commands.resize(3);
            initial_pose.resize(7);
            mutable_commands << 0, 0, 0;

            this->DeclarePeriodicUnrestrictedUpdateEvent(
                time_step, offset, &IsaacSimulator::Update);
            state_output_port_ = &this->DeclareVectorOutputPort("state", nq_ + nv_, &IsaacSimulator::stateOutput);
            generalized_acceleration_output_port_ = &this->DeclareVectorOutputPort("generalized_acceleration", nv_, &IsaacSimulator::accOutput);
            phase_output_port_ = &this->DeclareVectorOutputPort("phase", 1, &IsaacSimulator::phaseOutput);
            body_poses_output_port_ = &this->DeclareAbstractOutputPort("body_poses", &IsaacSimulator::bodyPoseOutput);
            body_spatial_velocities_output_port_ = &this->DeclareAbstractOutputPort("body_spatial_velocities", &IsaacSimulator::bodyVELOutput);
            body_spatial_accelerations_output_port_ = &this->DeclareAbstractOutputPort("body_spatial_accelerations", &IsaacSimulator::bodyACCOutput);

            vtarget_output_port_ = &this->DeclareVectorOutputPort(
                "vtarget", 2, &IsaacSimulator::CalcVtarget);

            turn_rps_output_port_ = &this->DeclareVectorOutputPort(
                "turn_rps", 1, &IsaacSimulator::CalcTurnRps);

            plant_context_ = plant_.CreateDefaultContext();
            double total_mass_ = plant_.CalcTotalMass(*plant_context_);
            std::cout << "total_mass_:" << total_mass_ << std::endl;
            step_updated = true;
            is_initial = true;

            actuationJsonArray = nlohmann::json::array();
            for (Eigen::Index i = 0; i < 10; ++i)
            {
                actuationJsonArray.push_back(0);
            }

            tcp_thread_ = std::thread(&IsaacSimulator::launch, this);
            if (!tcp_thread_.joinable())
            {
                std::cout << "tcp_thread_ start error!" << std::endl;
                exit(1);
            }
            else
            {
                std::cout << "tcp_thread_ started!" << std::endl;
            }
        }
        template <typename T>
        void IsaacSimulator<T>::CalcTurnRps(const drake::systems::Context<double> &context,
                                            drake::systems::BasicVector<double> *output) const
        {
            auto output_vector = output->get_mutable_value();

            output_vector(0) = M_PI_4 * mutable_commands[2];
        }
        template <typename T>
        void IsaacSimulator<T>::CalcVtarget(const drake::systems::Context<double> &context,
                                            drake::systems::BasicVector<double> *output) const
        {
            auto output_vector = output->get_mutable_value();

            output_vector(1) = mutable_commands[1];
            output_vector(0) = mutable_commands[0];
        }

        template <typename T>
        void IsaacSimulator<T>::Update(const drake::systems::Context<T> &context,
                                       drake::systems::State<T> *next_state) const
        {
            while (step_updated)
            {
                usleep(200); // wait for update
            }
            const auto &actuation = get_actuation_input_port().Eval(context);
            actuationJsonArray = nlohmann::json::array();
            for (Eigen::Index i = 0; i < 10; ++i)
            {
                actuationJsonArray.push_back(actuation(i));
            }
            auto state = mutable_state;
            plant_.SetPositionsAndVelocities(plant_context_.get(), state);

            // auto state = next_state->get_mutable_discrete_state(state_id_).get_mutable_value();
            // auto acceleration = next_state->get_mutable_discrete_state(acceleration_id_).get_mutable_value();
            // auto body_poses = next_state->get_mutable_discrete_state(body_poses_id_).get_mutable_value();
            // auto body_v = next_state->get_mutable_discrete_state(body_v_id_).get_mutable_value();

            // auto body_a = next_state->get_mutable_discrete_state(body_a_id_).get_mutable_value();

            // state = mutable_state;
            // acceleration = mutable_state_a;
            // body_poses = mutable_body_p;
            // body_v = mutable_body_v;
            // body_a = mutable_body_a;
            step_updated = true;
        }
        template <typename T>
        void IsaacSimulator<T>::bodyPoseOutput(const drake::systems::Context<double> &context,
                                               std::vector<drake::math::RigidTransform<T>> *output) const
        {
            *output = plant_.get_body_poses_output_port().template Eval<std::vector<drake::math::RigidTransform<double>>>(*plant_context_);
        }
        template <typename T>
        void IsaacSimulator<T>::bodyVELOutput(const drake::systems::Context<double> &context,
                                              std::vector<drake::multibody::SpatialVelocity<double>> *output) const
        {
            *output = plant_.get_body_spatial_velocities_output_port().template Eval<std::vector<drake::multibody::SpatialVelocity<double>>>(*plant_context_);
        }
        template <typename T>
        void IsaacSimulator<T>::bodyACCOutput(const drake::systems::Context<double> &context,
                                              std::vector<drake::multibody::SpatialAcceleration<double>> *output) const
        {
            std::vector<drake::multibody::SpatialAcceleration<double>> accelerations;
            accelerations.reserve(plant_.num_bodies());
            // auto &out = plant_.get_body_spatial_accelerations_output_port().template Eval<std::vector<drake::multibody::SpatialAcceleration<double>>>(*plant_context_);
            std::vector<drake::multibody::SpatialVelocity<double>> current_velocities = plant_.get_body_spatial_velocities_output_port().template Eval<std::vector<drake::multibody::SpatialVelocity<double>>>(*plant_context_);
            for (size_t i = 0; i < prev_mutable_body_v.size(); ++i)
            {
                const drake::multibody::SpatialVelocity<double> &previous_velocity = prev_mutable_body_v[i];
                const drake::multibody::SpatialVelocity<double> &current_velocity = current_velocities[i];

                const drake::multibody::SpatialAcceleration<double> acceleration = {
                    (current_velocity.translational() - previous_velocity.translational()) / dt,
                    (current_velocity.rotational() - previous_velocity.rotational()) / dt};
                // std::cout << "acceleration:"<<acceleration.translational()<<std::endl;
                accelerations.push_back(acceleration);
            }
            prev_mutable_body_v = current_velocities;
            *output = accelerations;
        }
        template <typename T>
        void IsaacSimulator<T>::stateOutput(const drake::systems::Context<double> &context,
                                            drake::systems::BasicVector<double> *output) const
        {
            auto output_vector = output->get_mutable_value();
            // // output_vector = mutable_state;
            output_vector = plant_.GetPositionsAndVelocities(*plant_context_);
        }
        template <typename T>
        void IsaacSimulator<T>::accOutput(const drake::systems::Context<double> &context,
                                          drake::systems::BasicVector<double> *output) const
        {

            auto output_vector = output->get_mutable_value();
            output_vector = mutable_state_a;
        }
         template <typename T>
        void IsaacSimulator<T>::phaseOutput(const drake::systems::Context<double> &context,
                                          drake::systems::BasicVector<double> *output) const
        {

            auto output_vector = output->get_mutable_value();
            output_vector = mutable_phase;
        }
        template <typename T>
        void IsaacSimulator<T>::get_initial_pose(Eigen::VectorXd &pose)
        {
            while (is_initial)
            {
                usleep(1000);
            }
            std::cout << "initial_pose :" << initial_pose.transpose() << std::endl;
            pose << initial_pose;
        }

        template <typename T>
        std::string IsaacSimulator<T>::processMsg_(const char *msg)
        {
            // std::cout << "processMsg_\n";
            while (!step_updated)
            {
                usleep(200); // wait for update
            }
            // parse input message to JSON format
            nlohmann::json jmsg = bin2json(msg);
            if (jmsg.contains("command"))
            {
                std::vector<double> command_vec = jmsg["command"].get<std::vector<double>>();
                mutable_commands << Eigen::VectorXd::Map(command_vec.data(), command_vec.size());
                // std::cout << "mutable_commands:"<<mutable_commands.transpose()<<std::endl;
            }
            if (jmsg.contains("change_state"))
            {
                int command_vec = jmsg["change_state"].get<int>();
                mutable_phase << command_vec;
            }else{
                mutable_phase << 0;
            }
            std::vector<double> q_leg = jmsg["q_leg"].get<std::vector<double>>();
            std::vector<double> q_arm = jmsg["q_arm"].get<std::vector<double>>();
            std::vector<double> v_leg = jmsg["dq_leg"].get<std::vector<double>>();
            std::vector<double> v_arm = jmsg["dq_arm"].get<std::vector<double>>();
            std::vector<double> p_wb = jmsg["p_wb"].get<std::vector<double>>();
            std::vector<double> quat_wb = jmsg["quat_wb"].get<std::vector<double>>();
            std::vector<double> v_wb = jmsg["v_wb"].get<std::vector<double>>();
            std::vector<double> w_wb = jmsg["w_wb"].get<std::vector<double>>();
            Eigen::VectorXd q_leg_eigen = Eigen::VectorXd::Map(q_leg.data(), q_leg.size());
            Eigen::VectorXd v_leg_eigen = Eigen::VectorXd::Map(v_leg.data(), v_leg.size());
            Eigen::VectorXd q(nq_), v(nv_);
            if (nq_ + nv_ > quat_wb.size() + p_wb.size() + q_leg_eigen.size() + w_wb.size() + v_wb.size() + v_leg_eigen.size())
            {
                q << Eigen::VectorXd::Map(quat_wb.data(), quat_wb.size()), Eigen::VectorXd::Map(p_wb.data(), p_wb.size()), q_leg_eigen, Eigen::VectorXd::Map(q_arm.data(), q_arm.size());
                v << Eigen::VectorXd::Map(w_wb.data(), w_wb.size()), Eigen::VectorXd::Map(v_wb.data(), v_wb.size()), v_leg_eigen, Eigen::VectorXd::Map(v_arm.data(), v_arm.size());
            }
            else
            {
                q << Eigen::VectorXd::Map(quat_wb.data(), quat_wb.size()), Eigen::VectorXd::Map(p_wb.data(), p_wb.size()), q_leg_eigen;
                v << Eigen::VectorXd::Map(w_wb.data(), w_wb.size()), Eigen::VectorXd::Map(v_wb.data(), v_wb.size()), v_leg_eigen;
            }
            mutable_state << q, v;
            mutable_state_a = (v - prev_mutable_state_v) / dt;
            prev_mutable_state_v = v;
            // std::cout <<"q:"<<q.transpose()<<std::endl;
            // std::cout <<"v:"<<v.transpose()<<std::endl;
            // std::cout <<"tau:"<<actuationJsonArray<<std::endl;

            // mutable_body_a = (mutable_body_v - prev_mutable_body_v) / dt;
            // prev_mutable_body_v = mutable_body_v;
            // print message, i.e., controller input
            // std::cout << jmsg.dump(4) << std::endl;

            nlohmann::json resp;
            // resp["content"] = "Hello from TCP Server";
            resp["mode"] = "effort";
            resp["effort"] = actuationJsonArray;

            if (is_initial)
            {
                initial_pose << Eigen::VectorXd::Map(quat_wb.data(), quat_wb.size()), Eigen::VectorXd::Map(p_wb.data(), p_wb.size());
                is_initial = false;
            }
            // else
            {
                step_updated = false;
            }

            // return controller output
            // the output will be automatically send back to the client
            return json2binstr(resp);
        }
        template class ::my_drake_examples::biped::IsaacSimulator<double>;

    } // namespace biped
} // namespace my_drake_examples
