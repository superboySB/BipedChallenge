#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class AlipPlanner final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(AlipPlanner)

      AlipPlanner(double time_step, double offset, const drake::multibody::MultibodyPlant<T> &);

      template <typename U>
      explicit AlipPlanner(const AlipPlanner<U> &);

      double time_step() const { return time_step_; }

      const drake::systems::InputPort<T> &get_stanceLeg_input_port() const
      {
        return *stanceLeg_input_port_;
      }

      const drake::systems::InputPort<T> &get_phase_input_port() const
      {
        return *stand_flag_input_port_;
      }
      const drake::systems::OutputPort<T> &get_phase_output_port() const
      {
        return *phase_output_port;
      }

      const drake::systems::InputPort<T> &get_q_input_port() const
      {
        return *q_input_port_;
      }

      const drake::systems::InputPort<T> &get_v_input_port() const
      {
        return *v_input_port_;
      }

      const drake::systems::InputPort<T> &get_vtarget_input_port() const
      {
        return *vtarget_input_port_;
      }

      const drake::systems::InputPort<T> &get_H_input_port() const
      {
        return *H_input_port_;
      }

      const drake::systems::InputPort<T> &get_stepWidth_input_port() const
      {
        return *stepWidth_input_port_;
      }

      const drake::systems::InputPort<T> &get_gait_period_input_port() const
      {
        return *gait_period_input_port_;
      }

      const drake::systems::InputPort<T> &get_turn_rps_input_port() const
      {
        return *turn_rps_input_port_;
      }

      const drake::systems::OutputPort<T> &get_h_ref_output_port() const
      {
        return *h_ref_output_port_;
      }

      const drake::systems::OutputPort<T> &get_hd_ref_output_port() const
      {
        return *hd_ref_output_port_;
      }

      const drake::systems::OutputPort<T> &get_hdd_ref_output_port() const
      {
        return *hdd_ref_output_port_;
      }

      const drake::systems::OutputPort<T> &get_stanceLeg_output_port() const
      {
        return *stanceLeg_output_port_;
      }

      const drake::systems::OutputPort<T> &get_vf_WScm_output_port() const
      {
        return *vf_WScm_output_port_;
      }

      const drake::systems::OutputPort<T> &get_vnextf_output_port() const
      {
        return *vnextf_output_port_;
      }

      const drake::systems::OutputPort<T> &get_pf_WSw_output_port() const
      {
        return *pf_WSw_output_port_;
      }

      const drake::systems::OutputPort<T> &get_calc_elapsed_output_port() const
      {
        return *calc_elapsed_output_port_;
      }

      const drake::systems::OutputPort<T> &get_begin_calc_output_port() const
      {
        return *begin_calc_output_port_;
      }

      const drake::systems::OutputPort<T> &get_end_calc_output_port() const
      {
        return *end_calc_output_port_;
      }

      void stand()
      {
        stand_phase = 0;
      }

    private:
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::DiscreteValues<T> *next_state) const;
      void phaseOutput(const drake::systems::Context<double> &context,
                       drake::systems::BasicVector<double> *output) const;
      const double time_step_{0};
      const double g{9.81};
      const double z_cl{0.15};
      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      T total_mass_{0};
      mutable int stepcount = 0;
      mutable int stand_phase = -1;
      const drake::systems::InputPort<T> *stanceLeg_input_port_{nullptr};
      const drake::systems::InputPort<T> *stand_flag_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::InputPort<T> *vtarget_input_port_{nullptr};
      const drake::systems::InputPort<T> *H_input_port_{nullptr};
      const drake::systems::InputPort<T> *stepWidth_input_port_{nullptr};
      const drake::systems::InputPort<T> *gait_period_input_port_{nullptr};
      const drake::systems::InputPort<T> *turn_rps_input_port_{nullptr};
      const drake::systems::OutputPort<T> *h_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *phase_output_port{nullptr};
      const drake::systems::OutputPort<T> *hd_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *hdd_ref_output_port_{nullptr};
      const drake::systems::OutputPort<T> *stanceLeg_output_port_{nullptr};
      const drake::systems::OutputPort<T> *vf_WScm_output_port_{nullptr};
      const drake::systems::OutputPort<T> *vnextf_output_port_{nullptr};
      const drake::systems::OutputPort<T> *pf_WSw_output_port_{nullptr};
      const drake::systems::OutputPort<T> *calc_elapsed_output_port_{nullptr};
      const drake::systems::OutputPort<T> *begin_calc_output_port_{nullptr};
      const drake::systems::OutputPort<T> *end_calc_output_port_{nullptr};
      drake::systems::DiscreteStateIndex gait_epoch_id_;
      drake::systems::DiscreteStateIndex p0_WSw_id_;
      drake::systems::DiscreteStateIndex stanceLeg_id_;
      drake::systems::DiscreteStateIndex h_ref_id_;
      drake::systems::DiscreteStateIndex hd_ref_id_;
      drake::systems::DiscreteStateIndex hdd_ref_id_;
      drake::systems::DiscreteStateIndex vf_WScm_id_;
      drake::systems::DiscreteStateIndex vnextf_id_;
      drake::systems::DiscreteStateIndex pf_WSw_id_;
      drake::systems::DiscreteStateIndex calc_elapsed_id_;
      drake::systems::DiscreteStateIndex begin_calc_id_;
      drake::systems::DiscreteStateIndex end_calc_id_;
      mutable Eigen::VectorXd stand_pose;
      mutable double split_time{0.0};
      mutable int planner_phase{0};
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::AlipPlanner)

  } // namespace biped
} // namespace my_drake_examples
