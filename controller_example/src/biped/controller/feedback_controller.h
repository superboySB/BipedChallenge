#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/systems/framework/context.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class FeedbackController final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FeedbackController)

      FeedbackController(double time_step, double offset, const drake::multibody::MultibodyPlant<T> &);

      // template <typename U>
      // explicit FeedbackController(const FeedbackController<U>&);

      double time_step() const { return time_step_; }

      const drake::systems::InputPort<T> &get_h_ref_input_port() const
      {
        return *h_ref_input_port_;
      }

      const drake::systems::InputPort<T> &get_hd_ref_input_port() const
      {
        return *hd_ref_input_port_;
      }

      const drake::systems::InputPort<T> &get_hdd_ref_input_port() const
      {
        return *hdd_ref_input_port_;
      }

      const drake::systems::InputPort<T> &get_stanceLeg_input_port() const
      {
        return *stanceLeg_input_port_;
      }

      const drake::systems::InputPort<T> &get_q_input_port() const
      {
        return *q_input_port_;
      }

      const drake::systems::InputPort<T> &get_v_input_port() const
      {
        return *v_input_port_;
      }

      const drake::systems::OutputPort<T> &get_torque_output_port() const
      {
        return *torque_output_port_;
      }

      const drake::systems::OutputPort<T> &get_G_output_port() const
      {
        return *G_output_port_;
      }

      const drake::systems::OutputPort<T> &get_Cv_output_port() const
      {
        return *Cv_output_port_;
      }

      const drake::systems::OutputPort<T> &get_H1_output_port() const
      {
        return *H1_output_port_;
      }

      const drake::systems::OutputPort<T> &get_H2_output_port() const
      {
        return *H2_output_port_;
      }

      const drake::systems::OutputPort<T> &get_lambda_output_port() const
      {
        return *lambda_output_port_;
      }

      const drake::systems::OutputPort<T> &get_h_est_output_port() const
      {
        return *h_est_output_port_;
      }

      const drake::systems::OutputPort<T> &get_hd_est_output_port() const
      {
        return *hd_est_output_port_;
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
      const drake::systems::InputPort<T> &get_phase_input_port() const
      {
        return *phase_input_port_;
      }

    private:
      void Update(const drake::systems::Context<T> &context,
                  drake::systems::DiscreteValues<T> *next_state) const;

      const double time_step_{0};
      int nh_{16};
      int nq_;
      int nv_;
      int na_;

      const drake::multibody::MultibodyPlant<T> &plant_;
      std::unique_ptr<drake::systems::Context<T>> plant_context_;
      const drake::systems::InputPort<T> *h_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hdd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *stanceLeg_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::InputPort<T> *phase_input_port_{nullptr};
      const drake::systems::OutputPort<T> *torque_output_port_{nullptr};
      const drake::systems::OutputPort<T> *G_output_port_{nullptr};
      const drake::systems::OutputPort<T> *Cv_output_port_{nullptr};
      const drake::systems::OutputPort<T> *H1_output_port_{nullptr};
      const drake::systems::OutputPort<T> *H2_output_port_{nullptr};
      const drake::systems::OutputPort<T> *lambda_output_port_{nullptr};
      const drake::systems::OutputPort<T> *h_est_output_port_{nullptr};
      const drake::systems::OutputPort<T> *hd_est_output_port_{nullptr};
      const drake::systems::OutputPort<T> *calc_elapsed_output_port_{nullptr};
      const drake::systems::OutputPort<T> *begin_calc_output_port_{nullptr};
      const drake::systems::OutputPort<T> *end_calc_output_port_{nullptr};
      drake::systems::DiscreteStateIndex torque_id_;
      drake::systems::DiscreteStateIndex G_id_;
      drake::systems::DiscreteStateIndex Cv_id_;
      drake::systems::DiscreteStateIndex H1_id_;
      drake::systems::DiscreteStateIndex H2_id_;
      drake::systems::DiscreteStateIndex lambda_id_;
      drake::systems::DiscreteStateIndex h_est_id_;
      drake::systems::DiscreteStateIndex hd_est_id_;
      drake::systems::DiscreteStateIndex calc_elapsed_id_;
      drake::systems::DiscreteStateIndex begin_calc_id_;
      drake::systems::DiscreteStateIndex end_calc_id_;
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::FeedbackController)

  } // namespace biped
} // namespace my_drake_examples
