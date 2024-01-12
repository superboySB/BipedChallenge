#pragma once

#include <drake/systems/framework/leaf_system.h>
#include <drake/multibody/plant/multibody_plant.h>

namespace my_drake_examples
{
  namespace biped
  {

    template <typename T>
    class FeedbackControllerAsync final : public drake::systems::LeafSystem<T>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(FeedbackControllerAsync)

      FeedbackControllerAsync(double time_step, double offset, double delay,
                              const drake::multibody::MultibodyPlant<T> &);

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

      // const drake::systems::OutputPort<T>& get_G_output_port() const {
      //   return *G_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_Cv_output_port() const {
      //   return *Cv_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_H1_output_port() const {
      //   return *H1_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_H2_output_port() const {
      //   return *H2_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_lambda_output_port() const {
      //   return *lambda_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_h_est_output_port() const {
      //   return *h_est_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_hd_est_output_port() const {
      //   return *hd_est_output_port_;
      // }

      // const drake::systems::OutputPort<T>& get_calc_elapsed_output_port() const {
      //   return *calc_elapsed_output_port_;
      // }

      const drake::systems::OutputPort<T> &get_begin_calc_output_port() const
      {
        return *begin_calc_output_port_;
      }

      const drake::systems::OutputPort<T> &get_end_calc_output_port() const
      {
        return *end_calc_output_port_;
      }

    private:
      // struct TickTockState;

      drake::systems::EventStatus Initialize(const drake::systems::Context<T> &,
                                             drake::systems::State<T> *) const;
      void CalcTick(const drake::systems::Context<T> &, drake::systems::State<T> *) const;
      void CalcTock(const drake::systems::Context<T> &, drake::systems::State<T> *) const;
      void CalcTorque(const drake::systems::Context<T> &, drake::systems::BasicVector<T> *) const;
      void CalcBeginCalc(const drake::systems::Context<T> &, drake::systems::BasicVector<T> *) const;
      void CalcEndCalc(const drake::systems::Context<T> &, drake::systems::BasicVector<T> *) const;
      void CalcTime(const drake::systems::Context<T> &, drake::systems::BasicVector<T> *) const;
      const drake::multibody::MultibodyPlant<T> &plant_;
      const drake::systems::InputPort<T> *h_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *hdd_ref_input_port_{nullptr};
      const drake::systems::InputPort<T> *stanceLeg_input_port_{nullptr};
      const drake::systems::InputPort<T> *q_input_port_{nullptr};
      const drake::systems::InputPort<T> *v_input_port_{nullptr};
      const drake::systems::OutputPort<T> *torque_output_port_{nullptr};
      const drake::systems::OutputPort<T> *begin_calc_output_port_{nullptr};
      const drake::systems::OutputPort<T> *end_calc_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* G_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* Cv_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* H1_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* H2_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* lambda_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* h_est_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* hd_est_output_port_{nullptr};
      // const drake::systems::OutputPort<T>* calc_elapsed_output_port_{nullptr};
      const double time_step_;
      const double offset_;
      const double delay_;
    };

    DRAKE_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::my_drake_examples::biped::FeedbackControllerAsync)

  } // namespace biped
} // namespace my_drake_examples
