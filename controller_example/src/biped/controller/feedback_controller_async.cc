#include "feedback_controller_async.h"

#include <iostream>
#include <future>

#include "feedback_controller.h"
#include <drake/systems/framework/discrete_values.h>

namespace my_drake_examples
{
  namespace biped
  {

    namespace
    {

      template <typename T>
      struct StampOutput
      {
        T time{std::numeric_limits<double>::quiet_NaN()};
        std::shared_ptr<const drake::VectorX<T>> torque;
        std::shared_ptr<const drake::VectorX<T>> begin_calc;
        std::shared_ptr<const drake::VectorX<T>> end_calc;
      };

      template <typename T>
      class Worker
      {
      public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(Worker)

        Worker(std::shared_ptr<const FeedbackController<T>> sys) : sys_{std::move(sys)}
        {
          DRAKE_DEMAND(sys_ != nullptr);
          sys_context_ = sys_->CreateDefaultContext();
          discrete_updates_ = sys_->AllocateDiscreteVariables();
          merged_events_ = sys_->AllocateCompositeEventCollection();
          sys_->CalcNextUpdateTime(*sys_context_, merged_events_.get());
        }

        void Start(T context_time,
                   const drake::VectorX<T> &h_ref,
                   const drake::VectorX<T> &hd_ref,
                   const drake::VectorX<T> &hdd_ref,
                   const drake::VectorX<T> &stanceLeg,
                   const drake::VectorX<T> &q,
                   const drake::VectorX<T> &v);

        StampOutput<T> Finish();

      private:
        const std::shared_ptr<const FeedbackController<T>> sys_;
        std::unique_ptr<drake::systems::Context<T>> sys_context_;
        std::unique_ptr<drake::systems::DiscreteValues<T>> discrete_updates_;
        std::unique_ptr<drake::systems::CompositeEventCollection<T>> merged_events_;
        std::future<StampOutput<T>> future_;
      };

      template <typename T>
      void Worker<T>::Start(T context_time,
                            const drake::VectorX<T> &h_ref,
                            const drake::VectorX<T> &hd_ref,
                            const drake::VectorX<T> &hdd_ref,
                            const drake::VectorX<T> &stanceLeg,
                            const drake::VectorX<T> &q,
                            const drake::VectorX<T> &v)
      {
        // std::cout << "begin start\n";
        if (future_.valid())
        {
          future_.wait();
          future_ = {};
        }
        auto task = [this, context_time, h_ref, hd_ref, hdd_ref, stanceLeg, q, v]() -> StampOutput<T>
        {
          // std::cout << "begin task\n";
          const auto &h_ref_port = sys_->get_h_ref_input_port();
          const auto &hd_ref_port = sys_->get_hd_ref_input_port();
          const auto &hdd_ref_port = sys_->get_hdd_ref_input_port();
          const auto &stanceLeg_port = sys_->get_stanceLeg_input_port();
          const auto &q_port = sys_->get_q_input_port();
          const auto &v_port = sys_->get_v_input_port();
          h_ref_port.FixValue(sys_context_.get(), h_ref);
          hd_ref_port.FixValue(sys_context_.get(), hd_ref);
          hdd_ref_port.FixValue(sys_context_.get(), hdd_ref);
          stanceLeg_port.FixValue(sys_context_.get(), stanceLeg);
          q_port.FixValue(sys_context_.get(), q);
          v_port.FixValue(sys_context_.get(), v);
          // 散装simulator
          sys_->CalcDiscreteVariableUpdate(*sys_context_,
                                           merged_events_->get_discrete_update_events(),
                                           discrete_updates_.get());
          sys_->ApplyDiscreteVariableUpdate(merged_events_->get_discrete_update_events(),
                                            discrete_updates_.get(),
                                            sys_context_.get());
          sys_context_->SetTime(context_time);

          StampOutput<T> result;
          result.torque = std::make_shared<const drake::VectorX<T>>(
              sys_->get_torque_output_port().Eval(*sys_context_));
          result.begin_calc = std::make_shared<const drake::VectorX<T>>(
              sys_->get_begin_calc_output_port().Eval(*sys_context_));
          result.end_calc = std::make_shared<const drake::VectorX<T>>(
              sys_->get_end_calc_output_port().Eval(*sys_context_));
          result.time = context_time;
          // std::cout << "end task\n";
          return result;
        };
        future_ = std::async(std::launch::async, std::move(task));
        // std::cout << "end start\n";
      }

      template <typename T>
      StampOutput<T> Worker<T>::Finish()
      {
        // std::cout << "begin finish\n";
        if (!future_.valid())
        {
          return {};
        }
        future_.wait();
        // std::cout << "end finish\n";
        return future_.get();
      }

      template <typename T>
      struct TickTockState
      {
        std::shared_ptr<Worker<T>> worker;
        StampOutput<T> output;
      };

    } // namespace

    template <typename T>
    FeedbackControllerAsync<T>::FeedbackControllerAsync(
        double time_step, double offset, double delay,
        const drake::multibody::MultibodyPlant<T> &plant)
        : time_step_(time_step), offset_(offset), delay_(delay), plant_(plant)
    {
      h_ref_input_port_ = &this->DeclareVectorInputPort("h_ref", 10);
      hd_ref_input_port_ = &this->DeclareVectorInputPort("hd_ref", 10);
      hdd_ref_input_port_ = &this->DeclareVectorInputPort("hdd_ref", 10);
      stanceLeg_input_port_ = &this->DeclareVectorInputPort("stanceLeg", 1);
      q_input_port_ = &this->DeclareVectorInputPort("EstStates.q", 17);
      v_input_port_ = &this->DeclareVectorInputPort("EstStates.v", 16);
      auto state_index = this->DeclareAbstractState(drake::Value<TickTockState<T>>{});
      this->DeclareInitializationUnrestrictedUpdateEvent(&FeedbackControllerAsync::Initialize);
      this->DeclarePeriodicUnrestrictedUpdateEvent(time_step, offset, &FeedbackControllerAsync::CalcTick);
      this->DeclarePeriodicUnrestrictedUpdateEvent(time_step, offset + delay, &FeedbackControllerAsync::CalcTock);
      const std::set<drake::systems::DependencyTicket> state = {this->abstract_state_ticket(state_index)};
      torque_output_port_ = &this->DeclareVectorOutputPort("torque", 10, &FeedbackControllerAsync::CalcTorque, state);
      begin_calc_output_port_ = &this->DeclareVectorOutputPort("begin_calc", 1, &FeedbackControllerAsync::CalcBeginCalc, state);
      end_calc_output_port_ = &this->DeclareVectorOutputPort("end_calc", 1, &FeedbackControllerAsync::CalcEndCalc, state);
    }

    template <typename T>
    drake::systems::EventStatus FeedbackControllerAsync<T>::Initialize(const drake::systems::Context<T> &context,
                                                                       drake::systems::State<T> *state) const
    {
      drake::unused(context);
      TickTockState<T> &next_state = state->template get_mutable_abstract_state<TickTockState<T>>(0);
      auto sys = std::make_shared<const FeedbackController<T>>(time_step_, offset_, plant_);
      next_state.worker = std::make_shared<Worker<T>>(std::move(sys));
      next_state.output = {};
      return drake::systems::EventStatus::Succeeded();
    }

    template <typename T>
    void FeedbackControllerAsync<T>::CalcTick(const drake::systems::Context<T> &context,
                                              drake::systems::State<T> *state) const
    {
      const auto &h_ref = get_h_ref_input_port().Eval(context);
      const auto &hd_ref = get_hd_ref_input_port().Eval(context);
      const auto &hdd_ref = get_hdd_ref_input_port().Eval(context);
      const auto &stanceLeg = get_stanceLeg_input_port().Eval(context);
      const auto &q = get_q_input_port().Eval(context);
      const auto &v = get_v_input_port().Eval(context);
      const TickTockState<T> &prior_state = context.template get_abstract_state<TickTockState<T>>(0);
      DRAKE_DEMAND(state != nullptr);
      TickTockState<T> &next_state = state->template get_mutable_abstract_state<TickTockState<T>>(0);
      next_state.output = prior_state.output;
      if (prior_state.worker == nullptr)
      {
        Initialize(context, state);
      }
      else
      {
        next_state.worker = prior_state.worker;
      }
      next_state.worker->Start(context.get_time(), h_ref, hd_ref, hdd_ref, stanceLeg, q, v);
    }

    template <typename T>
    void FeedbackControllerAsync<T>::CalcTock(const drake::systems::Context<T> &context,
                                              drake::systems::State<T> *state) const
    {
      const TickTockState<T> &prior_state = context.template get_abstract_state<TickTockState<T>>(0);
      DRAKE_DEMAND(state != nullptr);
      TickTockState<T> &next_state = state->template get_mutable_abstract_state<TickTockState<T>>(0);
      if (prior_state.worker == nullptr)
      {
        next_state = {};
        return;
      }
      next_state.worker = prior_state.worker;
      next_state.output = next_state.worker->Finish();
    }

    template <typename T>
    void FeedbackControllerAsync<T>::CalcTorque(const drake::systems::Context<T> &context,
                                                drake::systems::BasicVector<T> *output) const
    {
      const TickTockState<T> &state = context.template get_abstract_state<TickTockState<T>>(0);
      auto vec_ptr = state.output.torque.get();
      auto output_vec = output->get_mutable_value();
      if (vec_ptr == nullptr)
      {
        output->SetFromVector(drake::VectorX<T>::Zero(10));
        return;
      }
      output->SetFromVector(*vec_ptr);
    }

    template <typename T>
    void FeedbackControllerAsync<T>::CalcBeginCalc(const drake::systems::Context<T> &context,
                                                   drake::systems::BasicVector<T> *output) const
    {
      const TickTockState<T> &state = context.template get_abstract_state<TickTockState<T>>(0);
      auto vec_ptr = state.output.begin_calc.get();
      auto output_vec = output->get_mutable_value();
      if (vec_ptr == nullptr)
      {
        output->SetFromVector(drake::VectorX<T>::Zero(1));
        return;
      }
      output->SetFromVector(*vec_ptr);
    }

    template <typename T>
    void FeedbackControllerAsync<T>::CalcEndCalc(const drake::systems::Context<T> &context,
                                                 drake::systems::BasicVector<T> *output) const
    {
      const TickTockState<T> &state = context.template get_abstract_state<TickTockState<T>>(0);
      auto vec_ptr = state.output.end_calc.get();
      auto output_vec = output->get_mutable_value();
      if (vec_ptr == nullptr)
      {
        output->SetFromVector(drake::VectorX<T>::Zero(1));
        return;
      }
      output->SetFromVector(*vec_ptr);
    }

    template <typename T>
    void FeedbackControllerAsync<T>::CalcTime(const drake::systems::Context<T> &context,
                                              drake::systems::BasicVector<T> *output) const
    {
      const TickTockState<T> &state = context.template get_abstract_state<TickTockState<T>>(0);
      output->SetFromVector(drake::Vector1<T>{state.output.time});
    }
    template class ::my_drake_examples::biped::FeedbackControllerAsync<double>;

    // DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    //     class ::my_drake_examples::biped::FeedbackControllerAsync)

  } // namespace biped
} // namespace my_drake_examples
