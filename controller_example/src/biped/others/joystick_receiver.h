#pragma once

#include <memory>
#include <string>
#include <map>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <drake/systems/framework/leaf_system.h>

namespace my_drake_examples
{
  namespace biped
  {

    class JoystickReceiver final : public drake::systems::LeafSystem<double>
    {
    public:
      DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(JoystickReceiver)

      JoystickReceiver(double time_step);

      ~JoystickReceiver();

      double time_step() const { return time_step_; }

      const drake::systems::OutputPort<double> &get_axes_output_port() const
      {
        return *axes_output_port_;
      }

      const drake::systems::OutputPort<double> &get_buttons_output_port() const
      {
        return *buttons_output_port_;
      }

      const drake::systems::OutputPort<double> &get_hats_output_port() const
      {
        return *hats_output_port_;
      }

    private:
      class TextPrint;

      drake::systems::EventStatus Initialize(
          const drake::systems::Context<double> &) const;

      drake::systems::EventStatus Update(const drake::systems::Context<double> &context,
                                         drake::systems::State<double> *next_state) const;

      double time_step_{0};
      const drake::systems::OutputPort<double> *axes_output_port_{nullptr};
      const drake::systems::OutputPort<double> *buttons_output_port_{nullptr};
      const drake::systems::OutputPort<double> *hats_output_port_{nullptr};
      int axes_id_{0};
      int buttons_id_{0};
      int hats_id_{0};

      SDL_Window *win_{nullptr};
      SDL_Surface *screen_{nullptr};
      TextPrint *text_print_{nullptr};
    };

  } // namespace biped
} // namespace my_drake_examples
