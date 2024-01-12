#include "joystick_receiver.h"

#include <iostream>

#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>

namespace my_drake_examples
{
  namespace biped
  {

    class JoystickReceiver::TextPrint
    {
    public:
      TextPrint()
      {
        reset();
        font = TTF_OpenFont("src/biped/sans.ttf", 15);
        if (font == NULL)
        {
          throw std::runtime_error("font open failed");
        }
      }

      ~TextPrint()
      {
        TTF_CloseFont(font);
      }

      void tprint(SDL_Surface *screen, std::string text)
      {
        SDL_Color textColor = {0, 0, 0, 255};
        SDL_Surface *text_bitmap =
            TTF_RenderText_Solid(font, text.c_str(), textColor);
        SDL_Rect offset{x_, y_, 0, 0};
        SDL_BlitSurface(text_bitmap, NULL, screen, &offset);
        SDL_FreeSurface(text_bitmap);
        y_ += line_height_;
      }

      void reset()
      {
        x_ = 10;
        y_ = 10;
        line_height_ = 15;
      }

      void indent() { x_ += 10; }

      void unindent() { x_ -= 10; }

    private:
      int x_{10};
      int y_{10};
      int line_height_{15};
      TTF_Font *font{nullptr};
    };

    JoystickReceiver::JoystickReceiver(double time_step) : time_step_(time_step)
    {

      this->DeclareAbstractState(drake::Value<std::map<int, SDL_Joystick *>>());

      const auto &axes = this->DeclareDiscreteState(6);
      const auto &buttons = this->DeclareDiscreteState(11);
      const auto &hats = this->DeclareDiscreteState(1);
      axes_id_ = axes;
      buttons_id_ = buttons;
      hats_id_ = hats;

      axes_output_port_ = &this->DeclareStateOutputPort("axes", axes);
      buttons_output_port_ = &this->DeclareStateOutputPort("buttons", buttons);
      hats_output_port_ = &this->DeclareStateOutputPort("hats", hats);

      this->DeclareInitializationPublishEvent(&JoystickReceiver::Initialize);
      this->DeclarePeriodicUnrestrictedUpdateEvent(
          time_step, 0.0, &JoystickReceiver::Update);

      SDL_Init(SDL_INIT_VIDEO | SDL_INIT_JOYSTICK);
      TTF_Init();
      win_ = SDL_CreateWindow("Joystick example",
                              SDL_WINDOWPOS_UNDEFINED,
                              SDL_WINDOWPOS_UNDEFINED,
                              500, 700,
                              SDL_WINDOW_HIDDEN);
      screen_ = SDL_GetWindowSurface(win_);
      text_print_ = new TextPrint();
    }

    JoystickReceiver::~JoystickReceiver()
    {
      delete text_print_;
      SDL_FreeSurface(screen_);
      SDL_DestroyWindow(win_);
      SDL_Quit();
    }

    drake::systems::EventStatus JoystickReceiver::Initialize(
        const drake::systems::Context<double> &context) const
    {
      SDL_Rect rect = {0, 0, 500, 700};
      SDL_FillRect(screen_, &rect, 0xFFFFFFFF);
      SDL_UpdateWindowSurface(win_);
      return drake::systems::EventStatus::Succeeded();
    }

    drake::systems::EventStatus JoystickReceiver::Update(const drake::systems::Context<double> &context,
                                                         drake::systems::State<double> *next_state) const
    {
      auto axes = next_state->get_mutable_discrete_state(axes_id_)
                      .get_mutable_value();
      auto buttons = next_state->get_mutable_discrete_state(buttons_id_)
                         .get_mutable_value();
      auto hats = next_state->get_mutable_discrete_state(hats_id_)
                      .get_mutable_value();
      auto &next_joysticks = next_state->template get_mutable_abstract_state<std::map<int, SDL_Joystick *>>(0);
      auto joysticks = context.get_abstract_state<std::map<int, SDL_Joystick *>>(0);

      SDL_Event event;
      while (SDL_PollEvent(&event))
      {
        if (event.type == SDL_QUIT)
        {
          std::cout << "quit" << std::endl;
          throw std::runtime_error("SDL_QUIT event");                                           // 需要找到更优雅的方法处理这个问题
          return drake::systems::EventStatus::ReachedTermination(this, "SDL_QUIT(叉掉了窗口)"); // 没用
        }
        if (event.type == SDL_JOYDEVICEADDED)
        {
          SDL_Joystick *joy = SDL_JoystickOpen(event.jdevice.which);
          if (joy == NULL)
            throw std::runtime_error("joystick open failed");
          next_joysticks.insert(std::pair<int, SDL_Joystick *>(SDL_JoystickInstanceID(joy), joy));
          std::cout << "Joystick " << SDL_JoystickInstanceID(joy) << " connected\n";
        }
        if (event.type == SDL_JOYDEVICEREMOVED)
        {
          SDL_JoystickClose(joysticks[event.jdevice.which]);
          next_joysticks.erase(event.jdevice.which);
          std::cout << "Joystick " << event.jdevice.which << " disconnected\n";
        }
        if (event.type == SDL_JOYBUTTONDOWN)
        {
          std::cout << "Joystick button pressed.\n";
          if (event.jbutton.button == 0)
          {
            SDL_Joystick *joystick = joysticks[event.jbutton.which];
            SDL_JoystickRumble(joystick, 0 * 0xFFFF, 0.7 * 0xFFFF, 500);
            std::cout << "Rumble effect played on joystick "
                      << event.jbutton.which << std::endl;
          }
        }
        if (event.type == SDL_JOYBUTTONUP)
        {
          std::cout << "Joystick button released.\n";
        }
      }
      SDL_Rect rect = {0, 0, 500, 700};
      SDL_FillRect(screen_, &rect, 0xFFFFFFFF);
      text_print_->reset();
      int joystick_count = SDL_NumJoysticks();
      text_print_->tprint(screen_, "Number of joysticks: " + std::to_string(joystick_count));
      text_print_->indent();

      for (const auto &[id, joystick] : joysticks)
      {
        int jid = SDL_JoystickInstanceID(joystick);
        text_print_->tprint(screen_, "Joystick " + std::to_string(jid));
        text_print_->indent();
        std::string name = SDL_JoystickName(joystick);
        text_print_->tprint(screen_, "Joystick name: " + name);
        char pszGUID[40];
        SDL_JoystickGetGUIDString(
            SDL_JoystickGetGUID(joystick), pszGUID, sizeof(pszGUID));
        text_print_->tprint(screen_, "GUID: " + std::string(pszGUID));
        SDL_JoystickPowerLevel powerlevel = SDL_JoystickCurrentPowerLevel(joystick);
        text_print_->tprint(screen_, "Joystick's power level: " + std::to_string(powerlevel));
        int num_axis = SDL_JoystickNumAxes(joystick);
        text_print_->tprint(screen_, "Number of axes: " + std::to_string(num_axis));
        text_print_->indent();
        for (int i = 0; i < num_axis; i++)
        {
          short axis = SDL_JoystickGetAxis(joystick, i);
          text_print_->tprint(screen_, "Axis " + std::to_string(i) + " value: " + std::to_string(axis));
          short filtered_axis = abs(axis) > 3000 ? axis : 0; // 需要给手柄设置死区，因为松手不会完全回零
          axes(i) = filtered_axis;
        }
        text_print_->unindent();
        // buttons, hats
        int num_button = SDL_JoystickNumButtons(joystick);
        text_print_->tprint(screen_, "Number of buttons: " + std::to_string(num_button));
        text_print_->indent();
        for (int i = 0; i < num_button; i++)
        {
          bool button = SDL_JoystickGetButton(joystick, i);
          text_print_->tprint(screen_, "Button " + std::to_string(i) + " value: " + std::to_string(button));
          buttons(i) = button;
        }
        text_print_->unindent();
        int num_hat = SDL_JoystickNumHats(joystick);
        text_print_->tprint(screen_, "Number of hats: " + std::to_string(num_hat));
        text_print_->indent();
        for (int i = 0; i < num_hat; i++)
        {
          Uint8 hat = SDL_JoystickGetHat(joystick, i);
          text_print_->tprint(screen_, "Hat " + std::to_string(i) + " value: " + std::to_string(hat));
          hats(i) = hat;
        }
        text_print_->unindent();
        text_print_->unindent();
      }
      SDL_UpdateWindowSurface(win_);
      // SDL_FreeSurface(screen);
      return drake::systems::EventStatus::Succeeded();
    }

  } // namespace biped
} // namespace my_drake_examples
