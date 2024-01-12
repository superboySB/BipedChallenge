#include <iostream>
#include <sstream>
#include <string>
#include <memory>
#include <map>

#include <SDL2/SDL.h>
#include <SDL2/SDL_video.h>
#include <SDL2/SDL_ttf.h>

namespace
{

  class TextPrint
  {
  public:
    TextPrint()
    {
      reset();
      font = TTF_OpenFont("sans.ttf", 15);
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

  int do_main()
  {
    int ret = 0;
    SDL_Window *win = SDL_CreateWindow("Joystick example",
                                       SDL_WINDOWPOS_UNDEFINED,
                                       SDL_WINDOWPOS_UNDEFINED,
                                       500, 700,
                                       SDL_WINDOW_SHOWN);
    // 它是SDL1风格。但是反正我们用不上显卡，什么图形硬件加速之类不重要。
    SDL_Surface *screen = SDL_GetWindowSurface(win);
    TextPrint text_print;
    std::map<int, SDL_Joystick *> joysticks;

    bool done = false;
    while (!done)
    {
      SDL_Event event;
      while (SDL_PollEvent(&event))
      {
        if (event.type == SDL_QUIT)
        {
          done = true;
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
        if (event.type == SDL_JOYDEVICEADDED)
        {
          SDL_Joystick *joy = SDL_JoystickOpen(event.jdevice.which);
          joysticks.emplace(SDL_JoystickInstanceID(joy), joy);
          std::cout << "Joystick " << SDL_JoystickInstanceID(joy) << " connected\n";
        }
        if (event.type == SDL_JOYDEVICEREMOVED)
        {
          SDL_JoystickClose(joysticks[event.jdevice.which]);
          joysticks.erase(event.jdevice.which);
          std::cout << "Joystick " << event.jdevice.which << " disconnected\n";
        }
      }
      SDL_Rect rect = {0, 0, 500, 700};
      SDL_FillRect(screen, &rect, 0xFFFFFFFF);

      text_print.reset();
      int joystick_count = SDL_NumJoysticks();
      text_print.tprint(screen, "Number of joysticks: " + std::to_string(joystick_count));
      text_print.indent();

      for (const auto &[id, joystick] : joysticks)
      {
        int jid = SDL_JoystickInstanceID(joystick);
        text_print.tprint(screen, "Joystick " + std::to_string(jid));
        text_print.indent();
        std::string name = SDL_JoystickName(joystick);
        text_print.tprint(screen, "Joystick name: " + name);
        char pszGUID[40];
        SDL_JoystickGetGUIDString(
            SDL_JoystickGetGUID(joystick), pszGUID, sizeof(pszGUID));
        text_print.tprint(screen, "GUID: " + std::string(pszGUID));
        SDL_JoystickPowerLevel powerlevel = SDL_JoystickCurrentPowerLevel(joystick);
        text_print.tprint(screen, "Joystick's power level: " + std::to_string(powerlevel));
        int axes = SDL_JoystickNumAxes(joystick);
        text_print.tprint(screen, "Number of axes: " + std::to_string(axes));
        text_print.indent();
        for (int i = 0; i < axes; i++)
        {
          short axis = SDL_JoystickGetAxis(joystick, i);
          text_print.tprint(screen, "Axis " + std::to_string(i) + " value: " + std::to_string(axis));
        }
        text_print.unindent();
        // buttons, hats
        int buttons = SDL_JoystickNumButtons(joystick);
        text_print.tprint(screen, "Number of buttons: " + std::to_string(buttons));
        text_print.indent();
        for (int i = 0; i < buttons; i++)
        {
          bool button = SDL_JoystickGetButton(joystick, i);
          text_print.tprint(screen, "Button " + std::to_string(i) + " value: " + std::to_string(button));
        }
        text_print.unindent();
        int hats = SDL_JoystickNumHats(joystick);
        text_print.tprint(screen, "Number of hats: " + std::to_string(hats));
        text_print.indent();
        for (int i = 0; i < hats; i++)
        {
          Uint8 hat = SDL_JoystickGetHat(joystick, i);
          text_print.tprint(screen, "Hat " + std::to_string(i) + " value: " + std::to_string(hat));
        }
        text_print.unindent();
        text_print.unindent();
      }
      SDL_UpdateWindowSurface(win);
      SDL_Delay(30);
    }
    // SDL_DestroyWindowSurface(win);
    SDL_FreeSurface(screen);
    SDL_DestroyWindow(win);
    return 0;
  }

} // namespace

int main(int argc, char *argv[])
{
  SDL_Init(SDL_INIT_EVERYTHING);
  TTF_Init();
  return do_main();
  SDL_Quit();
}
