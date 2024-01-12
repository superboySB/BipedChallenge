'''
controller和joystick差不多，它描述xbox类手柄，
有两个手柄，右边ABXY，有肩键和扳机键，有十字键，中间两三个键
'''

import pygame
import pygame._sdl2.controller

pygame.init()
pygame._sdl2.controller.init()

class TextPrint:
  def __init__(self):
    self.reset()
    self.font = pygame.font.Font(None, 25)

  def tprint(self, screen, text):
    text_bitmap = self.font.render(text, True, (0, 0, 0))
    screen.blit(text_bitmap, (self.x, self.y))
    self.y += self.line_height

  def reset(self):
    self.x = 10
    self.y = 10
    self.line_height = 15

  def indent(self):
    self.x += 10

  def unindent(self):
    self.x -= 10

def main():
  screen = pygame.display.set_mode((500, 700))
  pygame.display.set_caption("Common controller demo")
  clock = pygame.time.Clock()
  text_print = TextPrint()
  controllers = {}

  done = False
  while not done:
    for event in pygame.event.get():
      if event.type == pygame.QUIT:
        done = True
      if event.type == pygame.CONTROLLERBUTTONDOWN:
        print("Controller button pressed.")
        if event.button == pygame.CONTROLLER_BUTTON_A:
          controller = controllers[event.instance_id]
          if controller.rumble(0.7, 0, 500):
            print(f"Rumble effect played on controller {event.instance_id}")
      
      if event.type == pygame.CONTROLLERBUTTONUP:
        print("Controller button released.")
      
      if event.type == pygame.CONTROLLERDEVICEADDED:
        controller = pygame._sdl2.controller.Controller(event.device_index)
        controllers[controller.as_joystick().get_instance_id()] = controller
        print(f"Controller {controller.as_joystick().get_instance_id()} connected")

      if event.type == pygame.CONTROLLERDEVICEREMOVED:
        del controllers[event.instance_id]
        print(f"Controller {controller.as_joystick().get_instance_id()} disconnected")
    
    screen.fill((255, 255, 255))
    text_print.reset()

    controller_count = pygame._sdl2.controller.get_count()

    text_print.tprint(screen, f"Number of controllers: {controller_count}")
    text_print.indent()

    for controller in controllers.values():
      jid = controller.as_joystick().get_instance_id()

      text_print.tprint(screen, f"Controller {jid}")
      text_print.indent()

      # name = controller.get_name()
      name = pygame._sdl2.controller.name_forindex(jid)
      text_print.tprint(screen, f"Controller name: {name}")

      guid = controller.as_joystick().get_guid()
      text_print.tprint(screen, f"GUID: {guid}")

      power_level = controller.as_joystick().get_power_level()
      text_print.tprint(screen, f"Controller's power level: {power_level}")

      axes = controller.as_joystick().get_numaxes()
      text_print.tprint(screen, f"Number of axes: {axes}")
      text_print.indent()

      for i in (pygame.CONTROLLER_AXIS_LEFTX, pygame.CONTROLLER_AXIS_LEFTY,
                pygame.CONTROLLER_AXIS_RIGHTX, pygame.CONTROLLER_AXIS_RIGHTY,
                pygame.CONTROLLER_AXIS_TRIGGERLEFT, pygame.CONTROLLER_AXIS_TRIGGERRIGHT):
        axis = controller.get_axis(i)
        text_print.tprint(screen, f"Axis {i} value: {axis}")
      text_print.unindent()

      buttons = controller.as_joystick().get_numbuttons()
      text_print.tprint(screen, f"Number of buttons: {buttons}")
      text_print.indent()

      for i in (pygame.CONTROLLER_BUTTON_A, pygame.CONTROLLER_BUTTON_B,
                pygame.CONTROLLER_BUTTON_X, pygame.CONTROLLER_BUTTON_Y,
                
                pygame.CONTROLLER_BUTTON_LEFTSHOULDER, pygame.CONTROLLER_BUTTON_RIGHTSHOULDER,
                pygame.CONTROLLER_BUTTON_LEFTSTICK, pygame.CONTROLLER_BUTTON_RIGHTSTICK,
                pygame.CONTROLLER_BUTTON_BACK, pygame.CONTROLLER_BUTTON_GUIDE,
                pygame.CONTROLLER_BUTTON_START):
        button = controller.get_button(i)
        text_print.tprint(screen, f"Button {i:>2} value: {button}")
      text_print.unindent()

      hats = controller.as_joystick().get_numhats()
      text_print.tprint(screen, f"Number of hats: {hats}")
      text_print.indent()

      # Hat position. All or nothing for direction, not a float like
      # get_axis(). Position is a tuple of int values (x, y).
      for i in (pygame.CONTROLLER_BUTTON_DPAD_UP, pygame.CONTROLLER_BUTTON_DPAD_DOWN,
                pygame.CONTROLLER_BUTTON_DPAD_LEFT, pygame.CONTROLLER_BUTTON_DPAD_RIGHT,):
        hat = controller.get_button(i)
        text_print.tprint(screen, f"Hat {i} value: {str(hat)}")
      text_print.unindent()

      text_print.unindent()

    pygame.display.flip()
    clock.tick(30)

if __name__ == "__main__":
    main()
    pygame.quit()
