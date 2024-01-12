# SPDX-License-Identifier: MIT-0

add_library(quaternion_sensor quaternion_sensor.cc quaternion_sensor.h)
target_link_libraries(quaternion_sensor drake::drake)

add_library(imu imu.cc imu.h)
target_link_libraries(imu drake::drake quaternion_sensor)

add_library(joystick_receiver joystick_receiver.cc joystick_receiver.h)
target_link_libraries(joystick_receiver drake::drake ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY})

add_library(keyboard_receiver keyboard_receiver.cc keyboard_receiver.h)
target_link_libraries(keyboard_receiver drake::drake ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY})

add_library(command_interpreter command_interpreter.cc command_interpreter.h)
target_link_libraries(command_interpreter drake::drake)



# add_executable(joystick_receiver_sim joystick_receiver_sim.cc) # 这两个是为了实验手柄程序，现在也许可以删了
# target_link_libraries(joystick_receiver_sim drake::drake joystick_receiver command_interpreter)

# add_executable(joystick_demo joystick_demo.cc)
# target_link_libraries(joystick_demo ${SDL2_LIBRARIES} ${SDL2_TTF_LIBRARY})

# add_executable(mbp_introspec mbp_introspec.cc) # 已初步验证pivot model inverse dynamics的想法
# target_link_libraries(mbp_introspec drake::drake)

add_executable(hello hello.cc)
target_link_libraries(hello drake::drake)
