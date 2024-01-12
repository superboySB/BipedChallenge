# SPDX-License-Identifier: MIT-0
set(CMAKE_BUILD_TYPE "Debug")
add_subdirectory(others)
add_subdirectory(planner)
add_subdirectory(controller)
add_subdirectory(estimator)
add_subdirectory(hardware)
add_subdirectory(tcp)

add_executable(task1_controller biped_sim_server_nonblocking.cc)
target_link_libraries(task1_controller
        tcp_server data_converter isaac_drake stair_planner
        drake::drake joystick_receiver command_interpreter alip_planner
        feedback_controller dummy_estimator mit_estimator imu ankle_estimator gflags pthread)

add_executable(task2_controller biped_sim_server_nonblocking.cc)
target_link_libraries(task2_controller
            tcp_server data_converter isaac_drake stair_planner
            drake::drake joystick_receiver command_interpreter alip_planner
            feedback_controller dummy_estimator mit_estimator imu ankle_estimator gflags pthread)

add_executable(task3_controller jump_sim_server.cc)
target_link_libraries(task3_controller isaac_drake
                    drake::drake jump_planner tau_controller jump_estimator gflags)

add_executable(task4_controller biped_sim_server_nonblocking.cc)
target_link_libraries(task4_controller
            tcp_server data_converter isaac_drake stair_planner
            drake::drake joystick_receiver command_interpreter alip_planner
            feedback_controller dummy_estimator mit_estimator imu ankle_estimator gflags pthread)

add_executable(task5_controller stair_sim_server.cc)
target_link_libraries(task5_controller isaac_drake
        drake::drake joystick_receiver command_interpreter stair_planner alip_planner
        feedback_controller dummy_estimator mit_estimator imu ankle_estimator gflags pthread)

add_executable(task6_controller biped_sim_server_nonblocking.cc)
target_link_libraries(task6_controller
            tcp_server data_converter isaac_drake stair_planner
            drake::drake joystick_receiver command_interpreter alip_planner
            feedback_controller dummy_estimator mit_estimator imu ankle_estimator gflags pthread)
        