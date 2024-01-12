# SPDX-License-Identifier: MIT-0

add_library(tau_controller tau_controller.cc tau_controller.h)
target_link_libraries(tau_controller drake::drake)

add_library(feedback_controller feedback_controller.cc feedback_controller.h)
target_link_libraries(feedback_controller drake::drake)

add_library(feedback_controller_async feedback_controller_async.cc feedback_controller_async.h)
target_link_libraries(feedback_controller_async drake::drake feedback_controller)

add_library(pivot_model_controller pivot_model_controller.cc pivot_model_controller.h)
target_link_libraries(pivot_model_controller drake::drake)

add_library(whole_body_controller whole_body_controller.cc whole_body_controller.h)
target_link_libraries(whole_body_controller drake::drake)
