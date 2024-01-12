# SPDX-License-Identifier: MIT-0
include_directories(./)
add_library(alip_planner alip_planner.cc alip_planner.h)
target_link_libraries(alip_planner drake::drake)

add_library(stair_planner stair_planner.cc stair_planner.h)
target_link_libraries(stair_planner drake::drake)

add_library(jump_planner jump_planner.cc jump_planner.h)
target_link_libraries(jump_planner drake::drake)

add_library(swing_arm_planner swing_arm_planner.cc swing_arm_planner.h)
target_link_libraries(swing_arm_planner drake::drake)
