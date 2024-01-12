# SPDX-License-Identifier: MIT-0

include_directories(include)

add_library(
    socket_base
    src/socket_base.cpp
)
target_link_libraries(
    socket_base
    pthread
    data_converter
)

add_library(
    data_converter
    src/data_converter.cpp
)
target_link_libraries(
    data_converter 
)

add_library(
    tcp_server
    src/tcp_server.cpp
)

add_library(
    isaac_drake
    src/isaac_sim_system.cpp
)
target_link_libraries(isaac_drake tcp_server drake::drake)
target_link_libraries(
    tcp_server
    socket_base
    pthread
    data_converter
   
)
