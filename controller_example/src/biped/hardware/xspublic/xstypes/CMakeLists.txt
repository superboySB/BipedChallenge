file(GLOB SRC_FILES "*.cpp" "*.c")
file(GLOB INC_FILES "*.h")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(..)

add_library(xstypes ${SRC_FILES} ${INC_FILES})
target_link_libraries(xstypes dl)