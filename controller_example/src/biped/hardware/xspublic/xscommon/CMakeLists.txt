file(GLOB SRC_FILES "*.cpp" "*.c")
file(GLOB INC_FILES "*.h")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(../)

add_compile_definitions(HAVE_JOURNALLER XSENS_DEBUG)
add_compile_definitions(JLDEF_BUILD=JLL_TRACE)

add_library(xscommon ${SRC_FILES} ${INC_FILES})
target_precompile_headers(xscommon PRIVATE ../xstypes/xsens_compat.h xscommon_config.h)