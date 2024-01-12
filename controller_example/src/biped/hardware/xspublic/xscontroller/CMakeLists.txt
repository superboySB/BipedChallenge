file(GLOB SRC_FILES "*.cpp" "*.c")
file(GLOB INC_FILES "*.h")

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include_directories(..)

add_compile_definitions(HAVE_JOURNALLER XSENS_DEBUG)

option(HAVE_LIBUSB "actually useless, doesn't matter.")
if(DEFINED HAVE_LIBUSB)
  add_compile_definitions("HAVE_LIBUSB")
endif()

add_library(xscontroller ${SRC_FILES} ${INC_FILES} xscontrollerconfig.h)
target_precompile_headers(xscontroller PRIVATE xscontrollerconfig.h)
target_link_directories(xscontroller PRIVATE ..)
target_link_libraries(xscontroller xstypes)