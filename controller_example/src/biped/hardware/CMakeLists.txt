add_subdirectory(xspublic)
# include_directories(xspublic)

add_library(xsens_mti xsens_mti.cc xsens_mti.h)
target_link_libraries(xsens_mti drake::drake
                      xstypes xscontroller xscommon pthread rt)
target_include_directories(xsens_mti PUBLIC xspublic)