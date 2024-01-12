add_library(dummy_estimator dummy_estimator.cc dummy_estimator.h)
target_link_libraries(dummy_estimator drake::drake)

add_library(jump_estimator jump_estimator.cc jump_estimator.h)
target_link_libraries(jump_estimator drake::drake)

add_library(ankle_estimator ankle_estimator.cc ankle_estimator.h)
target_link_libraries(ankle_estimator drake::drake)

add_library(mit_estimator mit_estimator.cc mit_estimator.h)
target_link_libraries(mit_estimator drake::drake)
