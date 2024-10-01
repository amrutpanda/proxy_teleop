set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(simviz simulation_teleop/simviz.cpp)
target_link_libraries(simviz ${PROXY_TELEOP_COMMON_LIBRARIES})

add_executable(robot_controller simulation_teleop/robot_controller.cpp)
target_link_libraries(robot_controller ${PROXY_TELEOP_COMMON_LIBRARIES})

# add_executable(test controller_testing/controller_test.cpp)
# target_link_libraries(test ${PROXY_TELEOP_COMMON_LIBRARIES})

add_executable(haptic_controller simulation_teleop/haptic_controller.cpp)
target_link_libraries(haptic_controller ${PROXY_TELEOP_COMMON_LIBRARIES})
