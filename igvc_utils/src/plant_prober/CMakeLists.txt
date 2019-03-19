add_executable(plant_probe main.cpp)
add_dependencies(plant_probe igvc_msgs_gencpp)
target_link_libraries(plant_probe ${catkin_LIBRARIES})
