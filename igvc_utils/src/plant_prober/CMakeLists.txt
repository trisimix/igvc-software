add_executable(plant_prober main.cpp)
add_dependencies(plant_prober igvc_msgs_gencpp)
target_link_libraries(plant_prober ${catkin_LIBRARIES})
