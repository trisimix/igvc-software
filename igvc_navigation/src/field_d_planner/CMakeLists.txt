add_executable(field_d_planner FieldDPlanner.cpp main.cpp field_d_service.cpp field_d_service.h ros_field_d.cpp ros_field_d.h)
add_dependencies(field_d_planner igvc_msgs_gencpp)
target_link_libraries(field_d_planner ${catkin_LIBRARIES} ${PCL_LIBRARIES} GraphSearch)
