find_package(OpenMP)
if (OpenMP_FOUND)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()
add_executable(
    mapper ros_mapper.cpp octomapper.cpp octomapper.h ros_mapper.h map_utils.cpp map_utils.h mapper.cpp mapper.h)
add_dependencies(mapper igvc_msgs_gencpp)
target_link_libraries(mapper ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES})
