/**
 * Temporary launch for Field D planner that allows for
 * toggling between callback and service implementations for testing
 */
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>

#include "ros_field_d.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "field_d_planner");
  ros::NodeHandle pNh("~");

  bool use_services = false;
  igvc::param(pNh, "use_services", use_services, false);

  if (use_services)
  {
    // TODO: Create services server for Field D
  }
  else
  {
    field_d::ROSFieldD planner;
  }
  return 0;
}
