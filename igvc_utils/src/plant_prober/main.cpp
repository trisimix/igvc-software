#include <igvc_msgs/velocity_pair.h>
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>

void stepFunction(double velocity, ros::Publisher cmd_pub)
{
  ROS_INFO_STREAM("Now probing motor value " << velocity);
  // Stay stationary for 1 second
  igvc_msgs::velocity_pair motor;
  motor.left_velocity = 0;
  motor.right_velocity = 0;
  motor.header.stamp = ros::Time::now();
  cmd_pub.publish(motor);
  ros::Duration(1).sleep();

  // Run at desired velocity for 5 seconds
  motor.left_velocity = velocity;
  motor.right_velocity = velocity;
  motor.header.stamp = ros::Time::now();
  cmd_pub.publish(motor);
  ros::Duration(5).sleep();

  ROS_INFO_STREAM("Finished probing " << velocity);
  // Stop the motor
  motor.left_velocity = 0;
  motor.right_velocity = 0;
  motor.header.stamp = ros::Time::now();
  cmd_pub.publish(motor);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plant_probe");
  ros::NodeHandle nh;

  double velocity_range;
  igvc::getParam(nh, "veloctiy_range", velocity_range);
  ros::Publisher cmd_pub = nh.advertise<igvc_msgs::velocity_pair>("/motors", 1);

  double velocity = -velocity_range;
  ROS_INFO_STREAM("Starting probing sequence from -255 to 255...");
  while (ros::ok())
  {
      stepFunction(velocity, cmd_pub);
      velocity += 10;
      if (velocity >= 255)
      {
        ROS_INFO_STREAM("Finished probing! Shutting down node...");
        ros::shutdown();
      }
  }
  return 0;
}
