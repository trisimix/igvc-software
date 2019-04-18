#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <mutex>
#include <geometry_msgs/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <igvc_utils/NodeUtils.hpp>
#include <robot_localization/navsat_conversions.h>

ros::Publisher g_ground_truth_pub;
// TODO make this a minimal object
nav_msgs::Odometry g_og_pose;
ros::Time g_last_estimate;

void odom_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  g_last_estimate = msg->header.stamp;
}

void ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg) {
  // get the starting location as the origin
  if(g_og_pose.header.stamp.toSec() == 0) {
    g_og_pose.pose = msg->pose;
    g_og_pose.header = msg->header;
    g_og_pose.pose.pose.position.x = msg->pose.pose.position.x;
    g_og_pose.pose.pose.position.y = msg->pose.pose.position.y;
    ROS_INFO_STREAM("setting g_og_pose to " << g_og_pose.pose.pose.position.x << ", "
                    << g_og_pose.pose.pose.position.y);
  } else {
    nav_msgs::Odometry result;
    result.pose = msg->pose;

    // use the initial location as an offset (makes the starting location 0, 0)
    result.pose.pose.position.x = msg->pose.pose.position.x - g_og_pose.pose.pose.position.x;
    result.pose.pose.position.y = msg->pose.pose.position.y - g_og_pose.pose.pose.position.y;

    result.twist = msg->twist;

    // set up the correct header
    result.header = msg->header;
    result.child_frame_id = "/base_footprint";
    result.header.frame_id = "/odom";

    tf::Quaternion quat(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    // publish odom message
    g_ground_truth_pub.publish(result);

    // publish transform for tf if there has not been a update from the localization node in the last second
    // since it also publishes the same transform
    if(std::abs(msg->header.stamp.toSec() - g_last_estimate.toSec()) > 1.0) {
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z) );
      transform.setRotation(quat);
      br.sendTransform(tf::StampedTransform(transform, msg->header.stamp, "odom", "base_footprint"));

      tf::Transform utm_to_odom;

    }
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_truth_republisher");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string ground_truth_topic, estimate_topic, pub_topic, diff_topic;
  igvc::param(pNh, "ground_truth_sub_topic", ground_truth_topic, std::string("/ground_truth/state_raw"));
  igvc::param(pNh, "ground_truth_pub_topic", pub_topic, std::string("/ground_truth"));

  double longitude, latitude;
  igvc::param(pNh, "longitude", longitude, -84.405001);
  igvc::param(pNh, "latitude", latitude, 33.774497);

  ros::Subscriber ground_truth = nh.subscribe<nav_msgs::Odometry>(ground_truth_topic, 10,
                                                         ground_truth_callback);

  ros::Subscriber estimate_sub = nh.subscribe<nav_msgs::Odometry>("/odometry/filtered", 1, odom_callback);
  g_ground_truth_pub = nh.advertise<nav_msgs::Odometry>(pub_topic, 1);

  double utm_x, utm_y;
  RobotLocalization::NavsatConversions::UTM(latitude, longitude, &utm_x, &utm_y);

  tf::Transform utm_to_odom;
  utm_to_odom.setOrigin( tf::Vector3(utm_x - g_og_pose.pose.pose.position.x, utm_y - g_og_pose.pose.pose.position.y, 0.0) );
  utm_to_odom.setRotation( tf::Quaternion(0.0, 0.0, 0.0, 1.0) );
  tf::TransformBroadcaster br;

  ros::Rate rate(20);
  while(ros::ok()) {
    br.sendTransform(tf::StampedTransform(utm_to_odom, ros::Time::now(), "utm", "odom"));
    ros::spinOnce();
    rate.sleep();
  }
  ros::spin();
}
