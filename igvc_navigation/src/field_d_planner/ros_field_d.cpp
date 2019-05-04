/**
 * A frontend for the Field D path planner, acting as a node that takes in callbacks and publishes
 * directly.
 *
 * Author: Oswin So <oswinso@gmail.com>
 */
#include <ros/node_handle.h>
#include "ros_field_d.h"

namespace field_d
{
ROSFieldD::ROSFieldD() : nh_{}, pNh_{"~"} {
  setupSubscribers();
  setupPublishers();
  getParams();
  planner_.setOptions(planner_options_);

  ros::spin();
}

void ROSFieldD::setupSubscribers()
{
  map_sub_ = nh_.subscribe("/map", 1, &ROSFieldD::mapCallback, this);
  waypoint_sub_ = nh_.subscribe("/waypoint", 1, &ROSFieldD::waypointCallback, this);
}

void ROSFieldD::setupPublishers()
{
  path_pub_ = nh_.advertise<nav_msgs::Path>("/path", 1);
  expanded_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/expanded/pointcloud", 1);
  nodes_expanded_pub_ = nh_.advertise<std_msgs::Int32>("/expanded/num_nodes_expanded", 1);
  nodes_updated_pub_ = nh_.advertise<std_msgs::Int32>("/expanded/num_nodes_updated", 1);
}

void ROSFieldD::getParams()
{
  igvc::getParam(pNh_, "c_space", planner_options_.configuration_space_);
  igvc::getParam(pNh_, "maximum_distance", planner_options_.maximum_distance_);
  igvc::getParam(pNh_, "goal_range", planner_options_.goal_range_);
  igvc::getParam(pNh_, "publish_expanded", planner_options_.publish_expanded_);
  igvc::getParam(pNh_, "follow_old_path", planner_options_.follow_old_path_);
  igvc::getParam(pNh_, "lookahead_dist", planner_options_.lookahead_dist_);
  igvc::getParam(pNh_, "occupancy_threshold", planner_options_.occupancy_threshold_);
}

void ROSFieldD::planPath() {
  std::optional<PlannerResult> result = planner_.planPath(start, end);
  if (result)
  {
    path_pub_.publish(result->path);
    nodes_expanded_pub_.publish(result->num_nodes_expanded);
    nodes_updated_pub_.publish(result->num_nodes_updated);

    if (planner_options_.publish_expanded_)
    {
      expanded_pub_.publish(result->expanded_cloud);
    }
  }
  else
  {
    ROS_WARN_STREAM_THROTTLE(2, "Couldn't get result from planner!");
  }
}

void ROSFieldD::mapCallback(const igvc_msgs::mapConstPtr &msg) {
  planner_.updateMap(msg);

  start.position.x = msg->x;
  start.position.y = msg->y;

  planPath();
}

void ROSFieldD::waypointCallback(const geometry_msgs::PointStampedConstPtr &msg) {
  end.position.x = msg->point.x;
  end.position.y = msg->point.y;

  planPath();
}

}
