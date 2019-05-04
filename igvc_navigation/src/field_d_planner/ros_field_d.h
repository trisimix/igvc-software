#ifndef SRC_ROS_FIELD_D_H
#define SRC_ROS_FIELD_D_H

#include <geometry_msgs/PointStamped.h>

#include <igvc_msgs/map.h>

#include "FieldDPlanner.h"

namespace field_d
{
class ROSFieldD {
 public:
  ROSFieldD();
 private:
  void setupSubscribers();
  void setupPublishers();
  void getParams();

  void planPath();

  void mapCallback(const igvc_msgs::mapConstPtr &msg);
  void waypointCallback(const geometry_msgs::PointStampedConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle pNh_;

  FieldDPlanner planner_{};
  PlannerOptions planner_options_{};

  geometry_msgs::Pose start;
  geometry_msgs::Pose end;

  ros::Subscriber map_sub_;
  ros::Subscriber waypoint_sub_;

  ros::Publisher path_pub_;
  ros::Publisher expanded_pub_;
  ros::Publisher nodes_expanded_pub_;
  ros::Publisher nodes_updated_pub_;
};

}
#endif //SRC_ROS_FIELD_D_H
