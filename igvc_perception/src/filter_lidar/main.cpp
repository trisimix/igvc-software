#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <igvc_utils/NodeUtils.hpp>
#include <string.h>

ros::Publisher filtered_pointcloud_pub;
std::string input_topic, output_topic;

double x_front, x_back;
double y_left, y_right;
double z_up, z_down;

double maximum_distance;

void point_cloud_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& msg)
{
  pcl::PointCloud<pcl::PointXYZ> result;
  for (auto it = msg->points.begin(); it != msg->points.end(); it++)
  {
    if ((((it->x >= x_front) || (it->x <= -x_back)) ||
        ((it->y >= y_left)  || (it->y <= -y_right)) ||
        ((it->z >= z_up)    || (it->z <= -z_down))) &&
        (sqrt(pow(it->x, 2) + pow(it->y, 2) + pow(it->z, 2)) < maximum_distance))
    {
      result.push_back(*it);
    }
  }
  result.header.frame_id = msg->header.frame_id;
  result.header.stamp = msg->header.stamp;
  filtered_pointcloud_pub.publish(result);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filter_lidar");

  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  igvc::getParam(pNh, "input_topic", input_topic);
  igvc::getParam(pNh, "output_topic", output_topic);


  igvc::getParam(pNh, "x_front", x_front);
  igvc::getParam(pNh, "x_back", x_back);

  igvc::getParam(pNh, "y_left", y_left);
  igvc::getParam(pNh, "y_right", y_right);

  igvc::getParam(pNh, "z_up", z_up);
  igvc::getParam(pNh, "z_down", z_down);

  igvc::getParam(pNh, "maximum_distance", maximum_distance);

  filtered_pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>(output_topic, 1);
  ros::Subscriber scan_sub = nh.subscribe(input_topic, 1, point_cloud_callback);
  ros::spin();
}
