#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "fake_map_server");
  ros::NodeHandle nh;
  ros::Rate loop_rate(25);

  ros::Publisher pub_map = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1, true);
  std::string frame_id = "map";
  double resolution = 0.1;
  double xMin=-100, xMax=100, yMin=-100, yMax=100;

  nav_msgs::OccupancyGrid map;
  int width = ceil((xMax-xMin)/resolution);
  int height = ceil((yMax-yMin)/resolution);

  map.info.resolution = resolution;
  map.info.width = width;
  map.info.height = height;
  map.info.map_load_time = ros::Time::now();

  map.info.origin.position.x = xMin;
  map.info.origin.position.y = yMin;
  map.info.origin.position.z = 0;
  map.info.origin.orientation.x = 0;
  map.info.origin.orientation.y = 0;
  map.info.origin.orientation.z = 0;
  map.info.origin.orientation.w = 1;

  map.header.frame_id = frame_id;

  map.data.resize(map.info.width * map.info.height);

  while (ros::ok()){
    map.header.stamp = ros::Time::now();
    pub_map.publish(map);
    loop_rate.sleep();
  }
}