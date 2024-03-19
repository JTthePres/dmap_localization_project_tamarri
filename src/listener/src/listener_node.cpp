#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "dmap.h"
#include "draw_helpers.h"
#include "dmap_localizer.h"

ros::Publisher pose_pub;
double x_origin, y_origin;
bool map_received = false;
bool init_received = false;

GridMapping grid_mapping;
DMapLocalizer localizer;
std::vector<Vector2f> obstacles;
DMap dmap;

// parameters
float max_range=10;
float resolution;
float expansion_range=1;

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;
    uint32_t map_width = msg->info.width;
    uint32_t map_height = msg->info.height;
    grid_mapping.reset(Vector2f(origin_x, origin_y), resolution);
    for (int i = 0; i < map_height; ++i)
    {
        for (int j = 0; j < map_width; ++j)
        {
            int index = i * map_width + j;
            int value = msg->data[index];
            if (value == 100)
            {
                ROS_INFO("Occupied cell at (%d, %d)", j, i);
                Vector2f coord = grid_mapping.grid2world(Vector2f(j,i));
                obstacles.push_back(coord);
            }
        }
    }
    localizer.setMap(obstacles, resolution, 10);
    map_received = true;
 
}


void scanCallback(const sensor_msgs::LaserScan& scan) {
  if (!map_received || !init_received){
    ROS_INFO("Map or initial pose not received");
    return;
  }
  std::vector<Vector2f> scan_endpoints;
  for (size_t i=0; i<scan.ranges.size(); ++i) {
    float alpha=scan.angle_min+i*scan.angle_increment;
    float r=scan.ranges[i];
    if (r< scan.range_min || r> scan.range_max)
      continue;
    scan_endpoints.push_back(Vector2f(r*cos(alpha), r*sin(alpha)));
  }

  localizer.localize(scan_endpoints,10);
  
  nav_msgs::Odometry odom;
  Isometry2f X = localizer.X;
  odom.pose.pose.position.x = X.translation().x();
  odom.pose.pose.position.y = X.translation().y();
  odom.header.frame_id = "map";
  odom.child_frame_id = "robot";  
  Eigen::Matrix3f R;
  R << X.linear()(0,0), X.linear()(0,1), 0,
       X.linear()(1,0), X.linear()(1,1), 0,
       0,      0,      1;
  Eigen::Quaternionf q(R);
  odom.pose.pose.orientation.x = q.x();
  odom.pose.pose.orientation.y = q.y();
  odom.pose.pose.orientation.z = q.z();
  odom.pose.pose.orientation.w = q.w();
  pose_pub.publish(odom);
  std::cerr << localizer.X.translation().transpose()<< std::endl;
}


void initCallback(const geometry_msgs::PoseWithCovarianceStamped &pose)
{
    x_origin = pose.pose.pose.position.x;
    y_origin = pose.pose.pose.position.y;
    Vector2f transl = Vector2f(x_origin, y_origin);
    ROS_INFO("Initial pose: (%f, %f)", x_origin, y_origin);
    Eigen::Quaternionf q(pose.pose.pose.orientation.w, pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z);
    Eigen::Matrix2f matrix = q.toRotationMatrix().block<2,2>(0,0);
    std::cout << "Matrice di rotazione:\n" << matrix << std::endl;
    Isometry2f isometry = Isometry2f::Identity();
    isometry.translation() = transl;
    isometry.linear()=matrix.cast<float>();
    localizer.X = isometry;
    init_received = true;
    nav_msgs::Odometry odom;
    Isometry2f X = localizer.X;
    odom.pose.pose.position.x = X.translation().x();
    odom.pose.pose.position.y = X.translation().y();
    odom.header.frame_id = "map";
    odom.child_frame_id = "robot";  
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    pose_pub.publish(odom);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener_node");
    ros::NodeHandle n;
    pose_pub =n.advertise<nav_msgs::Odometry>("/oodom", 10);;
    ros::Subscriber sub_map = n.subscribe("/map", 1, mapCallback);
    ros::Subscriber sub_init = n.subscribe("/initialpose", 0, initCallback);
    ros::Subscriber sub_laser = n.subscribe("/base_scan", 0, scanCallback);

  while (ros::ok()) {
    ros::spinOnce();
  }
    return 0;
}
