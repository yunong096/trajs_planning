#ifndef LATTICE_DYNAMIC_ROUTING_H
#define LATTICE_DYNAMIC_ROUTING_H

#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include <string>
#include <vector>
#include <nav_msgs/OccupancyGrid.h>
#include <boost/smart_ptr.hpp>
#include <visualization_msgs/MarkerArray.h>
// #include "DynamicPathStruct.hpp"
#include <tf/transform_broadcaster.h>
#include <waypoint_msgs/Waypoint.h>
#include <waypoint_msgs/WaypointArray.h>
// #include "Obstacles.hpp"
// #include "CoarsePathGenerator.hpp"
// #include "dp_planner.hpp"
// #include "nmpc2.hpp"
// #include "df_planner/traj_manager.h"
#include "altro_optimizer.hpp"


using std::string;
using namespace std;

class Dynamic_routing
{
public:
  Dynamic_routing(void);
  ~Dynamic_routing(void);
  void thread_routing(void);
  void odom_call_back(const nav_msgs::Odometry &odom);
  void control_call_back(const geometry_msgs::Vector3 &msg);
  void start_pose_call_backs(const geometry_msgs::Vector3 &msg);
  void goal_pose_call_backs(const geometry_msgs::Vector3 &msg);
private:
  bool is_set_start, is_set_goal, is_set_refline;
  bool is_reach_goal = false;

  // ROS
  // visual publish
  ros::Publisher global_waypoints_pub_;
  ros::Publisher local_waypoints_pub_;
  ros::Publisher local_paths_s;
  ros::Publisher local_paths_a;
  ros::Publisher local_paths_t;
  ros::Publisher local_paths_k;
  ros::Publisher local_paths_trj;
  ros::Publisher Start_Dynamic;

  // visual sub
  ros::Subscriber odom_sub_;
  ros::Subscriber control_sub_;           // 订阅控制信息
  ros::Subscriber start_pose_subscriber_;
  ros::Subscriber goal_pose_subscriber_;

  // pubmsgs
  geometry_msgs::PoseArray pubLocalPath_a;
  geometry_msgs::PoseArray pubLocalPath_s;
  geometry_msgs::PoseArray pubLocalPath_t;
  geometry_msgs::PoseArray pubLocalPath_k;
  waypoint_msgs::WaypointArray pubLocalPath_trj;
  visualization_msgs::MarkerArray local_path_array;
  visualization_msgs::MarkerArray gobal_path_array;

  // 变量
  std_msgs::String start_dynamic;  // 判断局部规划是否开始
  CartesianState init_car_state{0, 0, 0, 0, 0, 0};
  GlobalPath refline = GlobalPath();
  vector<CartesianState> best_path;
  vector<GlobalPathPoint> ref_path;


  // visual sub
  ros::Subscriber odom_sub;
  ros::Subscriber start_pose_subscriber;
  ros::Subscriber goal_pose_subscriber;
  // thread
  boost::thread *routing_thread_;

  void publishPathMarker(const std::vector<GlobalPathPoint>& path) {
    // Create a marker for each point in the path
    for (size_t i = 0; i < path.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.stamp = ros::Time::now();
        marker.header.frame_id = "map";
        marker.ns = "path";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = path[i].x;
        marker.pose.position.y = path[i].y;
        marker.pose.position.z = 0.3;
        marker.pose.orientation.w = 1.0;  // Quaternion representing no rotation
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;  // Alpha (opacity)
        marker.color.r = 0.0;  // Red
        marker.color.g = 1.0;  // Green
        marker.color.b = 0.0;  // Blue
 
        gobal_path_array.markers.push_back(marker);
    }
 
    // Publish the marker array
    global_waypoints_pub_.publish(gobal_path_array);
}
};

#endif