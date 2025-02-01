#ifndef __GLOBAL_ROUTING_H
#define __GLOBAL_ROUTING_H

#include <iostream>
#include <ros/ros.h>
#include <boost/thread.hpp>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "std_msgs/String.h"
#include "global_path_struct.hpp"
#include "lqr_controller.h"
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Polygon.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include "ackermann_msgs/AckermannDrive.h"
#include <waypoint_msgs/Waypoint.h>
#include <waypoint_msgs/WaypointArray.h>
#include "Obstacles.hpp"
#include "DynamicPathStruct.hpp"
#include "MovingObs.hpp"

using namespace std;

class GlobalRouting
{
public:
  GlobalRouting();
  ~GlobalRouting();
  void thread_routing(void);

  void odom_call_back(const nav_msgs::Odometry &msg);

  void Vehival_Theta(const geometry_msgs::PoseArray &theta);
  void Vehival_Kappa(const geometry_msgs::PoseArray &kappa);
  void Vehival_Acc(const geometry_msgs::PoseArray &acc);
  void Vehival_Speed(const geometry_msgs::PoseArray &speed);
  void Vehival_Go(const std_msgs::String::ConstPtr &go);
  void Vehival_Traj(const waypoint_msgs::WaypointArray::ConstPtr &msg);
  bool Vehical_Stop(const geometry_msgs::Pose &goal_point, nav_msgs::Odometry &odom);

  void publish_car_start_pose(const geometry_msgs::Pose &start_pose);
  void publish_car_goal_pose(const geometry_msgs::Pose &goal_pose);

  void Turn_obstacles_into_squares(visualization_msgs::Marker &marker,  const MatrixXd& poly, int id)
  {
    double Length = fabs(poly(0, 0) - poly(0, 1));
    double width = fabs(poly(1, 1) - poly(1, 2));
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id; // 注意了
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = fabs(poly(0, 0) + poly(0, 2)) / 2;
    marker.pose.position.y = fabs(poly(1, 0) + poly(1, 2)) / 2;
    // cout << Length << " "<< width<< " " << marker.pose.position.x<< " " << marker.pose.position.y<< " " << endl;
    // cout << fabs(poly(0, 0) - poly(0, 1)) << endl;
    marker.pose.position.z = 0.1;
    marker.scale.x = Length;
    marker.scale.y = width;
    marker.scale.z = 0.01;
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
    if(id == 0) {
      marker.color.r = 1.0f;
      marker.color.g = 1.0f;
      marker.color.b = 1.0f;
      marker.pose.position.z = 0.0;
    }
    // else if(id > 9) {
    //   cout << "plot moving obs: " << marker.pose.position.x << ", " << marker.pose.position.y << endl;
    //   marker.scale.x = 2.7;
    //   marker.scale.y = 2.0;
    //   marker.color.r = 0.0f;
    //   marker.color.g = 0.0f;
    //   marker.color.b = 1.0f;
    //   marker.pose.position.z = 0.2;
    // }
    // marker.lifetime = ros::Duration();
  }

  void Turn_moving_obstacles_into_squares(visualization_msgs::Marker &marker,  const Trajectory& cur_obs, int id)
  {
    double Length = 4.7;
    double width = 2.0;
    marker.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, cur_obs.cur_state.theta);;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = id; // 注意了
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = cur_obs.cur_state.x;
    marker.pose.position.y = cur_obs.cur_state.y;
    marker.pose.position.z = 0.2;
    // cout << Length << " "<< width<< " " << marker.pose.position.x<< " " << marker.pose.position.y<< " " << endl;
    // cout << fabs(poly(0, 0) - poly(0, 1)) << endl;
      // cout << "plot moving obs: " << marker.pose.position.x << ", " << marker.pose.position.y << endl;
      marker.scale.x = Length;
      marker.scale.y = width;
      marker.color.r = 0.0f;
      marker.color.g = 0.0f;
      marker.color.b = 1.0f;
      marker.color.a = 1.0;
    // marker.lifetime = ros::Duration();
  }

private:
  std::vector<double> speeds, thetas, kappas, accs; // 获取局部规划的速度与航向角,曲率
  std::vector<car_state> local_waypoints;     // 局部规划路径点
  nav_msgs::Odometry car_odom_; //用于接收当前odom
  car_state cur_pose; // 车辆当前状态

  // 函数对象
  geometry_msgs::Vector3 msg_ros;           // 发布控制数据
  visualization_msgs::MarkerArray map_points;
  visualization_msgs::MarkerArray obstacle_MarkerArray; //发布障碍数据，障碍可视化

  // 控制算法
  string controller;
  LqrController Lqr;

  // 路径点
  nav_msgs::Path dynamic_points;

  // 点
  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose goal_pose_;

  // visual Pub
  ros::Publisher obs_pub_;
  ros::Publisher map_points_pub_;
  ros::Publisher control_data_pub_;
  ros::Publisher vehicle_start_pose_pub_;
  ros::Publisher vehicle_goal_pose_pub_;

  // visual Subscriber
  ros::Subscriber odom_sub_;
  ros::Subscriber vehival_theta;
  ros::Subscriber vehival_kappa;
  ros::Subscriber vehival_acc;
  ros::Subscriber vehival_speed;
  ros::Subscriber vehival_go;
  ros::Subscriber vehival_traj;

  // thread
  boost::thread *routing_thread_;

  // flag
  double goal_distanse;     
  double car_speed;         // 车速度
  bool is_vehical_stop_set; // 判断车是否要停下来
  bool is_start_pose_set;   // 是否定义了起点
  bool is_goal_pose_set;    // 是否定义了终点
  string Start_dynamic;     // 判断局部规划是否开始

  //不带控制仿真index计数
  int simulation_count_ = 0;

   Obstacles obs;
};

#endif