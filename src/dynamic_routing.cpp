#include "dynamic_routing.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
// using namespace plan_manage;
// 全局变量
geometry_msgs::Vector3 start_pose_;   // 起点
geometry_msgs::Vector3 goal_pose_;    // 终点
const std::string Frame_id = "map"; //参考系的定义

/*定义起点位置*/
void Dynamic_routing::start_pose_call_backs(const geometry_msgs::Vector3 &msg)
{
  if (!is_set_start)
  {
      ROS_WARN("get start point");
      start_pose_ = msg;
      is_set_start = true;
  }
}

/*定义起点位置*/
void Dynamic_routing::goal_pose_call_backs(const geometry_msgs::Vector3 &msg)
{
  if (!is_set_goal)
  {
      ROS_WARN("get goal point");
      goal_pose_ = msg;
      is_set_goal = true;
  }
}

/*读回ros odom坐标系数据 , 接收车的里程信息，完成当前帧的规划*/
void Dynamic_routing::odom_call_back(const nav_msgs::Odometry &msg)
{
  // 坐标转换
  geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_quat, quat);

  // 根据转换后的四元数，获取roll pitch yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  init_car_state.x = msg.pose.pose.position.x;
  init_car_state.y = msg.pose.pose.position.y;
  init_car_state.theta = yaw;

  double vx = msg.twist.twist.linear.x;
  double vy = msg.twist.twist.linear.y;

  // 考虑不再获取速度，直接模拟
  init_car_state.speed = hypot(vx, vy);
  // cout << "当前速度： " << init_car_state.speed << endl;
}

/* 接收车的上一帧控制信息，完成当前帧的规划*/
void Dynamic_routing::control_call_back(const geometry_msgs::Vector3 &msg)
{
  init_car_state.acc = msg.x;
  init_car_state.kappa = tan(msg.y) / 2.7;
}

//---------------------------------默认构造函数：规划函数,初始化参数---------------------------------//
Dynamic_routing::Dynamic_routing(void)
{
  ros::NodeHandle n;

  // 参数初始化
  is_set_start = false;
  is_set_goal = false;
  is_set_refline = false;

  ////////////////////////////////发布////////////////////////////////////////
  global_waypoints_pub_ = n.advertise<visualization_msgs::MarkerArray>("/lyn/planning/global_waypoints", 2);       // 发布全局轨迹
  local_waypoints_pub_ = n.advertise<visualization_msgs::MarkerArray>("/lyn/planning/local_waypoints", 2); // 发布局部轨迹

  local_paths_a = n.advertise<geometry_msgs::PoseArray>("/lyn/planning/dynamic_paths_a", 2); // 发布局部轨迹的加速度
  local_paths_t = n.advertise<geometry_msgs::PoseArray>("/lyn/planning/dynamic_paths_t", 2); // 发布局部轨迹的航向角
  local_paths_k = n.advertise<geometry_msgs::PoseArray>("/lyn/planning/dynamic_paths_k", 2); // 发布局部轨迹的曲率
  local_paths_s = n.advertise<geometry_msgs::PoseArray>("/lyn/planning/dynamic_paths_s", 2); // 发布局部轨迹的速度
  local_paths_trj = n.advertise<waypoint_msgs::WaypointArray>("/lyn/planning/dynamic_waypoints", 2);  // 发布局部轨迹点
  Start_Dynamic = n.advertise<std_msgs::String>("/lyn/planning/Start_Dynamic", 2);           // 发布局部轨迹成功生存的信号
  // cur_goal =  n.advertise<geometry_msgs::Vector3>("/lyn/planning/cur_goal", 2);    //发布当前行驶的终点

  start_pose_subscriber_ = n.subscribe("lyn/car/car_start_pose", 10, &Dynamic_routing::start_pose_call_backs, this);         // 订阅
  goal_pose_subscriber_ = n.subscribe("lyn/car/car_goal_pose", 10, &Dynamic_routing::goal_pose_call_backs, this);         // 订阅

  odom_sub_ = n.subscribe("/lyn/odom", 10, &Dynamic_routing::odom_call_back, this);
  control_sub_ = n.subscribe("/lyn/car/control_car",  10, &Dynamic_routing::control_call_back, this);           // 订阅控制信息

  string aa = "0";
  start_dynamic.data = aa.c_str();
  Start_Dynamic.publish(start_dynamic);

  sleep(0.5);
  routing_thread_ = new boost::thread(boost::bind(&Dynamic_routing::thread_routing, this));
}

Dynamic_routing::~Dynamic_routing(void)
{
  delete routing_thread_;
}

void Dynamic_routing::thread_routing(void)
{
  ros::NodeHandle n;
  ros::Rate loop_rate(2);
  // 障碍物对象
  Obstacles obs = Obstacles();
  // sleep(2);
  int frame_count = 0;
  while (n.ok())
  {
    auto start_time_ros = std::chrono::high_resolution_clock::now();
    if (is_set_start && is_set_goal && (!is_set_refline) && (!is_reach_goal))
    {
      ROS_WARN("Try global planning!");
      //--------------------------------------------------全局轨迹生成--------------------------------------------------//
      //保留生成接口，连接起点和终点位姿，这里直接从文件读入
      SmoothConfig config_;
      std::vector<Eigen::Vector2d> raw_points_;
      config_.max_deviation = 0.1;
      config_.max_iteration = (1000);
      config_.opt_tol = (1.0e-6);
      config_.opt_acceptable_tol = (1.0e-4);
      config_.weight_curve_length = (1.0);
      config_.weight_kappa = (1.0);
      config_.weight_dkappa = (100.0);
      config_.resolution = 0.1;

      // 打开输入文件流
      std::ifstream inFile("/home/lynnn/test_ws/globalpoints_data.txt");
  
      // 检查文件是否成功打开
      if (!inFile) {
          std::cerr << "无法打开文件！" << std::endl;
          return;
      }
  
      std::string line;
      // 逐行读取文件内容
      while (std::getline(inFile, line)) {
          // 使用 std::stringstream 来分割每行的数据
          std::stringstream ss(line);
          double x, y;
          
          // 从字符串流中读取 x 和 y 坐标
          if (ss >> x >> y) {
              // 将读取到的坐标存储到 Eigen::Vector2d 中，并添加到 vector 中
            raw_points_.emplace_back(x, y);
          } else {
              std::cerr << "文件格式错误，无法读取坐标：" << line << std::endl;
          }
      }
      // std::reverse(raw_points_.begin(), raw_points_.end());
      // cout << "raw" <<  raw_points_[0](0) << endl;
      // cout << "raw" <<  raw_points_[0](1) << endl;
  
      // 关闭文件流
      inFile.close();
      CoarsePathGenerator spiral_smoother(config_);
      int res = spiral_smoother.SmoothStandAlone(raw_points_, &refline.theta, &refline.kappa,
                                                &refline.dkappa, &refline.s, &refline.x, &refline.y);
      cout << "solve finished" << endl;
      refline.generateAllS();
      std::vector<GlobalPathPoint> smoothed_point2d = spiral_smoother.Interpolate(refline.theta, refline.kappa, refline.dkappa, refline.s, refline.x, refline.y, config_.resolution); //插值函数，对refline采样，便于可视化
      // std::vector<GlobalPathPoint> smoothed_point2d;
      // for(int i = 0; i < refline.x.size(); ++i) {
      //   smoothed_point2d.push_back(GlobalPathPoint(refline.theta[i], refline.kappa[i], refline.dkappa[i], refline.s[i], refline.x[i], refline.y[i]));
      // }
      // cout << "行香蕉" <<  smoothed_point2d[smoothed_point2d.size() - 1].theta << endl;
      if (res > 0) {
        is_set_refline = true;
        publishPathMarker(smoothed_point2d);
      }
    }

    if(is_set_refline) {
      // if(hypot(init_car_state.x - goal_pose_.x, init_car_state.y - goal_pose_.y) < 0.1) {
      if(hypot(init_car_state.x - refline.x.back(), init_car_state.y - refline.y.back()) < 0.1) {
        //结束，不能放在if外，会导致一开始就到达了终点
        ROS_WARN("heading goal arrived!");
        
        //泊车轨迹规划
        //读入参考轨迹

        //轨迹优化

        //轨迹pub

        
        // string aa = "0";
        // start_dynamic.data = aa.c_str();
        // Start_Dynamic.publish(start_dynamic); 
        is_reach_goal = true;
      }
      if ( !is_reach_goal) {
        //--------------------------------------------------局部轨迹生成--------------------------------------------------//
        // ROS_WARN("try local traj generate");
        //开始计时
        auto start_time = std::chrono::high_resolution_clock::now();
        if(best_path.size() > 0) {
              init_car_state.x = best_path[5].x;
              init_car_state.y = best_path[5].y;
              init_car_state.speed = best_path[5].speed;
              init_car_state.theta = best_path[5].theta;
              init_car_state.kappa = best_path[5].kappa;
              init_car_state.acc = best_path[5].acc;
        }
        
        const vector<CartesianState> last_path = best_path;
        DpPlanner dp_planner = DpPlanner(refline, init_car_state, obs, frame_count, best_path);
        dp_planner.DynamicProgramming();
        best_path = dp_planner.getBestPath();
        ref_path = dp_planner.getRefPath();
        ROS_WARN("finish dp planner");

        Mpc npmc_opt(frame_count);
        npmc_opt.solve(init_car_state, ref_path, best_path, obs);
        best_path = npmc_opt.getFinalPath();

        // ROS_WARN("current frame_count: %d", frame_count);
        // TrajPlanner df_opt;
        // vector<CartesianState> new_path;
        // if(df_opt.Run(init_car_state, obs, refline, frame_count, best_path, ref_path, last_path, new_path)) {
        //   best_path.resize(new_path.size());
        //   best_path = new_path;
        // }

        // if(frame_count > 49)
        //   is_reach_goal = true; //单帧测试用
        ROS_WARN("finish dynamic planner");
        // 获取结束时间点
        auto end_time = std::chrono::high_resolution_clock::now();
        // 计算时间间隔
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        std::chrono::duration<double> elapsed_seconds2 = end_time - start_time_ros;
        // 输出时间间隔
        std::cout << "程序执行时间: " << elapsed_seconds2.count() << " 秒" << std::endl;
        ++frame_count;
        if(elapsed_seconds2.count() < 0.5 - 0.001) {
          cout << "process sleep" <<endl;
          ros::Duration(0.5 - 0.001 - elapsed_seconds2.count()).sleep();
        }

        //测试用bestpath
        // CartesianState newstate = init_car_state;
        // newstate.acc = 0.2;
        // newstate.kappa = tan(0.174) / 2.7; //10度
        // for(int i = 0; i < 30; ++i) {
        //   double  dt = 0.1;
        //   double delta_x = newstate.speed * cos(newstate.theta) * dt;
        //   double delta_y = newstate.speed * sin(newstate.theta) * dt;
        //   double delta_speed = newstate.acc * dt;
        //   double delta_theta = (newstate.speed * tan(0.174) / 2.7) * dt;
        //   // std::cout << "dt:" << dt << "\n";
        //   newstate.x += delta_x;
        //   newstate.y += delta_y;
        //   newstate.theta += delta_theta;
        //   newstate.speed += delta_speed;
        //   best_path.emplace_back(newstate);
        // }
        
        //---------------------------------发布轨迹---------------------------------//
          waypoint_msgs::WaypointArray local_waypoints;
          local_waypoints.header.frame_id = Frame_id;
          local_path_array.markers.resize(best_path.size());
          for (int i = 0; i < best_path.size(); i++)
          {
            waypoint_msgs::Waypoint point;
            point.pose.pose.position.x = best_path[i].x;
            point.pose.pose.position.y = best_path[i].y;
            point.twist.twist.linear.x = best_path[i].speed;
            point.twist.twist.linear.y = 0;
            local_waypoints.waypoints.emplace_back(point);

            visualization_msgs::Marker marker;
            marker.header.stamp = ros::Time::now();
            marker.header.frame_id = "map";
            marker.ns = "path";
            marker.lifetime = ros::Duration(0.6); // Marker将在5秒后自动消失
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = best_path[i].x;
            marker.pose.position.y = best_path[i].y;
            marker.pose.position.z = 0.3;
            marker.pose.orientation.w = 1.0;  // Quaternion representing no rotation
            marker.scale.x = 0.4;
            marker.scale.y = 0.4;
            marker.scale.z = 0.4;
            marker.color.a = 1.0;  // Alpha (opacity)
            marker.color.r = 1.0;  // Red
            marker.color.g = 0.0;  // Green
            marker.color.b = 0.0;  // Blue
            local_path_array.markers[i] = (marker);
    }
          // Publish the marker array
          local_waypoints_pub_.publish(local_path_array);
          local_paths_trj.publish(local_waypoints);

          //--------------------------------发布加速度---------------------------------//
          pubLocalPath_a.poses.clear();
          pubLocalPath_a.header.frame_id = Frame_id;
          pubLocalPath_a.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].acc;
            //  init_pose.position.x = 0.2;
            pubLocalPath_a.poses.push_back(init_pose);
          }
          local_paths_a.publish(pubLocalPath_a);
          //  //--------------------------------发布速度---------------------------------上述waypoints中已经包含速度信息，这里注掉发布速度的部分//   
          // pubLocalPath_s.poses.clear();
          // pubLocalPath_s.header.frame_id = Frame_id;
          // pubLocalPath_s.header.stamp = ros::Time::now();
          // for (size_t i = 0; i < best_path.size(); i++)
          // {
          //   geometry_msgs::Pose init_pose;
          //   init_pose.position.x = best_path[i].speed;
          //   pubLocalPath_s.poses.push_back(init_pose);
          // }
          // local_paths_s.publish(pubLocalPath_s);
          //--------------------------------发布角度---------------------------------//
          pubLocalPath_t.poses.clear();
          pubLocalPath_t.header.frame_id = Frame_id;
          pubLocalPath_t.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].theta;
            pubLocalPath_t.poses.push_back(init_pose);
          }
          local_paths_t.publish(pubLocalPath_t);
          //--------------------------------发布曲率---------------------------------//
          pubLocalPath_k.poses.clear();
          pubLocalPath_k.header.frame_id = Frame_id;
          pubLocalPath_k.header.stamp = ros::Time::now();
          for (size_t i = 0; i < best_path.size(); i++)
          {
            geometry_msgs::Pose init_pose;
            init_pose.position.x = best_path[i].kappa;
            // init_pose.position.x = 0;
            pubLocalPath_k.poses.push_back(init_pose);
          }
          local_paths_k.publish(pubLocalPath_k);
          string aa = "1";
          start_dynamic.data = aa.c_str();
          Start_Dynamic.publish(start_dynamic); // 轨迹生成成功，发送move信号

        // 输出时间间隔
        auto end_time3 = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds3 = end_time3 - start_time_ros;
        std::cout << "总的程序执行时间: " << elapsed_seconds3.count() << " 秒" << std::endl;
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}