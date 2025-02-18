#include "global_routing.hpp"

/*读回ros odom坐标系数据 , 接收车的里程信息，控制车的移动*/
void GlobalRouting::odom_call_back(const nav_msgs::Odometry &msg)
{
  car_odom_ = msg; // 车的里程信息，就是位置信息

  // 坐标转换
  geometry_msgs::Quaternion odom_quat = msg.pose.pose.orientation;
  tf::Quaternion quat;
  tf::quaternionMsgToTF(odom_quat, quat);

  // 根据转换后的四元数，获取roll pitch yaw
  double roll, pitch, yaw;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

  cur_pose.x = msg.pose.pose.position.x;
  cur_pose.y = msg.pose.pose.position.y;
  cur_pose.yaw = yaw;

  cur_pose.vx = msg.twist.twist.linear.x;
  cur_pose.vy = msg.twist.twist.linear.y;
  cur_pose.v = std::sqrt(cur_pose.vx * cur_pose.vx + cur_pose.vy * cur_pose.vy);

  cur_pose.z = msg.pose.pose.position.z;
}

/*获取局部规划来的航向角*/
void GlobalRouting::Vehival_Theta(const geometry_msgs::PoseArray &theta)
{
  thetas.clear();
  if (theta.poses.size() > 0)
  {
    for (int i = 0; i < theta.poses.size(); i++)
    {
      double x = theta.poses[i].position.x;
      thetas.push_back(x);
    }
  }
}

/*获取局部规划来的曲率*/
void GlobalRouting::Vehival_Kappa(const geometry_msgs::PoseArray &kappa)
{
  simulation_count_ = 0;
  kappas.clear();
  if (kappa.poses.size() > 0)
  {
    for (int i = 0; i < kappa.poses.size(); i++)
    {
      double x = kappa.poses[i].position.x;
      kappas.push_back(x);
    }
  }
}

/*获取局部规划来的加速度*/
void GlobalRouting::Vehival_Acc(const geometry_msgs::PoseArray &acc)
{
  accs.clear();
  if (acc.poses.size() > 0)
  {
    for (int i = 0; i <acc.poses.size(); i++)
    {
      double x = acc.poses[i].position.x;
      accs.push_back(x);
    }
  }
}

/*获取局部规划来的速度*/
void GlobalRouting::Vehival_Speed(const geometry_msgs::PoseArray &speed)
{
  speeds.clear();
  if (speed.poses.size() > 0)
  {
    for (int i = 0; i <speed.poses.size(); i++)
    {
      double x = speed.poses[i].position.x;
      speeds.push_back(x);
    }
  }
}

/*获取局部轨迹的信号*/
void GlobalRouting::Vehival_Go(const std_msgs::String::ConstPtr &go)
{
  Start_dynamic = go->data.c_str();
}

/*获取局部轨迹*/
void GlobalRouting::Vehival_Traj(const waypoint_msgs::WaypointArray::ConstPtr &msg)
{
  dynamic_points.poses.clear();
  dynamic_points.header.frame_id = Frame_id;
  dynamic_points.header.stamp = ros::Time::now();
  // speeds.clear();
  local_waypoints.resize(msg->waypoints.size());

  for (int i = 0; i < msg->waypoints.size(); i++)
  {
    // 为carla的控制模块准备变量
    car_state temp_point;
    temp_point.x = msg->waypoints[i].pose.pose.position.x;
    temp_point.y = msg->waypoints[i].pose.pose.position.y;
    temp_point.v = msg->waypoints[i].twist.twist.linear.x; //速度
    local_waypoints[i] = temp_point;
    //
    geometry_msgs::PoseStamped pose_stamp;
    pose_stamp.header.frame_id = Frame_id;
    pose_stamp.header.stamp = ros::Time::now();
    pose_stamp.pose.position.x = msg->waypoints[i].pose.pose.position.x;
    pose_stamp.pose.position.y = msg->waypoints[i].pose.pose.position.y;
    pose_stamp.pose.position.z = 0;
    dynamic_points.poses.push_back(pose_stamp);
    speeds.push_back(msg->waypoints[i].twist.twist.linear.x); //速度
  }
}

/*发布车辆在路径的起始位置,传入起点*/
void GlobalRouting::publish_car_start_pose(const geometry_msgs::Pose &start_pose)
{
  geometry_msgs::Vector3 state;
  // get 起始位置的yaw
  // double set_vehicle_yaw = tf::getYaw(start_pose.orientation);
  // get vehicle start pose
  state.x = start_pose.position.x;
  state.y = start_pose.position.y;
  state.z = 0; //-M_PI/2;//-0.837; //-2.291536; //-M_PI / 2.0;   0.850056
  // 发布车的起点位置
  vehicle_start_pose_pub_.publish(state);
}

/*发布车辆的终点位置,传入终点*/
void GlobalRouting::publish_car_goal_pose(const geometry_msgs::Pose &goal_pose)
{
  geometry_msgs::Vector3 state;
  // get 起始位置的yaw
  // double set_vehicle_yaw = tf::getYaw(start_pose.orientation);
  // get vehicle start pose
  state.x = goal_pose.position.x;
  state.y = goal_pose.position.y;
  state.z = 0; //-M_PI / 2.0;
  // 发布车的起点位置
  vehicle_goal_pose_pub_.publish(state);
}

// 判断车是否到了路径的尽头，传入 终点，车里程
bool GlobalRouting::Vehical_Stop(const geometry_msgs::Pose &goal_point, nav_msgs::Odometry &odom)
{
  // 获取车的里程位置
  double x = odom.pose.pose.position.x;
  double y = odom.pose.pose.position.y;
  // 获取路径点的最后一个位置
  double xt = goal_point.position.x;
  double yt = goal_point.position.y;
  // 判断如果两点坐标接近

  double dis = sqrt((x - xt) * (x - xt) + (y - yt) * (y - yt));
  if (dis < goal_distanse)
  {
    return true;
  }
  return false;
}

/*默认构造函数：规划函数,初始化参数*/
GlobalRouting::GlobalRouting()
{
    ros::NodeHandle n;
     // 参数获取
    goal_distanse = 0.1;
    goal_pose_.position.x = 58.6425 ; 
    goal_pose_.position.y = 28.3;
    start_pose_.position.x = 30.2; //14.37;//35;//14.58 ;   //14.38 70.9
    start_pose_.position.y = 9.9; //70.9;//64.93;//69.0;
    obs = Obstacles();

     // 回调
    odom_sub_ = n.subscribe("/lyn/odom", 10, &GlobalRouting::odom_call_back, this);
    
     //发布
    control_data_pub_ = n.advertise<geometry_msgs::Vector3>("/lyn/car/control_car", 10);                      // ros仿真时发布车的控制数据
    vehicle_start_pose_pub_ = n.advertise<geometry_msgs::Vector3>("lyn/car/car_start_pose", 10);             // 发布车的起点位置
    vehicle_goal_pose_pub_ = n.advertise<geometry_msgs::Vector3>("lyn/car/car_goal_pose", 10);             // 发布车的终点位置
    map_points_pub_ = n.advertise<visualization_msgs::MarkerArray>("/lyn/maps/map_points", 10);               // ros仿真下的地图路线点显示
    obs_pub_ = n.advertise<visualization_msgs::MarkerArray>("obs_polygons", 10);

    MatrixXd poly(2, 4);
    poly.row(0) << 140, 0, 0, 140;
    poly.row(1) << 0, 0, 80, 80;
     visualization_msgs::Marker marker;
    Turn_obstacles_into_squares(marker, poly, 0);
    obstacle_MarkerArray.markers.emplace_back(marker);
    for (int j = 0; j <  obs.getNum() - obs.getMoveInds().size(); ++j) {
      visualization_msgs::Marker marker;
      auto cur_obs = obs.getObs()[j];
      MatrixXd poly(2, cur_obs.vertex_x.size());
      for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
        poly(0, i) = cur_obs.vertex_x[i];
        poly(1, i) = cur_obs.vertex_y[i];
      }
      Turn_obstacles_into_squares(marker, poly,j+1);
      obstacle_MarkerArray.markers.emplace_back(marker);
      //obs_pub.publish(obstacle_MarkerArray);
    }
    MovingObs::num = 3;
		MovingObs::obs_traj.resize(3);
    MovingObs::obs_traj[0] = MovingObs::loadTrajectoryData("/home/lynnn/test_ws/opposite1.csv");
    MovingObs::obs_traj[1] = MovingObs::loadTrajectoryData("/home/lynnn/test_ws/following1.csv");
    MovingObs::obs_traj[2] = MovingObs::loadTrajectoryData("/home/lynnn/test_ws/parking1.csv");

        // obs_traj[1] = loadTrajectoryData("following.csv");
    

    // 打印控制器
    //  ROS_WARN("Controller: lqr");
    controller = "simulation";
    ROS_WARN("Controller: simulation");

    // 局部规划订阅
    vehival_theta = n.subscribe("/lyn/planning/dynamic_paths_t", 10, &GlobalRouting::Vehival_Theta, this);
    vehival_kappa = n.subscribe("/lyn/planning/dynamic_paths_k", 10, &GlobalRouting::Vehival_Kappa, this);
    vehival_traj = n.subscribe("/lyn/planning/dynamic_waypoints", 10, &GlobalRouting::Vehival_Traj, this);
    vehival_acc = n.subscribe("/lyn/planning/dynamic_paths_a", 10, &GlobalRouting::Vehival_Acc, this);
    vehival_speed = n.subscribe("/lyn/planning/dynamic_paths_v", 10, &GlobalRouting::Vehival_Speed, this);
    vehival_go = n.subscribe("/lyn/planning/Start_Dynamic", 10, &GlobalRouting::Vehival_Go, this);

    // 初始化 标志位：
    is_start_pose_set = true;
    is_goal_pose_set = true;
    is_vehical_stop_set = true;
    Start_dynamic = "0";   //测试改为1，实际应该是0

    // 车的初始值设置
    car_speed = 0.1;

    // 创建线程
    routing_thread_ = new boost::thread(boost::bind(&GlobalRouting::thread_routing, this));
}

/*析构函数：释放线程*/
GlobalRouting::~GlobalRouting()
{
  delete routing_thread_;
}

// 开车，控制车的行使
void GlobalRouting::thread_routing()
{
  double frequency = 10.0; //控制频率0.1s，规划频率考虑0.5s
  ros::NodeHandle n;
  ros::Rate loop_rate(frequency);
  double turn_angle = 0, Tc = 0;
  int out_index_ = 0;
  sleep(5);
  publish_car_start_pose(start_pose_);
  publish_car_goal_pose(goal_pose_);

 int frame_count = 0;
  while (n.ok())
  {
    //发布障碍可视化
    //   if(obstacle_MarkerArray.markers.size() > (obs.getNum() -  obs.getMoveInds().size() + 1))
    //     obstacle_MarkerArray.markers.resize(obs.getNum() -  obs.getMoveInds().size() + 1);
    //   for(auto& j : obs.getMoveInds()) { 
    //     // cout << "moving ind: " << j << endl;
    //     visualization_msgs::Marker marker;
    //     auto cur_obs = obs.getObs()[j];
    //     MatrixXd poly(2, cur_obs.vertex_x.size());
    //     for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
    //       poly(0, i) = cur_obs.vertex_x[i];
    //       poly(1, i) = cur_obs.vertex_y[i];
    //     }
    //     poly += cur_obs.speed * MatrixXd::Ones(1, 4) * 0.1 *  frame_count; 
    //     Turn_obstacles_into_squares(marker, poly, j + 1);
    //     obstacle_MarkerArray.markers.emplace_back(marker);
    //   }
      
      // if(obstacle_MarkerArray.markers.size() > (obs.getNum() -  MovingObs::num + 1))
      if(obstacle_MarkerArray.markers.size() > 10)
          obstacle_MarkerArray.markers.resize(10);
        int ind = 10;
        for(auto& traj : MovingObs::obs_traj) { 
          // cout << "moving ind: " << j << endl;
          visualization_msgs::Marker marker;
          if(frame_count < traj.trajs.size()) {
            auto& cur_obs = traj.trajs[frame_count];
            Turn_moving_obstacles_into_squares(marker, cur_obs, ind);
          }
          else {
            auto& cur_obs =  traj.trajs.back();
           Turn_moving_obstacles_into_squares(marker, cur_obs, ind);
          }
          obstacle_MarkerArray.markers.emplace_back(marker);
          ind++;
        }
      
    // Start_dynamic 局部规划轨迹开始生成
    if (is_start_pose_set == true && is_goal_pose_set == true && Start_dynamic == "1")
    {  
        ++frame_count; //开始仿真运行
        is_vehical_stop_set = Vehical_Stop(goal_pose_, car_odom_); // 判断是否到达终点
        if(controller ==  "simulation" ) {
            // Tc = accs[simulation_count_];
            // // turn_angle = tan(2.7 * kappas[simulation_count_]); //atan(L*kappa)
            // turn_angle = atan2(2.7 * kappas[simulation_count_], 1);
            // msg_ros.x = Tc;           // 加速度
            // msg_ros.y = turn_angle;  //转角
            // msg_ros.z = -1;
            // cout << "当前ind: " << simulation_count_ <<  ",  当前位置: (" << cur_pose.x << ", "  << cur_pose.y << "),  当前控制量： " << Tc  <<  ", " << turn_angle << endl;
            
            if(simulation_count_ < local_waypoints.size()) {
              //不再模拟控制，直接传入坐标
              msg_ros.x = local_waypoints[simulation_count_].x;           // 加速度
              msg_ros.y = local_waypoints[simulation_count_].y;  //转角
              msg_ros.z = thetas[simulation_count_];
            }
            else {
              msg_ros.x = local_waypoints.back().x;           // 加速度
              msg_ros.y = local_waypoints.back().y;  //转角
              msg_ros.z = thetas.back();
            }
            
            // 测试用
            // Tc = 0.2;
            // turn_angle = 0.174;
            
            ++simulation_count_;
        }
        else if (controller ==  "lqr")  {//保留lqr控制器接口
            Lqr.lqr_control(car_odom_, dynamic_points, kappas, thetas, speeds, turn_angle, out_index_);
            car_speed = speeds[out_index_];
            msg_ros.y = turn_angle;           // 转角
            msg_ros.x = car_speed; // ros仿真直接赋值速度
        }
        
        if (is_vehical_stop_set)  {// 到达终点 
          ROS_WARN("Arrive at goal");
          msg_ros.x = 0;
          msg_ros.y = 0;
          msg_ros.z = 0; //停车
          is_start_pose_set = false;
          is_goal_pose_set = false;
          Start_dynamic = "0";
          car_speed = 0;
        }
        control_data_pub_.publish(msg_ros);
    }

    else if (is_start_pose_set == true && is_goal_pose_set == true && Start_dynamic == "2") // 没有轨迹强制停车
    {
        msg_ros.x = 0;
        control_data_pub_.publish(msg_ros);
        car_speed = 0;
    }
    obs_pub_.publish(obstacle_MarkerArray); //障碍发布
    ros::spinOnce();
    loop_rate.sleep();
  }
}