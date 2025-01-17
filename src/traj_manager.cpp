#include "df_planner/traj_manager.h"
#include <cmath>
#include <utility>
#include "df_planner/constants.h"
using namespace plan_manage;
bool TrajPlanner::Run(
    const CartesianState& init_state,
    Obstacles& obs,
    GlobalPath& refline,
    const int count,
    const vector<CartesianState>& source_path, 
    const vector<GlobalPathPoint>& ori_path, 
    const vector<CartesianState>& last_path, 
    vector<CartesianState>& final_result) {
  
  final_result.clear();
  obs_ = obs;
  frame_count = count;
  traj_opt_.setParam();
  traj_opt_.SetDebugMode(is_debug_mode_);
  if (source_path.empty()) {
    ROS_WARN("DF planner illealInput");
    return false;
  }

  CartesianState end_state = source_path[source_path.size() - 1];
  if(std::isnan(end_state.acc)) end_state.acc = 0.0;
  if(std::isnan(end_state.kappa)) end_state.kappa = 0.0;
  cout << "df_init_state: " << init_state.x << ", " << init_state.y << ", " << init_state.theta << ", " << init_state.speed << ", " << init_state.acc << ", " << atan(2.7 * init_state.kappa) << endl;
  cout << "df_end_state: " << end_state.x << ", " << end_state.y << ", " << end_state.theta << ", " << end_state.speed << ", " << end_state.acc << ", " << atan(2.7 * init_state.kappa) << endl;

  traj_utils::FlatTrajData trajs;
  
  vector<CartesianState> ref_states = source_path;
  int max_inter = 1;
  double duration = 6.0;
  // ROS_WARN("DF planner set_init_param finished");
  
  //   ROS_WARN("DF planner cal_key_point finished");
  
  if (!CalExtremePoint(init_state, end_state, &trajs)) {
      ROS_WARN("DF planner illealInput");
      return false;
    }

  double last_cost = 1e6;
  while(max_inter--) {

    if (!CalKeyPoint(ref_states, &trajs, duration)) {
      ROS_WARN("DF planner illealInput");
      return false;
    }
    //每个时刻不止一个障碍
    std::vector<std::vector<Eigen::Vector3d>> rb_hPolys(ref_states.size());
    std::vector<std::vector<Eigen::Vector3d>> fb_hPolys(ref_states.size());
    std::vector<float> ref_states_para_front = FrontPos(ref_states);
    double dt_ = trajs.duration / (traj_opt_.piece_ * traj_opt_.corridor_per_piece );
    for (int i = 0; i < ref_states.size(); ++i)
    {
        // for j = 0; j < obs_num; ++j
        for(auto j : obs_.getMoveInds()) { 
            //获取障碍物位置
            //测试用，只有一个动态障碍
            if(init_state.x > 60 && j == 9) continue;
            if( j == 10) continue;
            auto cur_obs = obs.getObs()[j];
            Eigen::MatrixXd poly(2, cur_obs.vertex_x.size());
            for (int k = 0; k < cur_obs.vertex_x.size(); ++k) {
                poly(0, k) = cur_obs.vertex_x[k];
                poly(1, k) = cur_obs.vertex_y[k];
            }
            poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.5 *  frame_count + dt_ * i); 
            
            double margin =sqrt(2); //安全距离
            //后轴约束
            Eigen::VectorXd alpha;
            double beta, d;
            Eigen::Vector2d ref_pos(ref_states[i].x, ref_states[i].y);
            d2poly(ref_pos, poly,alpha, beta, d);  // 计算到多边形的距离
            rb_hPolys[i].emplace_back( Vector3d(alpha(0), alpha(1), beta) );

            // //前轴约束
            ref_pos  = Vector2d(ref_states_para_front[2 * i], ref_states_para_front[2 * i + 1]);
            // cout << "ref_pos:" << ref_pos(0) << "," << ref_pos(1) << endl;
            d2poly(ref_pos, poly, alpha, beta, d); // 计算到多边形的距离
             fb_hPolys[i].emplace_back( Vector3d(alpha(0), alpha(1), beta) );
        }
    }
    auto start_time = std::chrono::high_resolution_clock::now();
    if (!traj_opt_.OptimizeTrajectory(trajs, refline, ori_path, last_path, rb_hPolys, fb_hPolys)) {
      auto end_time = std::chrono::high_resolution_clock::now();
      // 计算时间间隔 
      std::chrono::duration<double> elapsed_seconds = end_time - start_time;
      // 输出时间间隔
      ROS_WARN( "a DF problem solve time: %f s" ,  elapsed_seconds.count() );
      ROS_WARN("DF planner opt failed!");
      return false;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    // 计算时间间隔 
    std::chrono::duration<double> elapsed_seconds = end_time - start_time;
    // 输出时间间隔
    ROS_WARN( "a DF problem solve time: %f s" ,  elapsed_seconds.count() );

    auto opt_path = traj_opt_.GetResult();
    double diff = 0;
    for (int i = 0; i < opt_path.size(); ++i) {
        diff += pow(opt_path[i].x - ref_states[i].x, 2) + pow(opt_path[i].y - ref_states[i].y, 2);
    }
    cout << "diff: " << sqrt(diff) << endl;
    cout << "diff_cost: " << std::abs(last_cost - traj_opt_.final_cost_) << endl;
    if(sqrt(diff) < 1.0 || std::abs(last_cost - traj_opt_.final_cost_) < 1e-6) {
      ROS_WARN("DF planner opt success!");
      final_result = traj_opt_.GetResult(0.1);
      plotFinalPath(final_result);
      return true;
    }
    ref_states = opt_path;
    duration = traj_opt_.getResultDuration();
    last_cost =  traj_opt_.final_cost_;
  }
  
  ROS_WARN("DF planner arrived max iter");
  // return false;

   //测试用
  final_result = traj_opt_.GetResult(0.1);
  plotFinalPath(final_result);
  return true; 
}

bool TrajPlanner::CalKeyPoint(const vector<CartesianState>& source_path,
                              traj_utils::FlatTrajData* trajs, double duration) {
  //piece点
  const int source_path_size = source_path.size();
  if (source_path_size < 2) {
    return false;
  }
  auto& inner_p = trajs->inner_pts;
  inner_p.resize(traj_opt_.piece_ - 1);
  for (int i = 1; i < traj_opt_.piece_; i++) {
    cout << "inner_pt: " << source_path[i * traj_opt_.corridor_per_piece].x << ", " << source_path[i * traj_opt_.corridor_per_piece].y << endl;
    inner_p[i - 1] << source_path[i * traj_opt_.corridor_per_piece].x, source_path[i * traj_opt_.corridor_per_piece].y;
  }
  // double delta_x = (source_path[source_path.size()].x - source_path[0].x) /  traj_opt_.piece_ ;
  // double delta_y = (source_path[source_path.size()].y - source_path[0].y) /  traj_opt_.piece_;
  // for (int i = 1; i < traj_opt_.piece_; i++) {
  //   cout << "inner_pt: " << source_path[i * traj_opt_.corridor_per_piece].x << ", " << source_path[i * traj_opt_.corridor_per_piece].y << endl;
  //   inner_p[i - 1] << source_path[0].x + i * delta_x, source_path[0].y + i * delta_y;
  // }

  //inner_collision点，是用来算凸走廊的，换成CFS不是很需要、
  // const int corridor_size_total =
  //     traj_opt_.piece_ * traj_opt_.corridor_per_piece;
  // auto& corridor_pts = trajs->corridor_pts;
  // corridor_pts.resize(corridor_size_total);
  // for (int i = 0; i < corridor_size_total; i++) {
  //   corridor_pts[i] <<  source_path.x,  source_path.y(),  source_path.theta();
  // }

  trajs->duration = duration; //总时间6s
  return true;
}

bool TrajPlanner::CalExtremePoint(const CartesianState& init_state,
                                                                          const CartesianState& end_node,
                                                                          traj_utils::FlatTrajData* trajs) {
  double st_lat_acc = init_state.kappa * init_state.speed * init_state.speed;
  if (st_lat_acc > 4.0) st_lat_acc = 4.0;
  double end_lat_acc = end_node.kappa * end_node.speed * end_node.speed;
  if (end_lat_acc > 4.0) end_lat_acc = 4.0;
 
  Eigen::Vector2d init_control(st_lat_acc, init_state.acc);
  Eigen::Vector2d end_control(end_lat_acc, end_node.acc);
  auto& start_state = trajs->start_state;
  auto& end_state = trajs->final_state;
  GetFlatState(
      {init_state.x, init_state.y, init_state.theta, init_state.speed},
                init_control, start_state);  // control是横摆角和纵向加速度
  GetFlatState({end_node.x, end_node.y, end_node.theta, end_node.speed},
               end_control, end_state); //原end_node.acc处为0
  return true;                                                                          
}

// 将普通的a*轨迹转化成微分平坦状态，也就是xy
void TrajPlanner::GetFlatState(const Eigen::Vector4d& state,
                               const Eigen::Vector2d& control_input,
                               Eigen::MatrixXd& flat_state) {
  flat_state.resize(2, 3);  // 2行三列
  const double theta = state(2);
  const double speed = state(3);  // vel > 0
  const double sin_theta = std::sin(theta);
  const double cos_theta = std::cos(theta);

  // 第一列是位置，第二列是速度，第三列是加速度
  // 纵向加速度和向心加速度，再乘以旋转矩阵
  flat_state.col(0) << state.head(2);
  flat_state.col(1) << cos_theta * speed, sin_theta * speed;
  flat_state.col(2) << cos_theta * control_input(1) -
                           sin_theta * control_input(0),
      sin_theta * control_input(1) + cos_theta * control_input(0);
  // if(std::abs(flat_state.col(1)) < 1e-6)  flat_state.col(1) = 0.0;
  // if(std::abs(flat_state.col(2)) < 1e-6)  flat_state.col(2) = 0.0;
}