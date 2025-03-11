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
            if(init_state.y < 65 && j == 10) continue;
            // if( j == 10) continue;
            auto cur_obs = obs.getObs()[j];
            Eigen::MatrixXd poly(2, cur_obs.vertex_x.size());
            for (int k = 0; k < cur_obs.vertex_x.size(); ++k) {
                poly(0, k) = cur_obs.vertex_x[k];
                poly(1, k) = cur_obs.vertex_y[k];
            }
            poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.3 *  frame_count + dt_ * i); 
            
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

bool TrajPlanner::RunGlobalOpt(const vector<Vector2d>& raw_pt) {
  ploy_traj_opt_.reset(new PolyTrajOptimizer);
  ploy_traj_opt_->setParam();

  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  double basetime = 0.0, worldtime =  0.0;
  double dense_traj_res = 32, traj_res = 16;

  /*try to merge optimization process*/
  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container,finState_container;

  int num = raw_pt.size();
  vector<int> index_corner(0);
  findCorners(raw_pt, index_corner);
  duration_container.resize(index_corner.size() - 1);

  for(int i = 0;i < index_corner.size() - 1; ++i) { //去掉终点
    double timePerPiece = 1.0;
    traj_utils::FlatTrajData trajs;
    trajs.singul =  i % 2 == 0 ? 1 : -1;
    singul_container.push_back(trajs.singul);

    int ind = index_corner[i], next_ind = index_corner[i+1];
    // double theta_ind = ind == 0 ? 0 : atan2(raw_pt[ind+1](1) - raw_pt[ind](1), raw_pt[ind+1](0) - raw_pt[ind](0));
    // double theta_next_ind = (next_ind == num-1) ? -M_PI / 2 : atan2(raw_pt[next_ind+1](1) - raw_pt[next_ind](1), raw_pt[next_ind+1](0) - raw_pt[next_ind](0));
    // CartesianState init_state(raw_pt[ind](0), raw_pt[ind](1), theta_ind, 0.0, 0.0, 0.0);
    // CartesianState end_state(raw_pt[next_ind](0), raw_pt[next_ind](1), theta_next_ind, 0.0, 0.0, 0.0);
    CartesianState init_state(raw_pt[ind](0), raw_pt[ind](1), 0.0, 0.0, 0.0, 0.0);
    CartesianState end_state(raw_pt[next_ind](0), raw_pt[next_ind](1), 0.0, 0.0, 0.0, 0.0);
    double theta_ind  = atan2(raw_pt[ind+1](1) - raw_pt[ind](1), raw_pt[ind+1](0) - raw_pt[ind](0));
    double theta_next_ind =atan2(raw_pt[next_ind](1) - raw_pt[next_ind - 1](1), raw_pt[next_ind](0) - raw_pt[next_ind-1](0));
    if(trajs.singul == 1)  { //前进 
      init_state.speed = 0.05;
      init_state.theta = theta_ind;
      end_state.speed = 0.05;
      end_state.theta = theta_next_ind;
    }
    else {
      init_state.speed = -0.05;
      init_state.theta = theta_ind <= 0 ? theta_ind + M_PI : theta_ind - M_PI;
      end_state.speed = -0.05;
      end_state.theta = theta_next_ind <= 0 ? theta_next_ind + M_PI : theta_next_ind - M_PI;
      //原范围：(-pi, pi]
    }

    // if(i == index_corner.size() - 2) {
    //   double theta_next_ind = (next_ind == num-1) ? -M_PI / 2 : atan2(raw_pt[next_ind+1](1) - raw_pt[next_ind](1), raw_pt[next_ind+1](0) - raw_pt[next_ind](0));
    //   end_state = CartesianState(raw_pt[next_ind](0), raw_pt[next_ind](1), theta_next_ind, 0.001, 0.0, 0.0);
    // }

    if (!CalExtremePoint(init_state, end_state, &trajs)) {
        ROS_WARN("DF planner illeal Input");
        return false;
    }

    cout << "init state" << endl;
    cout << trajs.start_state(0,0) << "," <<  trajs.start_state(0,1) << ", " << trajs.start_state(0, 2) << endl;
    cout << trajs.start_state(1,0) << "," <<  trajs.start_state(1,1) << ", " << trajs.start_state(1, 2) << endl;
    cout << "end state" << endl;
    cout << trajs.final_state(0,0) << "," <<  trajs.final_state(0,1) << ", " << trajs.start_state(0, 2) << endl;
    cout << trajs.final_state(1,0) << "," <<  trajs.final_state(1,1) << ", " << trajs.final_state(1, 2) << endl;
    
    int piece_nums;
    double initTotalduration = 0.0;
    // 三次样条拟合
    std::vector<double> x(next_ind - ind +1);
    std::vector<double> y(next_ind - ind +1);
    std::vector<double> s(next_ind - ind +1);
    x[0] = raw_pt[ind](0);
    y[0] = raw_pt[ind](1);
    s[0] = 0;
    double dis = 0.0;
    for(int j = ind; j < next_ind; ++j) {
      x[j - ind + 1] =  raw_pt[j + 1](0);
      y[j - ind + 1] =  raw_pt[j + 1](1);
      dis += hypot(raw_pt[j + 1](0) - raw_pt[j](0), raw_pt[j + 1](1) - raw_pt[j](1));
      s[j - ind + 1] =  dis;
    }
    CubicSpline1D sx, sy;
    sx.Init(s, x);
    sy.Init(s, y);
    
    // 梯形速度规划
    double new_acc = 0.0;
    double acc_t = 0;
    bool is_two = false;
    if( i % 2 == 0 ) {
      new_acc = ploy_traj_opt_->max_forward_acc;
      acc_t  = ploy_traj_opt_->max_forward_vel / new_acc;
      if(pow(ploy_traj_opt_->max_forward_vel, 2) / ploy_traj_opt_->max_forward_acc < dis) {
        initTotalduration = (ploy_traj_opt_->max_forward_vel / ploy_traj_opt_->max_forward_acc + dis / ploy_traj_opt_->max_forward_vel);
      }
      else  {
        initTotalduration =sqrt(dis / ploy_traj_opt_->max_forward_acc);
        acc_t = initTotalduration / 2;
      }
    }
    else {
      new_acc = ploy_traj_opt_->max_backward_acc;
      acc_t  = ploy_traj_opt_->max_backward_vel / new_acc;
     if(pow(ploy_traj_opt_->max_backward_vel, 2) / ploy_traj_opt_->max_backward_acc < dis) {
        initTotalduration = (ploy_traj_opt_->max_backward_vel / ploy_traj_opt_->max_backward_acc + dis / ploy_traj_opt_->max_backward_vel);        
      }
      else  {
        initTotalduration =sqrt(dis / ploy_traj_opt_->max_backward_acc);
        acc_t = initTotalduration / 2;
      }
    }

    piece_nums = std::max(int(initTotalduration / timePerPiece + 0.5),2);
    timePerPiece = initTotalduration / piece_nums; 
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums;
    ego_innerPs.resize(2, piece_nums-1);
    std::vector<Eigen::Vector3d> statelist;
    double res_time = 0;
    for(int j = 0; j < piece_nums; j++ ){
      int resolution;
      if(j==0||j==piece_nums-1){
        resolution = dense_traj_res;
      }
      else{
        resolution = traj_res;
      }
      for(int k = 0; k <= resolution; k++){
        double t = basetime+res_time + 1.0*k/resolution*ego_piece_dur_vec[j];
        // cout << "cur_t: " << t << endl;

        Eigen::Vector3d pos;
        //根据时间计算pos：x\y\theta
        double cur_s;
        if(is_two) {
          cur_s = t > initTotalduration / 2 ? 0.5 * new_acc * t * t : dis - 0.5 * new_acc *(initTotalduration -  t) * (initTotalduration -  t);
        }
        else {
          if(t <= acc_t) {
            cur_s = 0.5 * new_acc * t * t;
          }
          else if(t >= initTotalduration - acc_t) {
            cur_s = dis - 0.5 * new_acc *(initTotalduration -  t) * (initTotalduration -  t);
          }
          else {
            cur_s = 0.5 * new_acc * acc_t * acc_t + acc_t * new_acc * (t - acc_t);
          }
        }
        cout << "cur_s: " << cur_s << endl;

        // ROS_WARN("dis: %f, t: %f, cur_s:%f", dis, t, cur_s);
        Vector2d posx = sx.CalPosition(cur_s);
        Vector2d posy = sy.CalPosition(cur_s);
        pos << posx(0), posy(0), atan2(posy(1) , posx(1));
        statelist.push_back(pos);
        if(k==resolution && j!=piece_nums-1){
          ego_innerPs.col(j) = pos.head(2); 
        }
      } 
      res_time += ego_piece_dur_vec[j];
    }
    // std::cout<<"s: "<<kino_traj.singul<<"\n";
    // double tm1 = ros::Time::now().toSec();
    getRectangleConst(statelist);
    sfc_container.push_back(hPolys_);
    display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(trajs.start_state);
    finState_container.push_back(trajs.final_state);
    // basetime += initTotalduration;
  }

  double t1= ros::Time::now().toSec();
  std::cout<<"try to optimize!\n";
  
  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container, 
                                                      waypoints_container,duration_container, 
                                                      sfc_container,  singul_container,worldtime,0.0);
  std::cout<<"optimize ended!\n";
  double t2 = ros::Time::now().toSec();
  std::cout<<"opt time: "<<(t2-t1)<<std::endl;

  if (flag_success)
  {
      std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
      for(unsigned int i = 0; i < index_corner.size() - 1; i++){
        std::cout<<"init duration: "<<duration_container[i]<<std::endl;
        std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
        std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
        std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost(1)<<std::endl;
        // worldtime = traj_container_.singul_traj.back().end_time;
      }
      // ploy_traj_opt_->GetResult(0.1, refline);
  }
  else{
      ROS_WARN("[PolyTrajManager] Planning fails! ");
      return false;
  }
  return true; 
}

bool TrajPlanner::RunGlobalOpt2(const vector<Vector2d>& raw_pt) {
  ploy_traj_opt_.reset(new PolyTrajOptimizer);
  ploy_traj_opt_->setParam();

  Eigen::VectorXd ego_piece_dur_vec;
  Eigen::MatrixXd ego_innerPs;
  double basetime = 0.0, worldtime =  0.0;
  double dense_traj_res = 32, traj_res = 16;

  /*try to merge optimization process*/
  std::vector<std::vector<Eigen::MatrixXd>> sfc_container;
  std::vector<int> singul_container;
  Eigen::VectorXd duration_container;
  std::vector<Eigen::MatrixXd> waypoints_container;
  std::vector<Eigen::MatrixXd> iniState_container,finState_container;

  int num = raw_pt.size();
  vector<int> index_corner(0);
  findCorners(raw_pt, index_corner);
  duration_container.resize(index_corner.size() - 1);

  for(int i = 0;i < index_corner.size() - 1; ++i) { //去掉终点
    double timePerPiece = 1.0;
    traj_utils::FlatTrajData trajs;
    trajs.singul =  i % 2 == 0 ? 1 : -1;

    int ind = index_corner[i], next_ind = index_corner[i+1];

    CartesianState init_state(raw_pt[ind](0), raw_pt[ind](1), 0.0, 0.0, 0.0, 0.0);
    CartesianState end_state(raw_pt[next_ind](0), raw_pt[next_ind](1), 0.0, 0.0, 0.0, 0.0);
    double theta_ind  = atan2(raw_pt[ind+1](1) - raw_pt[ind](1), raw_pt[ind+1](0) - raw_pt[ind](0));
    double theta_next_ind =atan2(raw_pt[next_ind](1) - raw_pt[next_ind - 1](1), raw_pt[next_ind](0) - raw_pt[next_ind-1](0));
    if(trajs.singul == 1)  { //前进 
      init_state.speed = 0.05;
      init_state.theta = theta_ind;
      end_state.speed = 0.05;
      end_state.theta = theta_next_ind;
    }
    else {
      init_state.speed = -0.05;
      init_state.theta = theta_ind <= 0 ? theta_ind + M_PI : theta_ind - M_PI;
      end_state.speed = -0.05;
      end_state.theta = theta_next_ind <= 0 ? theta_next_ind + M_PI : theta_next_ind - M_PI;
      //原范围：(-pi, pi]
    }

    if (!CalExtremePoint(init_state, end_state, &trajs)) {
        ROS_WARN("DF planner illeal Input");
        return false;
    }

    cout << "init state" << endl;
    cout << trajs.start_state(0,0) << "," <<  trajs.start_state(0,1) << ", " << trajs.start_state(0, 2) << endl;
    cout << trajs.start_state(1,0) << "," <<  trajs.start_state(1,1) << ", " << trajs.start_state(1, 2) << endl;
    cout << "end state" << endl;
    cout << trajs.final_state(0,0) << "," <<  trajs.final_state(0,1) << ", " << trajs.start_state(0, 2) << endl;
    cout << trajs.final_state(1,0) << "," <<  trajs.final_state(1,1) << ", " << trajs.final_state(1, 2) << endl;

    singul_container.push_back(trajs.singul);
    int piece_nums;
    double initTotalduration = 0.0;
    // 三次样条拟合
    std::vector<double> x(next_ind - ind +1);
    std::vector<double> y(next_ind - ind +1);
    std::vector<double> s(next_ind - ind +1);
    x[0] = raw_pt[ind](0);
    y[0] = raw_pt[ind](1);
    s[0] = 0;
    double dis = 0.0;
    for(int j = ind; j < next_ind; ++j) {
      x[j - ind + 1] =  raw_pt[j + 1](0);
      y[j - ind + 1] =  raw_pt[j + 1](1);
      dis += hypot(raw_pt[j + 1](0) - raw_pt[j](0), raw_pt[j + 1](1) - raw_pt[j](1));
      s[j - ind + 1] =  dis;
    }
    CubicSpline1D sx, sy;
    sx.Init(s, x);
    sy.Init(s, y);

    
    piece_nums = std::max(8, (int)(s.back() / 3));
    double s_per_piece = s.back() / piece_nums;
    //均匀分段若分的段的间距较大，也会导致边界处出现龙格现象，因此不进行均匀采样，在首位两端处以较低的分辨率采样
    vector<double> pieces_s(piece_nums, s_per_piece);
    if(s_per_piece > 0.3) {
      std::vector<double> node, weights;
      gaussNodesAndWeights(piece_nums - 1, node, weights); //中间有piece_num-1个点
      reverse(node.begin(), node.end()); //反转
      node.emplace_back(1);
      for(int j = 0; j < piece_nums; ++j) {
        if(j == 0) {
          pieces_s[j] = s.back() * (node[j] - (-1)) / 2;
        }
        else {
          pieces_s[j] = s.back() * (node[j] - node[j-1]) / 2;
        }
      }
    }

    timePerPiece = 1.0;  //s_per_piece
    ego_piece_dur_vec.resize(piece_nums);
    ego_piece_dur_vec.setConstant(timePerPiece);
    duration_container[i] = timePerPiece * piece_nums;
    ego_innerPs.resize(2, piece_nums-1);
    std::vector<Eigen::Vector3d> statelist;
    double base_s = 0.0;
    for(int j = 0; j < piece_nums; j++ ){
      int resolution;
      if(j==0||j==piece_nums-1){
        resolution = dense_traj_res;
      }
      else{
        resolution = traj_res;
      }
      for(int k = 0; k <= resolution; k++){
        Eigen::Vector3d pos;
        double cur_s = base_s + k * (pieces_s[j] / resolution);
        // cout << "cur_s: " << cur_s << endl;
        // ROS_WARN("dis: %f, t: %f, cur_s:%f", dis, t, cur_s);
        // Vector2d posx = interpolate(x, s, cur_s);
        // Vector2d posy =  interpolate(y, s, cur_s);
        // pos << posx(0), posy(0), atan2(posy(1) / posx(1), 1);
        Vector2d posx = sx.CalPosition(cur_s);
        Vector2d posy = sy.CalPosition(cur_s);
        pos << posx(0), posy(0), atan2(posy(1), posx(1));
        statelist.push_back(pos);
        if(k==resolution && j!=piece_nums-1){
          ego_innerPs.col(j) = pos.head(2); 
        }
      } 
      base_s += pieces_s[j];
    }
    // std::cout<<"s: "<<kino_traj.singul<<"\n";
    // double tm1 = ros::Time::now().toSec();
    getRectangleConst(statelist);
    sfc_container.push_back(hPolys_);
    display_hPolys_.insert(display_hPolys_.end(),hPolys_.begin(),hPolys_.end());
    waypoints_container.push_back(ego_innerPs);
    iniState_container.push_back(trajs.start_state);
    finState_container.push_back(trajs.final_state);
    // basetime += initTotalduration;
  }

  double t1= ros::Time::now().toSec();
  std::cout<<"try to optimize!\n";
  
  int flag_success = ploy_traj_opt_->OptimizeTrajectory(iniState_container, finState_container, 
                                                      waypoints_container,duration_container, 
                                                      sfc_container,  singul_container,worldtime,0.0);
  std::cout<<"optimize ended!\n";
  double t2 = ros::Time::now().toSec();
  std::cout<<"opt time: "<<(t2-t1)<<std::endl;

  if (flag_success)
  {
      std::cout << "[PolyTrajManager] Planning success ! " << std::endl;
      for(unsigned int i = 0; i < index_corner.size() - 1; i++){
        std::cout<<"init duration: "<<duration_container[i]<<std::endl;
        std::cout<<"pieceNum: " << waypoints_container[i].cols() + 1 <<std::endl;
        std::cout<<"optimized total duration: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTraj(1).getTotalDuration()<<std::endl;
        std::cout<<"optimized jerk cost: "<<(*ploy_traj_opt_->getMinJerkOptPtr())[i].getTrajJerkCost(1)<<std::endl;
        // worldtime = traj_container_.singul_traj.back().end_time;
      }
      // ploy_traj_opt_->GetResult(0.1, refline);
  }
  else{
      ROS_WARN("[PolyTrajManager] Planning fails! ");
      return false;
  }
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