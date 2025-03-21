#include "dp_planner.hpp"

void DpPlanner::Initialize() {
    // Initialize configuration parameters
    v_ref = 10 / 3.6; // m/s
    v_max = 15 / 3.6; // m/s
    a_max = 2; // m/s^2
    t_num = 7;
    t_hri = 6;
    s_num = 19;
    if(!is_using_global_df_)
        s_hri = std::min(v_max * t_hri, refline_.all_s.back() - init_sl_state_.s);
    else
        s_hri = std::min(v_max * t_hri, traj_.Pieces_allS.back() - init_sl_state_.s);
    // s_num = s_hri  < refline_.all_s.back() - init_sl_state_.s ? 21 : 11;

    // s_hri = std::min(v_max * t_hri, 60 - init_sl_state_.s); //测试用
    l_num = 11;
    l_hri = 6.4;

    t_res = t_hri / (t_num - 1);
    s_res = s_hri / (s_num - 1);
    l_res = l_hri / (l_num - 1);

    if(s_hri < 3) s_num = 5;
    else if(s_hri < 5) s_num = 7;
    else if(s_hri < 7) s_num = 9;


    s_range.emplace_back(0);
    // double ds_dense = std::max(0.1, std::min(init_sl_state_.ds *t_res, s_hri / (s_num - 1)));
    double ds_dense = std::min(s_hri / (s_num - 1), std::max(0.1, init_sl_state_.ds *t_res));
    for (int i = 1; i <= (s_num - 1) / 2; ++i) {
        s_range.emplace_back(ds_dense * i);
    }
    double ds_sparse = (s_hri - ds_dense * (s_num - 1) / 2) / ((s_num - 1) / 2);
    if(ds_sparse > 1e-6) {
        for (int i = 1; i <= (s_num - 1) / 2; ++i) {
            s_range.emplace_back(ds_dense * (s_num - 1) / 2 + ds_sparse * i);
        }
    }
    else{
        s_num = s_range.size();
    }
    
    linspace(-l_hri / 2, l_hri / 2, l_num,  l_range);
    linspace(0, t_hri, t_num, t_range);
    // ROS_WARN("GenerateSLTrange finished");
    // ROS_WARN("s_range: %f, %f, %f", s_range[0], s_range[s_range.size() / 2], s_range[s_range.size() - 1]);
    

    //s_hri需要随工况的变化而变化
    //1. 巡航，正常采样，期望速度为巡航速度
    //2. 跟车，距离为s_obs - s_init - 3.7 - 2.35 - 1，每一帧期望速度为目标车辆速度
    //这里引入跟车的目标车辆，测试用
    double new_hri = 0;

        //先判断是否有正在停车的车辆
        for(auto& traj : MovingObs::obs_traj) {
            if(traj.type == "PARKING") {
                CartesianState obs_cartesian;
                CartesianState end_obs_cartesian;
                if(3 * frame_count < traj.trajs.size()) {
                    auto& cur_obs = traj.trajs[3 * frame_count];
                    obs_cartesian = CartesianState(cur_obs.cur_state.x + 1.35 * cos(cur_obs.cur_state.theta), cur_obs.cur_state.y + 1.35* sin(cur_obs.cur_state.theta), cur_obs.cur_state.theta, cur_obs.cur_state.v, 0, 0);
                }
                else {
                    auto& cur_obs =  traj.trajs.back();
                    obs_cartesian = CartesianState(cur_obs.cur_state.x + 1.35 * cos(cur_obs.cur_state.theta), cur_obs.cur_state.y + 1.35* sin(cur_obs.cur_state.theta), cur_obs.cur_state.theta, cur_obs.cur_state.v, 0, 0);
                }
                FrenetState obs_frenet;
                CarToFrenet(obs_cartesian, obs_frenet);
                
                auto& end_obs = traj.trajs.back();
                end_obs_cartesian = CartesianState(end_obs.cur_state.x + 1.35 * cos(end_obs.cur_state.theta), end_obs.cur_state.y + 1.35* sin(end_obs.cur_state.theta), end_obs.cur_state.theta, end_obs.cur_state.v, 0, 0);
                FrenetState end_obs_frenet;
                CarToFrenet(end_obs_cartesian, end_obs_frenet);

                if(end_obs_frenet.s > obs_frenet.s)  {//认为跟车
                    cout << "parking的跟车阶段" << endl;
                    if(obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7 < s_range[s_range.size() -1]) {
                        if(fabs(obs_frenet.l) < 3)
                            new_hri =  obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7 + 6.0 * obs_frenet.ds; //考虑这里是起始点的s，往上加一点
                    }
                }
                else if(fabs(obs_frenet.l) < 4){
                    cout << "parking的停车阶段" << endl;
                    if(end_obs_frenet.s - init_sl_state_.s - 3.7- 2.6 < s_range[s_range.size() -1]) {
                            new_hri = end_obs_frenet.s - init_sl_state_. s - 3.7 - 2.6;
                    }
                }
            } 
        }

        //再判断是否有同向的车辆
        for(auto& traj : MovingObs::obs_traj) {
            if(traj.type == "PEDESTRAIN") continue;
            if(traj.type == "FOLLOWING") {
                CartesianState obs_cartesian;
                if(3 * frame_count < traj.trajs.size()) {
                    auto& cur_obs = traj.trajs[3 * frame_count];
                    obs_cartesian = CartesianState(cur_obs.points.back().x + 1.35 * cos(cur_obs.points.back().theta), cur_obs.points.back().y + 1.35* sin(cur_obs.points.back().theta), cur_obs.points.back().theta, cur_obs.points.back().v, 0, 0);
                }
                else {
                    auto& cur_obs =  traj.trajs.back();
                    obs_cartesian = CartesianState(cur_obs.points.back().x + 1.35 * cos(cur_obs.points.back().theta), cur_obs.points.back().y + 1.35* sin(cur_obs.points.back().theta), cur_obs.points.back().theta, cur_obs.points.back().v, 0, 0);
                }
                FrenetState obs_frenet;
                CarToFrenet(obs_cartesian, obs_frenet);

                if(obs_frenet.s - init_sl_state_.s > 0 && obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7 < s_range[s_range.size() -1]) {
                    if(fabs(obs_frenet.l) < 3) {
                        cout << "update new hri" << endl;
                        // cout << "frame_count: " << frame_count  << ", obs_frenet:" << obs_frenet.s << ", " << obs_frenet.l << endl;
                        // cout << "cur_frenet:" << init_sl_state_.s << ", " << init_sl_state_.l << endl;
                        new_hri = min(new_hri, obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7);
                    }
                }
            } 
        }

        // 测试用，模拟障碍物位置，实际接入预测结果（p>0.2以上置信度，且差距小于0.05) 
        // double t = t_idx * t_res;    
        // double t = t_hri;
        // for(auto j : obj_.getMoveInds()) { 
        //     auto cur_obs = obj_.getObs()[j];
        //     if(cur_obs.speed(1) < 0){ //同向
        //         MatrixXd poly(2, cur_obs.vertex_x.size());
        //         for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
        //             poly(0, i) = cur_obs.vertex_x[i];
        //             poly(1, i) = cur_obs.vertex_y[i];
        //         }
        //         poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.3 *  frame_count + t); 
                
        //         CartesianState obs_cartesian((poly(0, 0) +  poly(0, 1)) / 2, (poly(1,1) + poly(1,2)) / 2, -M_PI / 2, 1, 0, 0);
        //         FrenetState obs_frenet;
        //         CarToFrenet(obs_cartesian, obs_frenet);
        //         // cout << "obs_frenet:" << obs_frenet.s << ", " << obs_frenet.l << endl;
        //         // cout << "cur_frenet:" << init_sl_state_.s << ", " << init_sl_state_.l << endl;
        //         if(obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7 < s_range[s_range.size() -1]) {
        //             if(new_hri > 0 || fabs(obs_frenet.l) < 3) {
        //                 cout << "update new hri" << endl;
        //                 new_hri = max(new_hri, obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7);
        //             }
        //         }
        //     } 
        // }

    if (new_hri > 0)  {
        cout << "new_hri" << endl;
        s_hri = new_hri;
        if(s_hri < 7) s_num = 11;
        if(s_hri < 3) s_num = 7;
        if(s_hri < 1) s_num = 5;
        linspace(0, s_hri, s_num,  s_range);
    }
    ROS_WARN("s_range: %f, %f, %f, %f, %f", s_range[0], s_range[1], s_range[2],s_range[s_range.size()-2],s_range[s_range.size()-1]);

    //3. 车辆正在停车，若目标车位在s_hri内部，目标车位前1.5m处应停车，约束s_hri为目标车位前1.5m处，考虑后轴中心，应该减去3.7

    // Initialize matrices
    cost_matrix_ = std::vector<std::vector<std::vector<double>>>(s_num, std::vector<std::vector<double>>(l_num, std::vector<double>(t_num, INFINITY))); //保存代价便于回溯
    backtrace_ =  std::vector<std::vector<std::vector<Vector3d>>>(s_num, std::vector<std::vector<Vector3d>>(l_num, std::vector<Vector3d>(t_num, {-1,-1,-1})));  //回溯路径
    speed_matrix_ = std::vector<std::vector<std::vector<double>>>(s_num, std::vector<std::vector<double>>(l_num, std::vector<double>(t_num, INFINITY)));  //用于添加速度变化代价
    acc_matrix_ = std::vector<std::vector<std::vector<double>>>(s_num, std::vector<std::vector<double>>(l_num, std::vector<double>(t_num, INFINITY)));  //用于添加加速度变化代价
    // ROS_WARN("dp_planner_init finished");
}

void DpPlanner::DynamicProgramming() {
    // Main loop of dynamic programming
    v_ref = 10 / 3.6;
    for (int t_idx = 1; t_idx < t_num; ++t_idx) {
        double t = t_range[t_idx];
        vector<Vector3d> obs_vehicles; //x,y,yaw
        //测试用，模拟障碍物位置，实际接入预测结果（p>0.2以上置信度，且差距小于0.05)
        // ROS_WARN("Start to get obs_vehicles");
        double stop_s = 1e6;
        
        // for(auto j : obj_.getMoveInds()) { 
        //     auto cur_obs = obj_.getObs()[j];
        //     MatrixXd poly(2, cur_obs.vertex_x.size());
        //     for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
        //     poly(0, i) = cur_obs.vertex_x[i];
        //     poly(1, i) = cur_obs.vertex_y[i];
        //     }
        //     poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.3 *  frame_count + t); 
        //     cout << "ori_obs: " << (poly(0, 0) +  poly(0, 1)) / 2 << ", " << (poly(1,1) + poly(1,2)) / 2 << endl;
            // if(cur_obs.speed(1) > 0){
            //     obs_vehicles.emplace_back(Vector3d{(poly(0, 0) +  poly(0, 1)) / 2, (poly(1,1) + poly(1,2)) / 2, M_PI / 2});
            // }
            // else {
            //     CartesianState obs_cartesian((poly(0, 0) +  poly(0, 1)) / 2, (poly(1,1) + poly(1,2)) / 2, -M_PI / 2, 1, 0, 0);
            //     FrenetState obs_frenet;
            //     CarToFrenet(obs_cartesian, obs_frenet);
            //     // cout << "obs_frenet:" << obs_frenet.s << ", " << obs_frenet.l << endl;
            //     // cout << "cur_frenet:" << init_sl_state_.s << ", " << init_sl_state_.l << endl;
            //     if(obs_frenet.s - init_sl_state_.s - 2.35 - 3.7 - 1 <= s_range[s_range.size() -1] && std::abs(obs_frenet.l) < 3) {
            //         stop_s = obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7;
            //         // cout << "obs_ds: " << obs_frenet.ds << endl;
            //         if(obs_frenet.s - init_sl_state_.s - 2.35 - 3.7 <= 4 || v_ref < 10/3.6) {
            //             v_ref = min(10/3.6, obs_frenet.ds);
            //         }
            //     }
            //     else {
            //         obs_vehicles.emplace_back(Vector3d{(poly(0, 0) +  poly(0, 1)) / 2, (poly(1,1) + poly(1,2)) / 2, -M_PI / 2});
            //         v_ref = 10 / 3.6;
            //     }
            // }
            // ROS_WARN("obs_vehicles: %f, %f, %f,  time: %f", obs_vehicles[obs_vehicles.size() - 1](0), obs_vehicles[obs_vehicles.size() - 1](1), obs_vehicles[obs_vehicles.size() - 1](2), 0.3 *  frame_count + t);
        // }

        for(auto& traj : MovingObs::obs_traj) {
            if(traj.type == "PEDESTRAIN") continue;
            CartesianState obs_cartesian;
            int relative_index = (int)(t * 10); //简化，实际应该调用插值函数
            if(3 * frame_count < traj.trajs.size()) {
                auto& cur_obs = traj.trajs[3 * frame_count];
                obs_cartesian = CartesianState(cur_obs.points[relative_index].x + 1.35 * cos(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].y + 1.35* sin(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                // obs_cartesian = CartesianState(cur_obs.points[relative_index].x, cur_obs.points[relative_index].y, cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                
            }
            else {
                auto& cur_obs =  traj.trajs.back();
                obs_cartesian = CartesianState(cur_obs.points[relative_index].x + 1.35 * cos(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].y + 1.35* sin(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
            }
            // cout << "new_obs:" <<  obs_cartesian.x << ", " << obs_cartesian.y << ", " << obs_cartesian.theta<< endl;
            if(traj.type == "OPPOSITE") {
                obs_vehicles.emplace_back(Vector3d{obs_cartesian.x, obs_cartesian.y, obs_cartesian.theta});
            }
            else {
                FrenetState obs_frenet;
                CarToFrenet(obs_cartesian, obs_frenet);
                // cout << "obs_frenet:" << obs_frenet.s << ", " << obs_frenet.l << endl;
                // cout << "cur_frenet:" << init_sl_state_.s << ", " << init_sl_state_.l << endl;
                if(obs_frenet.s - init_sl_state_.s > 0 && obs_frenet.s - init_sl_state_.s - 2.35 - 3.7 - 1 <= s_range[s_range.size() -1] && std::abs(obs_frenet.l) < 3) {
                    stop_s = obs_frenet.s - init_sl_state_.s - 1 - 2.35 - 3.7;
                    // cout << "obs_ds: " << obs_frenet.ds << endl;
                    if(obs_frenet.s - init_sl_state_.s - 2.35 - 3.7 <= 4 || v_ref < 10/3.6) {
                        v_ref = min(10/3.6, fabs(obs_frenet.ds));
                    }
                }
                else {
                    obs_vehicles.emplace_back(Vector3d{obs_cartesian.x, obs_cartesian.y, obs_cartesian.theta});
                    v_ref = 10 / 3.6;
                }
            }            
        }

        ROS_WARN("stop_s: %f", stop_s);
        ROS_WARN("v_ref: %f", v_ref);
        for (int s_idx = 0; s_idx < s_num; ++s_idx) {
            double s =s_range[s_idx];

            GlobalPathPoint ref_state{0, 0, 0, 0, 0, 0};
            if(is_using_global_df_) {
                double all_s = 0.0, relative_s = 0.0;
                int piece_ind = -1;
                for(int i = 0; i < traj_.Pieces_S.size(); ++i) {
                    if(all_s +traj_.Pieces_S[i] >= s + init_sl_state_.s) {
                        piece_ind = i;
                        relative_s = s + init_sl_state_.s - all_s;
                        break;
                    }
                    all_s += traj_.Pieces_S[i];
                }
                if(piece_ind == -1) {
                    // ROS_ERROR("Find S failed");
                    // ROS_ERROR("Find S failed, need_S: %f, all_S: %f", s + init_sl_state_.s, traj_.Pieces_allS.back());
                    // return;
                    ROS_WARN("Find S failed, need_S: %f, all_S: %f", s + init_sl_state_.s, traj_.Pieces_allS.back());
                    piece_ind = traj_.Pieces_S.size() - 1;
                    relative_s = traj_.Pieces_S.back();
                }
                double t = traj_.findTInSegment(piece_ind, relative_s);
                ref_state = traj_.getState(t, piece_ind);
                // cout << ref_state.x << endl;
            }
            else {
                int first_ind = -1;
                for(int i = 1; i < refline_.x.size(); ++i) {
                    if (refline_.all_s[i] >s + init_sl_state_.s) {
                        first_ind = i - 1;
                        // cout << "first_ind: " << first_ind << endl;
                        break;
                    }
                }
                // cout << "first_ind: " << first_ind << endl;
                if(first_ind != -1) {
                    ref_state = CoarsePathGenerator::FindRefPt(refline_.x[first_ind], refline_.y[first_ind], refline_.all_s[first_ind],
                                                                                                                refline_.theta[first_ind], refline_.kappa[first_ind], refline_.dkappa[first_ind],
                                                                                                                refline_.theta[first_ind + 1], refline_.kappa[first_ind + 1], refline_.dkappa[first_ind + 1],
                                                                                                                refline_.s[first_ind], s + init_sl_state_.s);    
                }else {
                    ref_state = GlobalPathPoint{refline_.theta[refline_.x.size() - 1], refline_.kappa[refline_.x.size() - 1], refline_.dkappa[refline_.x.size() - 1], refline_.all_s[refline_.x.size() - 1], refline_.x[refline_.x.size() - 1], refline_.y[refline_.x.size() - 1]};
                }
            }
            
            if(t_idx == 1 && s >= v_max * t_res) break;
            if(s > stop_s) break;
            for (int l_idx = 0; l_idx < l_num; ++l_idx) {
                double l = l_range[l_idx];
                // Calculate cost and update matrices
                if (t_idx == 1) {
                    double prev_s = 0.0;
                    double prev_l = init_sl_state_.l;
                    double prev_t = 0.0;
                    double total_cost = costFunction(s, l, t, prev_s, prev_l, prev_t, init_sl_state_.ds, init_sl_state_.dds, obs_vehicles, ref_state);
                    updateMatrices(s_idx, l_idx, t_idx, 0, 0, 0, total_cost, prev_s, prev_l, prev_t, init_sl_state_.ds);
                } else {
                    for (int prev_s_idx = 0; prev_s_idx <= s_idx; ++prev_s_idx) { // 考虑前进和可能的停车（不倒车）
                        double prev_s = s_range[prev_s_idx];
                        if (std::abs((s - prev_s)) / t_res > v_max) continue;
                        for (int prev_l_idx = 0; prev_l_idx < l_num; ++prev_l_idx) {
                            double prev_l = l_range[prev_l_idx];
                            double prev_t = t_range[t_idx - 1];
                            double prev_speed = speed_matrix_[prev_s_idx][prev_l_idx][t_idx - 1];
                            double prev_acc = acc_matrix_[prev_s_idx][prev_l_idx][t_idx - 1];
    
                            if (prev_acc > 1e4 || prev_speed > 1e4) {
                                continue;
                            }

                            double total_cost = cost_matrix_[prev_s_idx][prev_l_idx][t_idx - 1] + costFunction(s, l, t, prev_s, prev_l, prev_t, prev_speed, prev_acc, obs_vehicles, ref_state);
                            updateMatrices(s_idx, l_idx, t_idx, prev_s_idx, prev_l_idx, t_idx - 1, total_cost, prev_s, prev_l, prev_t, prev_speed);
                        }
                    }
                }
            }
        }
    }
    // ROS_WARN("dp_planner_dp_process finished");
    std::vector<Vector3d> trajectory = Backtrack(); //回溯s,l,t
    vector<double> t_list, s_list, l_list;
    for(auto& point : trajectory) {
        t_list.emplace_back(point(2));
        s_list.emplace_back(point(0));
        l_list.emplace_back(point(1));
        cout << "t: " << point(2) << ", s: " << point(0) << ", l: " << point(1) << endl;
    }
    vector<double> t_new_list;
    linspace(0, t_hri, (t_hri - 0) / 0.1 + 1.0, t_new_list); //采样时间0.1s
    vector<double> s_new_list = linear(t_list, s_list, t_new_list); //线性插值1
    vector<double> ds_new_list , dds_new_list;
    computeDerivatives(t_new_list, s_new_list, ds_new_list, dds_new_list);
    vector<double> l_new_list = linear(s_list, l_list, s_new_list); //线性插值2
    vector<double> dl_new_list, ddl_new_list;
    // cout <<  "s_list_end : " << s_list[s_list.size() - 1] << endl;
    // cout << "s_new_list_end : " << s_new_list[s_new_list.size() - 1] << endl;
    computeDerivatives(s_new_list, l_new_list, dl_new_list, ddl_new_list);
    for(int i = 0; i < t_new_list.size(); ++i) {
        FrenetState state(s_new_list[i] + init_sl_state_.s, l_new_list[i], ds_new_list[i], dds_new_list[i], dl_new_list[i], ddl_new_list[i]);
        // cout << "Frenet_state: " << s_new_list[i] << " " << state.l << " " << state.ds << " " << state.dds << " " << state.dl << " " << state.ddl << endl;
        CartesianState car_state{0, 0, 0, 0, 0, 0};
        FrenetToCar(state, car_state);
        best_path_.emplace_back(car_state);
        // cout << "Car_state: " << car_state.x << " " << car_state.y << " " << car_state.theta << " " << car_state.speed << " " << car_state.acc << " " << atan(car_state.kappa * 2.7) << endl;
    }
}

std::vector<Vector3d> DpPlanner::Backtrack() {
    // Backtrack to find the optimal path
    std::vector<Vector3d> trajectory;
    double min_cost = std::numeric_limits<double>::infinity();
    Vector3d best_idx = {-1, -1, -1};
    //找到最后时间层代价最小的终点
    for (int s_idx = 0; s_idx < s_num; ++s_idx) {
        for (int l_idx = 0; l_idx < l_num; ++l_idx) {
            if (cost_matrix_[s_idx][l_idx][t_num-1] < min_cost) {
                min_cost = cost_matrix_[s_idx][ l_idx][t_num-1];
                best_idx = Vector3d{(double)s_idx, (double)l_idx, (double)(t_num-1)};
            }
        }
    }
    // cout << "best_idx: " << best_idx(0) << " " << best_idx(1) << " " << best_idx(2) <<  " " << min_cost << endl;
    while (best_idx(2) != -1) {
        double s_idx = best_idx(0), l_idx = best_idx(1),  t_idx = best_idx(2);
        //将当前点加入轨迹
        trajectory.emplace_back(Vector3d{s_range[(int)s_idx], l_range[(int)l_idx], t_range[(int)t_idx]});
        // cout << "cost: " << cost_matrix_[(int)s_idx][(int)l_idx][(int)t_idx] << endl;
        // cout << "s: " << s_range[(int)s_idx] << " l: " << l_range[(int)l_idx] << " t: " << t_range[(int)t_idx] << endl;
        //获取回溯点
        best_idx = backtrace_[s_idx][l_idx][t_idx];
        // ROS_WARN("best_idx: %f, %f, %f", best_idx(0), best_idx(1), best_idx(2));
    }
    trajectory.emplace_back(Vector3d{0.0, init_sl_state_.l, 0.0});
    reverse(trajectory.begin(), trajectory.end());
    // ROS_WARN("dp_planner_backtrack finished");
    return trajectory;
}