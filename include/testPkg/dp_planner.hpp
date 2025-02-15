#ifndef DP_ROUTING_H
#define DP_ROUTING_H

// #include <vector>
#include <cmath>
#include <memory>
// #include <ros/ros.h>
// #include <iostream>
// #include "DynamicPathStruct.hpp"
// #include "Obstacles.hpp"
// #include "CoarsePathGenerator.hpp"
// #include "df_planner/traj_optimizer_global.h"
#include "df_planner/traj_manager.h"
#include <chrono>
class DpPlanner {
public:
    DpPlanner(const bool is_using_global_df, const GlobalPath& refline, const CartesianState& init_state, const Obstacles& obj, int count, const vector<CartesianState>& last_path) {
        last_best_path_ = last_path;
        frame_count = count;
        refline_ = refline;
        obj_ = obj;
        is_using_global_df_ = is_using_global_df;
        ROS_WARN("current_state: %f, %f, %f, %f, %f, %f", init_state.x, init_state.y, init_state.theta, init_state.speed, init_state.acc, init_state.kappa);
        CarToFrenet(init_state, init_sl_state_);
        ROS_WARN("current_frenet_state:  s--%f,  l--%f,  ds--%f,  dds--%f", init_sl_state_.s, init_sl_state_.l, init_sl_state_.ds, init_sl_state_.dds);
        Initialize();
        // GenerateSLTrange();
    }

    DpPlanner(const bool is_using_global_df, std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline,  const traj_utils::Trajectory traj,const CartesianState& init_state, const Obstacles& obj, int count, const vector<CartesianState>& last_path) {
        last_best_path_ = last_path;
        frame_count = count;
        df_refline_ = df_refline;
        // traj_ =  (*df_refline_->getMinJerkOptPtr())[0].getTraj(1);
        // traj_.setPiece_S();
        traj_ = traj;
        obj_ = obj;
        is_using_global_df_ = is_using_global_df;
        ROS_WARN("current_state: %f, %f, %f, %f, %f, %f", init_state.x, init_state.y, init_state.theta, init_state.speed, init_state.acc, init_state.kappa);
        CarToFrenet(init_state, init_sl_state_);
        ROS_WARN("current_frenet_state:  s--%f,  l--%f,  ds--%f,  dds--%f", init_sl_state_.s, init_sl_state_.l, init_sl_state_.ds, init_sl_state_.dds);
        Initialize();
        // GenerateSLTrange();
    }
    // std::vector<CartesianState> Solve();
    void DynamicProgramming();
    vector<CartesianState> getBestPath() { return best_path_; }
    vector<GlobalPathPoint> getRefPath() { return best_path_ref_; }
    ~DpPlanner() {}

private:
    bool is_using_global_df_ = false;
    std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline_ = nullptr;
    traj_utils::Trajectory traj_;
    GlobalPath refline_;
    vector<CartesianState> best_path_;
    vector<CartesianState> last_best_path_;
    vector<GlobalPathPoint> best_path_ref_;
    FrenetState init_sl_state_;
    Obstacles obj_;
    // Configuration parameters
    double v_ref;
    int s_num, l_num, t_num;
    double v_max, a_max, t_hri,  s_hri, l_hri;
    double t_res, s_res, l_res;
    vector<double> s_range, l_range, t_range;
    int frame_count = 0; //用于计算当前绝对时间

    // Conversion functions
    void CarToFrenet(const CartesianState& car_state, FrenetState& fre_state) {
        // ROS_WARN("enter cartofrenet 0");
        auto start_time = std::chrono::high_resolution_clock::now();
        
        if (!is_using_global_df_)
            CarToFrenet1(car_state, fre_state);
        else 
            CarToFrenet2(car_state, fre_state);

        auto end_time = std::chrono::high_resolution_clock::now();
        // 计算时间间隔
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        // 输出时间间隔
        // std::cout << "CarToFrenet时间: " << elapsed_seconds.count() << " 秒" << std::endl;
    }

    void CarToFrenet1(const CartesianState& car_state, FrenetState& fre_state) {
        //找最近点
        GlobalPathPoint ref_state = GlobalPathPoint{0, 0, 0, 0, 0, 0};
        double min_dist = std::numeric_limits<double>::infinity();
        int min_ind = 0;
        for(int i = 0; i < refline_.x.size(); ++i) {
            double dx = car_state.x - refline_.x[i];
            double dy = car_state.y - refline_.y[i];
            double dist = hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                min_ind = i;
            }
        }
        // cout << "min_ind: " << min_ind << " min_dist: " << min_dist << endl;
        if(min_ind > 0 && min_ind < refline_.x.size() - 1)
            ref_state = CoarsePathGenerator::FindNearestPt(refline_.x[min_ind-1], refline_.y[min_ind-1], refline_.all_s[min_ind-1], 
                                                                                                                refline_.theta[min_ind-1], refline_.kappa[min_ind-1], refline_.dkappa[min_ind-1], 
                                                                                                                refline_.x[min_ind], refline_.y[min_ind], refline_.all_s[min_ind], 
                                                                                                                refline_.theta[min_ind], refline_.kappa[min_ind], refline_.dkappa[min_ind], 
                                                                                                                refline_.theta[min_ind + 1], refline_.kappa[min_ind + 1], refline_.dkappa[min_ind + 1], 
                                                                                                                refline_.s[min_ind-1] ,refline_.s[min_ind], car_state.x, car_state.y);
        else if (min_ind == 0)
            ref_state = CoarsePathGenerator::FindNearestPt(refline_.x[min_ind], refline_.y[min_ind], refline_.all_s[min_ind], 
                                                                                                                refline_.theta[min_ind], refline_.kappa[min_ind], refline_.dkappa[min_ind], 
                                                                                                                refline_.theta[min_ind + 1], refline_.kappa[min_ind + 1], refline_.dkappa[min_ind + 1], 
                                                                                                                refline_.s[min_ind] , car_state.x, car_state.y);
        else if (min_ind == refline_.x.size() - 1)
            ref_state = CoarsePathGenerator::FindNearestPt(refline_.x[min_ind-1], refline_.y[min_ind-1], refline_.all_s[min_ind-1], 
                                                                                                                refline_.theta[min_ind-1], refline_.kappa[min_ind-1], refline_.dkappa[min_ind-1], 
                                                                                                                refline_.theta[min_ind], refline_.kappa[min_ind], refline_.dkappa[min_ind], 
                                                                                                                refline_.s[min_ind - 1] , car_state.x, car_state.y);
                                                                                                                
        double del_theta = car_state.theta - ref_state.theta;
        // double kappa_x = tan(car_state.delta) / L;
    
        fre_state.s = ref_state.s;
        double dy = car_state.y - ref_state.y;
        double dx = car_state.x - ref_state.x;
        fre_state.l = std::copysign(hypot(dx, dy), (dy * cos(ref_state.theta) - dx * sin(ref_state.theta)));
        fre_state.dl = (1 - ref_state.kappa * fre_state.l) * tan(del_theta);
        fre_state.ddl = -(ref_state.dkappa * fre_state.l + ref_state.kappa * fre_state.dl) * tan(del_theta) +
                        (1 - ref_state.kappa * fre_state.l) / (cos(del_theta) * cos(del_theta)) *
                        (((1 - ref_state.kappa * fre_state.l) / cos(del_theta) * car_state.kappa - ref_state.kappa));
        fre_state.ds = car_state.speed * cos(del_theta) / (1 - ref_state.kappa * fre_state.l);
        fre_state.dds = (car_state.acc * cos(del_theta) - pow(fre_state.ds, 2) *
                        (fre_state.dl * ((1 - ref_state.kappa * fre_state.l) / cos(del_theta) * car_state.kappa - ref_state.kappa) -
                        (ref_state.dkappa * fre_state.l + ref_state.kappa * fre_state.dl))) /
                        (1 - ref_state.kappa * fre_state.l);
        // ROS_WARN("init_sl_finished");
        // ROS_WARN("s:%f, l:%f, ds:%f, dds:%f, dl:%f, ddl:%f", fre_state.s, fre_state.l, fre_state.ds, fre_state.dds, fre_state.dl, fre_state.ddl);
    }

    void CarToFrenet2(const CartesianState& car_state, FrenetState& fre_state) {
        // ROS_WARN("enter cartofrenet 2");
        //找最近点
        Vector2d cur_pos(car_state.x, car_state.y);
        if(df_refline_ == nullptr) ROS_ERROR("df refline is null");
        GlobalPathPoint ref_state = GlobalPathPoint{0, 0, 0, 0, 0, 0};
        MatrixXd pos_matrix= traj_.getPositions();
        double min_dist = 1e6;
        int min_ind = 0;
        for(int i = 0; i < pos_matrix.cols(); ++i) {
            double dx = car_state.x - pos_matrix(0, i);
            double dy = car_state.y - pos_matrix(1, i);
            double dist = hypot(dx, dy);
            if (dist < min_dist) {
                min_dist = dist;
                min_ind = i;
            }
        }
        cout << "min_ind: " << min_ind << ", min_dist: " << min_dist << endl;

        if(min_ind > 0 && min_ind < pos_matrix.cols() - 1) {
            // ROS_WARN("enter inner");
            double t = traj_.getDurations()[min_ind - 1];
            Vector2d min_res = traj_.findMinPtInSegment(min_ind - 1, cur_pos, t);
            if (!(fabs(min_res(1) - t) < 1e-8)) { //最近点不是端点 
                ref_state = traj_.getState(min_res(1), min_ind - 1);
            }
            else {
                // cout << "min_dis0: " <<min_res(0) << endl;
                ROS_WARN("last part");
                min_res = traj_.findMinPtInSegment(min_ind, cur_pos, 0.0);
                ref_state = traj_.getState(min_res(1), min_ind);
            }
            // cout << "min_dis: " <<min_res(0) << endl;
        } 
        else if (min_ind == 0) {
            // ROS_WARN("enter start");
            Vector2d min_res = traj_.findMinPtInSegment(0, cur_pos, 0.0);
            ref_state = traj_.getState(min_res(1), 0);
            // cout << "min_dis: " <<min_res(0) << endl;
        }
        else {
            // ROS_WARN("enter end");
            double t = traj_.getDurations()[min_ind - 1];
            Vector2d min_res = traj_.findMinPtInSegment(min_ind - 1, cur_pos, t);
            ref_state = traj_.getState(min_res(1), min_ind - 1);
            // cout << "min_dis: " <<min_res(0) << endl;

        }                                                                                                       
        double del_theta = car_state.theta - ref_state.theta;
        // double kappa_x = tan(car_state.delta) / L;
    
        fre_state.s = ref_state.s;
        double dy = car_state.y - ref_state.y;
        double dx = car_state.x - ref_state.x;
        fre_state.l = std::copysign(hypot(dx, dy), (dy * cos(ref_state.theta) - dx * sin(ref_state.theta)));
        fre_state.dl = (1 - ref_state.kappa * fre_state.l) * tan(del_theta);
        fre_state.ddl = -(ref_state.dkappa * fre_state.l + ref_state.kappa * fre_state.dl) * tan(del_theta) +
                        (1 - ref_state.kappa * fre_state.l) / (cos(del_theta) * cos(del_theta)) *
                        (((1 - ref_state.kappa * fre_state.l) / cos(del_theta) * car_state.kappa - ref_state.kappa));
        fre_state.ds = car_state.speed * cos(del_theta) / (1 - ref_state.kappa * fre_state.l);
        fre_state.dds = (car_state.acc * cos(del_theta) - pow(fre_state.ds, 2) *
                        (fre_state.dl * ((1 - ref_state.kappa * fre_state.l) / cos(del_theta) * car_state.kappa - ref_state.kappa) -
                        (ref_state.dkappa * fre_state.l + ref_state.kappa * fre_state.dl))) /
                        (1 - ref_state.kappa * fre_state.l);
        // ROS_WARN("init_sl_finished");
        // ROS_WARN("s:%f, l:%f, ds:%f, dds:%f, dl:%f, ddl:%f", fre_state.s, fre_state.l, fre_state.ds, fre_state.dds, fre_state.dl, fre_state.ddl);
    }
    
    void FrenetToCar(const FrenetState& fre_state, CartesianState& car_state) {
        auto start_time = std::chrono::high_resolution_clock::now();
        // ROS_WARN("enter FrenetToCar");
        //这里将refline上的参考点也保存下来用于构造后续优化的目标函数，直接对best_path_ref进行插值，之后如果在别的地方调用这个函数会出问题
        GlobalPathPoint ref_state{0, 0, 0, 0, 0, 0};
        if(!is_using_global_df_) {
            int first_ind = -1;
            for(int i = 1; i < refline_.x.size(); ++i) {
                if (refline_.all_s[i] > fre_state.s) {
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
                                                                                                            refline_.s[first_ind], fre_state.s);    
            }else {
                ref_state = GlobalPathPoint{refline_.theta[refline_.x.size() - 1], refline_.kappa[refline_.x.size() - 1], refline_.dkappa[refline_.x.size() - 1], refline_.all_s[refline_.x.size() - 1], refline_.x[refline_.x.size() - 1], refline_.y[refline_.x.size() - 1]};
            }
        }
        else {
            double s = 0.0, raletive_s = 0.0;
            int piece_ind = -1;
            for(int i = 0; i < traj_.Pieces_S.size(); ++i) {
                if(s +traj_.Pieces_S[i] >= fre_state.s) {
                    piece_ind = i;
                    raletive_s = fre_state.s - s;
                    break;
                }
                s += traj_.Pieces_S[i];
            }
            if(piece_ind == -1) {
                // ROS_ERROR("Find S failed, need_S: %f, all_S: %f", fre_state.s, traj_.Pieces_allS.back());
                // return;
                ROS_WARN("Find S failed, need_S: %f, all_S: %f", fre_state.s, traj_.Pieces_allS.back());
                piece_ind = traj_.Pieces_S.size() - 1;
            }
            double t = traj_.findTInSegment(piece_ind, raletive_s);
            ref_state = traj_.getState(t, piece_ind);
        }
        best_path_ref_.emplace_back(ref_state);
        car_state.x = ref_state.x - fre_state.l * sin(ref_state.theta);
        car_state.y = ref_state.y + fre_state.l * cos(ref_state.theta);
        car_state.theta = ref_state.theta + atan2(fre_state.dl / (1 - ref_state.kappa * fre_state.l), 1);
        car_state.speed = hypot(fre_state.ds * (1 - ref_state.kappa * fre_state.l), fre_state.ds * fre_state.dl);
        double del_theta = car_state.theta - ref_state.theta;
        // cout << "del_theta: " << del_theta << endl;
        double temp_kappa = ((fre_state.ddl + (ref_state.dkappa * fre_state.l + ref_state.kappa * fre_state.dl) * tan(del_theta)) *
                            (cos(del_theta) * cos(del_theta) / (1 - ref_state.kappa * fre_state.l)) + ref_state.kappa) *
                            cos(del_theta) / (1 - ref_state.kappa * fre_state.l);
        // cout << "init_delta: " << atan2(2.7 * temp_kappa, 1) << endl;
        // cout << "ref_state:(曲率) " << ref_state.kappa << ", " << ref_state.dkappa << endl;
        car_state.kappa = temp_kappa;
        car_state.acc = fre_state.dds * (1 - ref_state.kappa * fre_state.l) / cos(del_theta) +
                    pow(fre_state.dds, 2) / cos(del_theta) *
                    (fre_state.dl * ((1 - ref_state.kappa * fre_state.l) / cos(del_theta) * car_state.kappa - ref_state.kappa) -
                        (ref_state.dkappa * fre_state.l + ref_state.kappa * fre_state.dl));
        // cout << "Car_state: " << car_state.x << " " << car_state.y << " " << car_state.theta << " " << car_state.speed << " " << car_state.acc << " " << atan(2.7 * temp_kappa)<< endl;
    
        auto end_time = std::chrono::high_resolution_clock::now();
        // 计算时间间隔
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        // 输出时间间隔
        // std::cout << "FrenetToCar时间: " << elapsed_seconds.count() << " 秒" << std::endl;
    }
    void FrenetToCarOnlyPos(const double s, const double l, const double dl, CartesianState& car_state) {
        // cout << "s: " << s << " l: " << l << " dl: " << dl << endl;
        // ROS_WARN("enterFrenetToCarOnlyPos");
        //这里将refline上的参考点也保存下来用于构造后续优化的目标函数，直接对best_path_ref进行插值，之后如果在别的地方调用这个函数会出问题
        GlobalPathPoint ref_state{0, 0, 0, 0, 0, 0};
        if(!is_using_global_df_) {
            int first_ind = -1;
            for(int i = 1; i < refline_.x.size(); ++i) {
                if (refline_.all_s[i] > s) {
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
                                                                                                            refline_.s[first_ind], s);    
            }else {
                ref_state = GlobalPathPoint{refline_.theta[refline_.x.size() - 1], refline_.kappa[refline_.x.size() - 1], refline_.dkappa[refline_.x.size() - 1], refline_.all_s[refline_.x.size() - 1], refline_.x[refline_.x.size() - 1], refline_.y[refline_.x.size() - 1]};
            }
        }
        else {
            double all_s = 0.0, relative_s = 0.0;
            int piece_ind = -1;
            for(int i = 0; i < traj_.Pieces_S.size(); ++i) {
                if(all_s +traj_.Pieces_S[i] >= s) {
                    piece_ind = i;
                    relative_s = s - all_s;
                    break;
                }
                all_s += traj_.Pieces_S[i];
            }
            if(piece_ind == -1) {
                // ROS_ERROR("Find S failed");
                // return;
                ROS_WARN("Find S failed, need_S: %f, all_S: %f", s, traj_.Pieces_allS.back());
                piece_ind = traj_.Pieces_S.size() - 1;
            }
            double t = traj_.findTInSegment(piece_ind, relative_s);
            ref_state = traj_.getState(t, piece_ind);
        }
        // cout << ref_state.x << " " << ref_state.y << " " << ref_state.theta << " " << ref_state.kappa << " " << ref_state.s << endl;
        car_state.x = ref_state.x - l * sin(ref_state.theta);
        car_state.y = ref_state.y + l * cos(ref_state.theta);
        car_state.theta = ref_state.theta + atan2(dl / (1 - ref_state.kappa * l), 1);
    }

    void FrenetToCarOnlyPos2(const GlobalPathPoint ref_state, const double l, const double dl, CartesianState& car_state) {
        // cout << ref_state.x << " " << ref_state.y << " " << ref_state.theta << " " << ref_state.kappa << " " << ref_state.s << endl;
        car_state.x = ref_state.x - l * sin(ref_state.theta);
        car_state.y = ref_state.y + l * cos(ref_state.theta);
        car_state.theta = ref_state.theta + atan2(dl / (1 - ref_state.kappa * l), 1);
    }

    // Collision detection
    int isCollision(const CartesianState& xy_state, const vector<Vector3d>& obs) {
        // cout << "xy_state: " << xy_state.x << " " << xy_state.y << endl;
        //他车看作椭圆进行碰撞检测
        double len = 4.7, width = 2.0, radius = sqrt(2);
        //测试，未进行坐标系变换，没有考虑他车的角度
        double front_x = xy_state.x + 2.7 * cos(xy_state.theta);
        double front_y = xy_state.y + 2.7 * sin(xy_state.theta);
        for (auto& ob : obs) {
            double a = len * sqrt(2) / 2 + radius, b = width * sqrt(2) / 2 + radius;
            double res = pow(xy_state.x - ob(0), 2.0) / pow(a, 2.0) + pow(xy_state.y - ob(1), 2.0) / pow(b, 2.0);
            if (res < 1) {
                // ROS_WARN("Collision detected");
                // cout << "xy_state: " << xy_state.x << " " << xy_state.y << " " <<  "ob: " << ob(0) << " " << ob(1) << endl;
                return 1;
            }
            double res_front = pow(front_x - ob(0), 2.0) / pow(a, 2.0) + pow(front_y - ob(1), 2.0) / pow(b, 2.0);
            if(res_front < 1) {
                return 1;
            }
        }
        return 0;
    };

    // Cost function
    double costFunction(double s, double l, double t, double prev_s, double prev_l, double prev_t, double prev_speed, double prev_acc, const Obstacles& obs);

    // Dynamic programming
    std::vector<std::vector<std::vector<double>>> cost_matrix_;
    std::vector<std::vector<std::vector<Vector3d>>> backtrace_;
    std::vector<std::vector<std::vector<double>>> speed_matrix_;
    std::vector<std::vector<std::vector<double>>> acc_matrix_;

    void Initialize();
    std::vector<Vector3d> Backtrack();
    void GenerateSLTrange() {
         s_range.emplace_back(0);
        double ds_dense = std::max(0.1, std::min(init_sl_state_.ds *t_res, s_hri / (s_num - 1)));
        for (int i = 1; i <= (s_num - 1) / 2; ++i) {
            s_range.emplace_back(ds_dense * i);
        }
        double ds_sparse = (s_hri - ds_dense * (s_num - 1) / 2) / ((s_num - 1) / 2);
        for (int i = 1; i <= (s_num - 1) / 2; ++i) {
            s_range.emplace_back(ds_dense * (s_num - 1) / 2 + ds_sparse * i);
        }
        // linspace(0, s_hri, s_num,  s_range);
        linspace(-l_hri / 2, l_hri / 2, l_num,  l_range);
        linspace(0, t_hri, t_num, t_range);
        // ROS_WARN("GenerateSLTrange finished");
        // ROS_WARN("s_range: %f, %f, %f", s_range[0], s_range[s_range.size() / 2], s_range[s_range.size() - 1]);
        ROS_WARN("s_range: %f, %f, %f, %f, %f", s_range[0], s_range[1], s_range[2],s_range[s_range.size()-2],s_range[s_range.size()-1]);
        // ROS_WARN("l_range: %f, %f, %f", l_range[0], l_range[l_range.size() / 2], l_range[l_range.size() - 1]);
        // ROS_WARN("t_range: %f, %f, %f", t_range[0], t_range[t_range.size() / 2], t_range[t_range.size() - 1]);
    }
    void linspace(double start, double end, int num, std::vector<double>& linspace) {
        linspace.clear();
        double step = (end - start) / (num - 1);
        for (int i = 0; i < num; ++i) {
            linspace.emplace_back(start + i * step);
        };
    }

    void updateMatrices(int s_idx, int l_idx, int t_idx, int prev_s_idx, int prev_l_idx, int prev_t_idx, double total_cost, double prev_s, double prev_l, double prev_t, double prev_speed) {
        if (total_cost < cost_matrix_[s_idx][l_idx][t_idx]) {
            cost_matrix_[s_idx][l_idx][t_idx] = total_cost;
            // speed_matrix_[s_idx][l_idx][t_idx] = std::hypot((s_range[s_idx] - prev_s) / (t_range[t_idx] - prev_t), (l_range[l_idx] - prev_l) / (t_range[t_idx] - prev_t));
            speed_matrix_[s_idx][l_idx][t_idx] = std::abs((s_range[s_idx] - prev_s) / (t_res));
            acc_matrix_[s_idx][l_idx][t_idx] = (speed_matrix_[s_idx][l_idx][t_idx] - prev_speed) / (t_res);
            if (t_idx > 1) {
                backtrace_[s_idx][l_idx][t_idx] = {(double)prev_s_idx, (double)prev_l_idx, (double)prev_t_idx};
            }
            else {
                backtrace_[s_idx][l_idx][t_idx] = {-1, -1, -1};
            }
        }
    }

    double costFunction(double s, double l, double t, double prev_s, double prev_l, double prev_t, double prev_speed, double prev_acc, const vector<Vector3d>& obs, GlobalPathPoint& ref_state) {
        const double w_offset = 1500;//1000.0;
        const double w_lat_change = 800;//100.0;
        const double w_vlat = 1000;//1000.0;
        const double w_vref = 500.0;
        const double w_v_change = 500.0;//1.0;
        const double w_a_change = 100.0;
        const double w_effi = 800;//1000.0;
        const double w_acc = 10;//10.0;
        const double w_consistency = 1000;//800.0;
    
        // 计算速度
        double dt = t - prev_t; //+std::numeric_limits<double>::epsilon();
        double speed = std::abs(s - prev_s)/dt; //std::hypot(s - prev_s, l - prev_l) / dt;

        // 速度或加速度超过限制，返回无穷大成本
        if (std::abs(prev_speed - speed) / dt > a_max){     //|| isCollision(s, l, obj)) {
            return std::numeric_limits<double>::infinity();
        }

        // 计算各项成本
        double offset_cost = w_offset * std::abs(l);
        double dl = (l - prev_l) / (s - prev_s + std::numeric_limits<double>::epsilon());
        double lat_change_cost = w_lat_change * std::abs(dl);
        double vlat_cost = w_vlat * std::abs((l - prev_l) / dt);
        double vref_cost = w_vref * std::abs(speed / v_ref - 1); // - speed); // 目标速度10/3.6 m/s
        // cout << "vref_cost:  " << vref_cost << endl;
        double v_change_cost = w_v_change * std::abs(speed - prev_speed);
        double a_change_cost = w_a_change * std::abs((speed - prev_speed) / dt - prev_acc);
        double acc_cost = w_acc * std::abs((speed - prev_speed) / dt);
        double effi_cost = w_effi * (s_hri - s + prev_s);

        // 碰撞成本
        CartesianState car_state{0, 0, 0, 0, 0, 0};
        // FrenetToCarOnlyPos(s + init_sl_state_.s, l, dl, car_state);
        FrenetToCarOnlyPos2(ref_state, l, dl, car_state);

        double coll_cost = 0;//isCollision(car_state, obs) * 1e8;
        if(isCollision(car_state, obs) > 0) {
            // ROS_WARN("Collision detected");
            return std::numeric_limits<double>::infinity();
        }
        int ind = t / 0.1 +  5;
        double consist_cost = w_consistency * (ind < last_best_path_.size() ?  hypot(car_state.x - last_best_path_[ind].x, car_state.y - last_best_path_[ind].y) : 0);
        // ROS_WARN("consist_cost: %f", consist_cost);
        // cout << "ind: " << ind << ", consist_cost: " << consist_cost << endl;
    
        // 总成本
        double cost = offset_cost + lat_change_cost + vlat_cost + vref_cost + v_change_cost + a_change_cost + acc_cost + effi_cost + coll_cost + consist_cost;
        // cout << "cost: " << cost << endl;
        return cost;
    }

    vector<double> linear(vector<double>& x, vector<double>& y, double resolution) {
        int n = x.size();
        vector<double> new_x;
        linspace(x[0], x[n-1], (x[n-1] - x[0]) / resolution + 1, new_x);
        vector<double> new_y(new_x.size());
        double last_x = 0, last_y = 0, next_x = 0, next_y = 0;
        int p = 0;
        for (int i = 0; i < new_x.size(); i++) {
            while (p < n&& new_x[i] < x[p]) {
                p++;
                if (p == n)
                    break;
            }
            // 考虑落在最小和最大范围外的点，做特殊处理，取最近相邻的点作为线性插值的计算
            if (p == 0) {
                last_x = x[0];
                last_y = y[0];
                next_x = x[1];
                next_y = y[1];
            }
            else if (p == n) {
                last_x = x[n-2];
                last_y = y[n-2];
                next_x = x[n-1];
                next_y = y[n-1];
            }
            else {
                last_x = x[p-1];
                last_y = y[p-1];
                next_x = x[p];
                next_y = y[p];
            }
            new_y[i] = (new_x[i]-last_x)*((next_y - last_y) / (next_x - last_x))+last_x;
        }
        return new_y;
    }

    //线性插值
    vector<double> linear(vector<double>& x, vector<double>& y, vector<double>& new_x) {
        int n = x.size();        vector<double> new_y(new_x.size());
        double last_x = 0, last_y = 0, next_x = 0, next_y = 0;
        int p = 0;
        for (int i = 0; i < new_x.size(); i++) {
            while (p < n&& new_x[i] >= x[p]) {
                ++p;
                if (p == n)
                    break;
            }
            // 考虑落在最小和最大范围外的点，做特殊处理，取最近相邻的点作为线性插值的计算
            if (p == 0) {
                last_x = x[0];
                last_y = y[0];
                next_x = x[1];
                next_y = y[1];
            }
            else if (p == n) {
                last_x = x[n-2];
                last_y = y[n-2];
                next_x = x[n-1];
                next_y = y[n-1];
            }
            else {
                last_x = x[p-1];
                last_y = y[p-1];
                next_x = x[p];
                next_y = y[p];
            }
            new_y[i] = (new_x[i]-last_x)*((next_y - last_y) / (next_x - last_x +  std::numeric_limits<double>::epsilon()))+last_y;
            // cout << "new_y: " << new_y[i] << endl;
        }
        return new_y;
    }

    // Function to compute first and second derivatives
    void computeDerivatives(const std::vector<double>& x, const std::vector<double>& y,
                            std::vector<double>& dy, std::vector<double>& d2y) {
        size_t n = x.size();
        dy.resize(n, 0.0);
        d2y.resize(n, 0.0);
    
        // Use central difference for interior points
        for (size_t i = 1; i < n - 1; ++i) {
            double dx = x[i + 1] - x[i - 1];
            if(dx == 0) dx += std::numeric_limits<double>::epsilon();
            dy[i] = (y[i + 1] - y[i - 1]) / dx;
            d2y[i] = (y[i + 1] - 2 * y[i] + y[i - 1]) / (dx * dx);
        }
    
        // Use forward difference for the first point
        if (n > 1) {
            dy[0] = (y[1] - y[0]) / (x[1] - x[0] + std::numeric_limits<double>::epsilon());
            // d2y[0] = 2 * (dy[1] - dy[0]) / (x[2] - x[0] + std::numeric_limits<double>::epsilon()) - dy[0] / (x[1] - x[0] + std::numeric_limits<double>::epsilon());
            d2y[0] =(dy[1] - dy[0]) / (x[2] - x[0] + std::numeric_limits<double>::epsilon());
        }
    
        // Use backward difference for the last point
        if (n > 1) {
            dy[n - 1] = (y[n - 1] - y[n - 2]) / (x[n - 1] - x[n - 2] + std::numeric_limits<double>::epsilon());
            // d2y[n - 1] = 2 * (dy[n - 2] - dy[n - 1]) / (x[n - 2] - x[n - 4] + std::numeric_limits<double>::epsilon()) - dy[n - 1] / (x[n - 1] - x[n - 2] + std::numeric_limits<double>::epsilon());
            d2y[n - 1] =  (dy[n - 1] - dy[n - 2]) / (x[n - 1] - x[n - 2]);
        }
    
        // Note: The second derivative for the last point might have a better approximation if more points are available.
        // The current formula is a simplified version for demonstration.
    }
};

#endif