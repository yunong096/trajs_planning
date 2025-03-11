#include <vector>
#include "dp_planner.hpp"
#include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_problem.h"

class ParkingTrajGenerator {
    public:
        ParkingTrajGenerator(const CartesianState& init_state,  std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline, const traj_utils::Trajectory traj,int count, const vector<CartesianState>& last_path){
            frame_count = count;
            refline_ = refline;
            df_refline_ = df_refline;
            traj_ = traj;
            // ROS_WARN("current_state: %f, %f, %f, %f, %f, %f", init_state.x, init_state.y, init_state.theta, init_state.speed, init_state.acc, init_state.kappa);
            CarToFrenet(init_state, init_sl_state_);
            // ROS_WARN("current_frenet_state:  s--%f,  l--%f,  ds--%f,  dds--%f", init_sl_state_.s, init_sl_state_.l, init_sl_state_.ds, init_sl_state_.dds);
            Initialize();
        };

    private:
        ~ParkingTrajGenerator(){};
        std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline_ = nullptr;
        traj_utils::Trajectory traj_;
        vector<CartesianState> best_path_;
        FrenetState init_sl_state_;
        CartesianState init_state_;
        int  = 0;

        double a_max = 0.6, a_min = -0.6;
        double v_max_abs = 2;
        double j_max = 0.6, j_min = -0.6;
        double dt_ = 0.1;

        void CarToFrenet(const CartesianState& car_state, FrenetState& fre_state) {
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
            // cout << "min_ind: " << min_ind << ", min_dist: " << min_dist  << ", car_state:" << car_state.x << ", " << car_state.y << endl;
    
            if(min_ind > 0 && min_ind < pos_matrix.cols() - 1) {
                // ROS_WARN("enter inner");
                double t = traj_.getDurations()[min_ind - 1];
                Vector2d min_res = traj_.findMinPtInSegment(min_ind - 1, cur_pos, t);
                if (!(fabs(min_res(1) - t) < 1e-8)) { //最近点不是端点 
                    ref_state = traj_.getState(min_res(1), min_ind - 1);
                }
                else {
                    // cout << "min_dis0: " <<min_res(0) << endl;
                    // ROS_WARN("last part");
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
            double s = 0.0, relative_s = 0.0;
            int piece_ind = -1;
            for(int i = 0; i < traj_.Pieces_S.size(); ++i) {
                if(s +traj_.Pieces_S[i] >= fre_state.s) {
                    piece_ind = i;
                    relative_s = fre_state.s - s;
                    break;
                }
                s += traj_.Pieces_S[i];
            }
            if(piece_ind == -1) {
                // ROS_ERROR("Find S failed, need_S: %f, all_S: %f", fre_state.s, traj_.Pieces_allS.back());
                // return;
                // ROS_WARN("Find S failed, need_S: %f, all_S: %f", fre_state.s, traj_.Pieces_allS.back());
                piece_ind = traj_.Pieces_S.size() - 1;
                relative_s = traj_.Pieces_S.back();
            }
            double t = traj_.findTInSegment(piece_ind, relative_s);
            ref_state = traj_.getState(t, piece_ind);
        
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

        vector<CartesianState>  solvePieceJerkProblem() {
            double path_length = traj_.Pieces_allS.back() - init_sl_state_.s;
            double total_t = max(1.3 *(v_max_abs * v_max_abs + path_length * a_max) / (a_max * v_max_abs) , 3.0);
            double start_t = 0.0, end_t = 0.0;
            int num_of_knots = (int)(10 * total_t);
            vector<pair<double, double>> s_bounds(num_of_knots);
            for(int i = 0; i < s_bounds.size(); ++i) {
                for(auto& traj : MovingObs::obs_traj) {
                    CartesianState obs_cartesian;
                    int relative_index = (int)(i * dt_ * 10); //简化，实际应该调用插值函数
                    if(3 * frame_count < traj.trajs.size()) {
                        auto& cur_obs = traj.trajs[3 * frame_count];
                        obs_cartesian = CartesianState(cur_obs.points[relative_index].x + 1.35 * cos(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].y + 1.35* sin(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                        // obs_cartesian = CartesianState(cur_obs.points[relative_index].x, cur_obs.points[relative_index].y, cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                        
                    }
                    else {
                        auto& cur_obs =  traj.trajs.back();
                        obs_cartesian = CartesianState(cur_obs.points[relative_index].x + 1.35 * cos(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].y + 1.35* sin(cur_obs.points[relative_index].theta), cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                    }
                    FrenetState obs_sl_state;
                    CarToFrenet(obs_cartesian, obs_sl_state);
                    if(obs_sl_state.s - 2 <= traj_.Pieces_allS.back() && abs(obs_sl_state.l) < 1.3) { //障碍物与轨迹交叉
                        s_bounds[i] = std::pair(0, max(0, obs_sl_state.s - init_sl_state_.s - 2));
                        if(start_t == 0) start_t = dt_ * i;
                        else end_t = dt_ * i;
                    }
                    else {
                        s_bounds[i] = std::pair(0, max(0, path_length));
                    }
                }
            }
            if(start_t > 0) {
                total_t += (max((end_t - start_t), 0.1) + 1);
                int n_tmp = num_of_knots;
                num_of_knots = (int)(10 * total_t);
                for(int i = 0; i < num_of_knots - n_tmp; ++i) {
                    s_bounds[i].emplace_back(std::pair(0, max(0, path_length)));
                }
            }
            PiecewiseJerkSpeedProblem piecewise_jerk_problem(num_of_knots, dt_,  std::array<double, 3>{0, init_sl_state_.ds, init_sl_state_.dds});
            piecewise_jerk_problem.set_weight_x(0.0);
            piecewise_jerk_problem.set_weight_dx(0.0);
            piecewise_jerk_problem.set_weight_ddx(1);
            piecewise_jerk_problem.set_weight_dddx(10);
            piecewise_jerk_problem.set_scale_factor({1.0, 1.1, 10.0});
            piecewise_jerk_problem.set_x_bounds(std::move(s_bounds));
            piecewise_jerk_problem.set_dx_bounds(0, v_max_abs);
            piecewise_jerk_problem.set_ddx_bounds(a_min, a_max);
            piecewise_jerk_problem.set_dddx_bound(j_min, j_max);
            piecewise_jerk_problem.Optimize();
            // Extract output
            const std::vector<double>& s = piecewise_jerk_problem.opt_x();
            const std::vector<double>& ds = piecewise_jerk_problem.opt_dx();
            const std::vector<double>& dds = piecewise_jerk_problem.opt_ddx();
            // for(int )
            // std::vector<double> dx_ref_weight(num_of_knots, 10);
            // piecewise_jerk_problem.set_dx_ref(dx_ref_weight, dx_ref);
            // piecewise_jerk_problem.set_x_ref(config_.ref_s_weight(), std::move(x_ref));
            // piecewise_jerk_problem.set_penalty_dx(penalty_dx);
            // piecewise_jerk_problem.set_dx_bounds(std::move(s_dot_bounds));

            piecewise_jerk_problem.Optimize(4000);
            vector<CartesianState> best_path;
            for(int i = 0; /*i <= 6.0 && */i < 0.1 * ds.size(); i += 0.1) {
                int ind = (int) (i * 10);
                FrenetState new_sl_state(s[ind] + init_sl_state_.s, 0.0, ds[ind], dds[ind], 0.0, 0.0);
                Cartesianstate new_state;
                FrenetToCar(new_sl_state, new_state);
                best_path.emplace_back(new_state);
            }
        }
};