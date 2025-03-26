#include <vector>
#include "dp_planner.hpp"
// #include "modules/planning/planning_base/math/piecewise_jerk/piecewise_jerk_speed_problem.h"
#include <casadi/casadi.hpp>
using namespace std;

class ParkingTrajGenerator {
    public:
        ParkingTrajGenerator(int mode, const CartesianState& init_state,  std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline, const traj_utils::Trajectory traj,int count, const vector<CartesianState>& last_path){
            frame_count = count;
            last_path_ = last_path;
            mode_ = mode;
            df_refline_ = df_refline;
            traj_ = traj;
            CartesianState init_state_ = init_state;
            ROS_WARN("current_state: %f, %f, %f, %f, %f, %f", init_state.x, init_state.y, init_state.theta, init_state.speed, init_state.acc, init_state.kappa);
            if(mode % 2 == 1)  {//倒档
                init_state_.speed = -init_state_.speed;
                if(init_state_.speed < 0) init_state_.speed = 0;
                init_state_.acc = -init_state_.acc;
            }
            CarToFrenet(init_state_, init_sl_state_);
            ROS_WARN("current_frenet_state:  s--%f,  l--%f,  ds--%f,  dds--%f", init_sl_state_.s, init_sl_state_.l, init_sl_state_.ds, init_sl_state_.dds);
            // Initialize();
        };
        vector<CartesianState> last_path_;
        ~ParkingTrajGenerator(){};

        vector<CartesianState>  solvePieceJerkProblem() {
            // cout << "0" << endl;
            double path_length = traj_.Pieces_allS.back() - init_sl_state_.s;
            cout << "1" << endl;
            double total_t =  1.5 * max((v_max_abs * v_max_abs + path_length * a_max) / (a_max * v_max_abs) , 2.0); //不考虑障碍的估计总时间
            double start_t = -1, end_t = total_t;
            int num_of_knots = (int)(10 * total_t + 1);
            vector<pair<double, double>> s_bounds(num_of_knots);
            // cout << "2" << endl;
            for(int i = 0; i < num_of_knots; ++i) {
                for(auto& traj : MovingObs::obs_traj) {
                    if(traj.type != "PEDESTRAIN") continue;
                    CartesianState obs_cartesian;
                    int relative_index = (int)(i * dt_ * 10); //简化，实际应该调用插值函数
                    if(3 * frame_count < traj.trajs.size()) {
                        auto& cur_obs = traj.trajs[3 * frame_count];
                        obs_cartesian = CartesianState(cur_obs.points[relative_index].x, cur_obs.points[relative_index].y, cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                        // obs_cartesian = CartesianState(cur_obs.points[relative_index].x, cur_obs.points[relative_index].y, cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);                       
                    }
                    else {
                        auto& cur_obs =  traj.trajs.back();
                        obs_cartesian = CartesianState(cur_obs.points[relative_index].x , cur_obs.points[relative_index].y, cur_obs.points[relative_index].theta, cur_obs.points[relative_index].v, 0, 0);
                    }
                    FrenetState obs_sl_state;
                    CarToFrenet(obs_cartesian, obs_sl_state);
                    // ROS_WARN("obs_frenet_state:  s--%f,  l--%f,  ds--%f,  dds--%f", obs_sl_state.s, obs_sl_state.l, obs_sl_state.ds, obs_sl_state.dds);
                    if(obs_sl_state.s - 1.8 <= traj_.Pieces_allS.back() && fabs(obs_sl_state.l) < 1.8  &&  obs_sl_state.s - init_sl_state_.s > 1.8) { //障碍物与轨迹交叉
                        s_bounds[i] = std::pair<double, double>(0.0, max(0.0, obs_sl_state.s - init_sl_state_.s - 1.8)); //1 + 0.5 + 0.3
                        cout  << "检测到障碍轨迹与路径相交 : " <<  obs_sl_state.s - init_sl_state_.s - 1.8 << endl;
                        if(start_t == -1) start_t = dt_ * i;
                        end_t = dt_ * i;
                    }
                    else {
                        s_bounds[i] = std::pair<double, double>(0.0, max(0.0, path_length));
                    }
                }
            }
            // cout << "完成初步s上下限设置和总时间设置" << endl;
            if(start_t >= 0) {
                // total_t += 1.5 * (max((end_t - start_t), 0.1));
                total_t += 1.5 * (max((end_t - start_t)+0.1, 0.1));
                cout << "更新总时间: add " << end_t - start_t << endl;
                int n_tmp = num_of_knots;
                num_of_knots = (int)(10 * total_t + 1);
                for(int i = n_tmp; i < num_of_knots; ++i) {
                    s_bounds.emplace_back(std::pair<double, double>(0.0, max(0.0, path_length)));
                }
            }
            cout << "完成s上下限设置和总时间设置, 最终点的个数为： " <<  num_of_knots << endl;
        
        int N_ = num_of_knots;
        Opti opti = Opti();
        Slice all;
        MX cost = 0;
        MX S,DS,DDS,DDDS;
        S = opti.variable(1, N_);
        DS = opti.variable(1, N_);
        DDS = opti.variable(1, N_);
        DDDS = opti.variable(1, N_);

        DM DS_ref =DM::zeros(1, N_);   // 定义一个4行N_+1列的参数矩阵X_ref 注意：是已知的常量
        DM DDS_ref =DM::zeros(1, N_);   // 定义一个4行N_+1列的参数矩阵X_ref 注意：是已知的常量
        DM DDDS_ref =DM::zeros(1, N_);   // 定义一个4行N_+1列的参数矩阵X_ref 注意：是已知的常量
        // DS_ref(0, 0) =  init_sl_state_.ds;
        // DDS_ref(0, 0) =  init_sl_state_.dds;
        for (int i = 1; i < N_ ; ++i) {
            DS_ref(0, 0) =  2;
        }
        // cout << "set ref state success" << endl;
        double w_s = 0.0, w_ds = 5.0, w_dds = 4.0, w_ddds = 4.0;
        DM Q_ = DM::zeros(1,1); //索引之前初始化size
        DM R_ = DM::zeros(1,1);
        DM S_ = DM::zeros(1,1);
        Q_(0,0) = w_ds;
        R_(0,0) = w_dds;
        S_(0,0) = w_ddds;
        //set costfunction
        for (int i = 0; i < N_; ++i) {
            MX DS_0 = DS(all, i) - DS_ref(all, i); 
            MX DDS_0 = DDS(all, i) - DDS_ref(all, i); 
            MX DDDS_0 = DDDS(all, i) - DDDS_ref(all, i); 
            cost += MX::mtimes({DS_0.T(), Q_, DS_0}); //目标函数是状态误差和控制输入的成本之和
            cost += MX::mtimes({DDS_0.T(), R_, DDS_0});
            cost += MX::mtimes({DDDS_0.T(), S_, DDDS_0});
        }
        opti.minimize(cost); //opti.minimize 用于定义优化问题的目标函数
        // cout << "set cost success" << endl;

        //kinematic constrains opti.subject_to 用于添加约束条件到优化问题中
        for (int i = 0; i < N_ - 1; ++i) {
            //连续性约束
            opti.subject_to(DDDS(0,i)== (DDS(0,i+1) - DDS(0,i)) / dt_);  // 
            opti.subject_to(DS(0,i + 1)== DS(0,i)  + dt_*DDS(0,i) + 1/2 *dt_ * dt_ * DDDS(0,i));  //
            opti.subject_to(S(0,i + 1)== S(0,i) + dt_ *  DS(0,i)  + 1/2 * dt_ * dt_ * DDS(0,i) + 1/6 * dt_ * dt_* dt_ * DDDS(0,i));  // 
        }
        // cout << "set kinematic constrains success" << endl;

        //init value 初始化，初始状态赋值第一列所有行
        opti.subject_to(S(0, 0) == 0);//第一个时间步（索引为0）表示当前时刻的状态
        opti.subject_to(DS(0, 0) == init_sl_state_.ds);//第一个时间步（索引为0）表示当前时刻的状态
        opti.subject_to(DDS(0, 0) == init_sl_state_.dds);//第一个时间步（索引为0）表示当前时刻的状态
        
        opti.subject_to(S(0, N_-1) == path_length - 0.0001);//第一个时间步（索引为0）表示当前时刻的状态
        opti.subject_to(DS(0, N_-1) == 0.0);//第一个时间步（索引为0）表示当前时刻的状态
        // opti.subject_to(DDS(0, N_-1) == 0);//第一个时间步（索引为0）表示当前时刻的状态

        for(int i = 0; i < N_; ++i) {
            opti.subject_to(s_bounds[i].first <= S(0,i) <= s_bounds[i].second);
            // opti.subject_to(0<= S(0,i) <= path_length);
            opti.subject_to(0 <= DS(0,i) <= 2);
            opti.subject_to(- 1<= DDS(0,i) <= 1);
        }
    
        //set solver
        casadi::Dict solver_opts; // 设置求解器选项
        solver_opts["expand"] = true; //MX change to SX for speed up
        solver_opts["ipopt.max_iter"] = 1000;
        solver_opts["ipopt.print_level"] = 0;
        solver_opts["print_time"] = 0;
        solver_opts["ipopt.acceptable_tol"] = 1e-6;
        solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

        opti.solver("ipopt", solver_opts);

        // auto start_time = std::chrono::high_resolution_clock::now();

        // solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

        // auto end_time = std::chrono::high_resolution_clock::now();
        // // 计算时间间隔 
        // std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        // // 输出时间间隔
        // ROS_WARN( "a NMPC problem solve time: %f s" ,  elapsed_seconds.count() );

            // piecewise_jerk_problem.Optimize(4000);
            // 缓存 solution_->value(X) 和 solution_->value(U) 的结果
        std::unique_ptr<casadi::OptiSol>  solution_; // = std::make_unique<casadi::OptiSol>(opti.solve());
        vector<CartesianState> best_path;
        try {
            solution_ = std::make_unique<casadi::OptiSol>(opti.solve());
        } catch (const casadi::CasadiException& e) {
            std::cerr << "Solver failed: " << e.what() << std::endl;
        
            // 调试变量值
            auto S_val = opti.debug().value(S);
            auto DS_val = opti.debug().value(DS);
            auto DDS_val = opti.debug().value(DDS);
            auto DDDS_val = opti.debug().value(DDDS);
        
            std::cerr << "S values: " << S_val << std::endl;
            std::cerr << "DS values: " << DS_val << std::endl;
            std::cerr << "DDS values: " << DDS_val << std::endl;
            std::cerr << "DDDS values: " << DDDS_val << std::endl;
            best_path.assign(last_path_.begin() +3, last_path_.end());
            for (int i = best_path.size(); i < 61; ++i) {
                best_path.emplace_back(best_path.back());
            }
            return best_path;
        }
        cout << "finish calc" << endl;
        const auto& s_values = solution_->value(S);
        const auto& ds_values = solution_->value(DS);
        const auto& dds_values = solution_->value(DDS);
        
        if (N_ >= 61) {
            // int idx = 0;
            // for (int i = 0; i < N_; ++i) {
            for (int i = 0; i < N_; ++i) {
                CartesianState state;
                double s = static_cast<double>(s_values(0, i));
                double ds = static_cast<double>(ds_values(0, i));
                double dds = static_cast<double>(dds_values(0, i));
                // if(mode_ % 2 == 1) {
                //     cout << "ds0: " << ds << endl;
                //     cout << "dds0: " << dds << endl;
                // }
                FrenetState new_sl_state(s + init_sl_state_.s, 0.0, ds, dds, 0.0, 0.0);
                CartesianState new_state;
                getRefFromFrenet(new_sl_state, new_state);
                // cout << new_state.x << ", " << new_state.y << endl;
                if(mode_ % 2 == 1) {
                    new_state.speed = -new_state.speed;
                    new_state.acc = -new_state.acc;
                }
                best_path.emplace_back(new_state);
            }
            cout << best_path[0].x << ", " << best_path[0].y << ", " << best_path[0].theta << ", " <<  best_path[0].speed << ", "<<  best_path[0].acc << ", " << best_path[0].kappa << endl;
            cout << best_path[1].x << ", " << best_path[1].y << ", " << best_path[1].theta << ", "<< best_path[1].speed << ", " <<  best_path[0].acc << ", "<< best_path[1].kappa << endl;
            cout << best_path[2].x << ", " << best_path[2].y << ", " << best_path[2].theta << ", "<< best_path[2].speed << ", " <<  best_path[0].acc << ", "<< best_path[2].kappa << endl;
            cout << best_path[3].x << ", " << best_path[3].y << ", " << best_path[3].theta << ", "<< best_path[3].speed << ", " <<  best_path[0].acc << ", "<< best_path[3].kappa << endl;
            cout << best_path[4].x << ", " << best_path[4].y << ", " << best_path[4].theta << ", "<< best_path[4].speed << ", " <<  best_path[0].acc << ", "<< best_path[4].kappa << endl;
            return best_path;
        }
        for (int i = 0; i < N_; ++i) {
            CartesianState state;
            double s = static_cast<double>(s_values(0, i));
            double ds = static_cast<double>(ds_values(0, i));
            double dds = static_cast<double>(dds_values(0, i));
            FrenetState new_sl_state(s + init_sl_state_.s, 0.0, ds, dds, 0.0, 0.0);
            CartesianState new_state;
            getRefFromFrenet(new_sl_state, new_state);
            // cout << new_state.x << ", " << new_state.y << endl;
            if(mode_ % 2 == 1) {
                new_state.speed = -new_state.speed;
                new_state.acc = -new_state.acc;
            }
            best_path.emplace_back(new_state);
        }
        for (int i = 0; i < 61 - N_; ++i) {
            best_path.emplace_back(best_path.back());
        }
         cout << best_path[0].x << ", " << best_path[0].y << ", " << best_path[0].theta << ", " <<  best_path[0].speed << ", " << best_path[0].kappa << endl;
            cout << best_path[1].x << ", " << best_path[1].y << ", " << best_path[1].theta << ", "<< best_path[1].speed << ", " << best_path[1].kappa << endl;
            cout << best_path[2].x << ", " << best_path[2].y << ", " << best_path[2].theta << ", "<< best_path[2].speed << ", " << best_path[2].kappa << endl;
            cout << best_path[3].x << ", " << best_path[3].y << ", " << best_path[3].theta << ", "<< best_path[3].speed << ", " << best_path[3].kappa << endl;
            cout << best_path[4].x << ", " << best_path[4].y << ", " << best_path[4].theta << ", "<< best_path[4].speed << ", " << best_path[4].kappa << endl;
        return best_path;
    }

    private:
        std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline_ = nullptr;
        traj_utils::Trajectory traj_;
        vector<CartesianState> best_path_;
        FrenetState init_sl_state_;
        CartesianState init_state_;
        int  frame_count= 0;

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
            //  cout << "min_ind: " << min_ind << ", min_dist: " << min_dist  << ", car_state:" << car_state.x << ", " << car_state.y << endl;
    
            if(min_ind > 0 && min_ind < pos_matrix.cols() - 1) {
                // ROS_WARN("enter inner");
                double t = traj_.getDurations()[min_ind - 1];
                Vector2d min_res = traj_.findMinPtInSegment(min_ind - 1, cur_pos, t / 2);

                int ind = 0;
                // cout << min_res(1) << " min_dis0: 0" <<min_res(0) << endl;

                t = traj_.getDurations()[min_ind];
                Vector2d min_res2 = traj_.findMinPtInSegment(min_ind, cur_pos, t/2);
                // cout << min_res2(1) <<  " min_dis0: " <<min_res2(0) << endl;

                min_res = min_res(0) < min_res2(0) ? min_res : min_res2;
                ind = min_res(0) < min_res2(0) ? min_ind-1 : min_ind;
                ref_state = traj_.getState(min_res(1), ind);
                // }
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
        
            // best_path_ref_.emplace_back(ref_state);
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

        void getRefFromFrenet(const FrenetState& fre_state, CartesianState& car_state) {
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
            car_state.x = ref_state.x;
            car_state.y = ref_state.y;
            car_state.theta = ref_state.theta;
            car_state.speed = fre_state.ds;
            car_state.acc =  fre_state.dds;
            car_state.kappa = ref_state.kappa;
        }
        int mode_  = 0;
       
};