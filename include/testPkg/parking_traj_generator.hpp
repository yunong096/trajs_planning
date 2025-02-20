#include <vector>
#include "dp_planner.hpp"

class ParkingTrajGenerator {
    public:
        ParkingTrajGenerator(){};

    private:
        ~ParkingTrajGenerator(){};
        std::shared_ptr<plan_manage::PolyTrajOptimizer> df_refline_ = nullptr;
        traj_utils::Trajectory traj_;
        vector<CartesianState> best_path_;
        FrenetState init_sl_state_;

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
};