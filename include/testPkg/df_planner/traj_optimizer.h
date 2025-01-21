#pragma once

#include <vector>

#include "df_planner/traj_container.h"
#include "df_planner/traj_utils.h"

namespace plan_manage {

class TrajOptimizer {
 public:
  TrajOptimizer() {}
  ~TrajOptimizer() {}
  int piece_ = 10;
  int corridor_per_piece = 6;
  double ego_buffer = 0.0;
  traj_utils::MinJerkOpt jerkOpt;
  traj_utils::VehicleParam vehicle_param_;
  // npp::planning_math::PncKDPath *ara_kd_path_;
  void setParam();
  /* main planning API */
  bool OptimizeTrajectory(
      const traj_utils::FlatTrajData &traj,
      GlobalPath& refline,
      const vector<GlobalPathPoint>& ori_path, 
      const vector<CartesianState>& last_path, 
      const std::vector<std::vector<Eigen::Vector3d>> &rb_hPoly_container,
      const std::vector<std::vector<Eigen::Vector3d>> &fb_hPoly_container); 
  std::vector<CartesianState> GetResult(double gap); //const;
  std::vector<CartesianState> GetResult(); //const;
  Eigen::Vector2d UpdateRefIndex(const Eigen::Vector2d &sigma);
  Eigen::Vector2d UpdateRefIndex(const Eigen::Vector2d &sigma, int index);
  void SetDebugMode(const bool is_debug_mode) {
    is_debug_mode_ = is_debug_mode;
  }
  Eigen::Vector2d UpdateConsistIndex(const Eigen::Vector2d &sigma, int index);

  double getResultDuration();
  double final_cost_ = 0;

 private:
  /* optimization parameters */
  // 重复定义，直接用proto表示的参数
  double wei_time_ = 0.0;  // time weight
  double wei_feas_ = 250.0;
  double wei_obs_ = 400.0;
  double wei_lb_ = 4.0;
  double wei_ref_ = 500.0;
  double wei_consist_ = 400.0;
  double wei_cur_ = 300.0;
  double wei_jerk_ = 10.0;
  double wei_acc_ = 50.0;
  /*dynamic constraints*/
  double max_forward_vel_ = 15/3.6;//35.0;
  double max_forward_cur = 0.2138; //kappa
  double max_forward_acc = 2.0; //3.0
  double max_phidot_ = 1.0; //10000.0;
  double max_latacc_ = 4.0; //2.0
  std::vector<Eigen::Vector2d> vec_le_;
  Eigen::Vector2d vec_front_;
  Eigen::Matrix<double, 2, 2> B_h;

  Eigen::MatrixXd iniState_container;
  Eigen::MatrixXd finState_container;
  // std::vector<Eigen::Matrix<double, 4, 4>> rb_sfc_container;
  // std::vector<Eigen::Matrix<double, 4, 4>> lb_sfc_container;
  std::vector<std::vector<Eigen::Vector3d>> rb_sfc_container;
  std::vector<std::vector<Eigen::Vector3d>> fb_sfc_container;
  // std::vector<Eigen::Vector3d> refline_container;
  // int trajnum;            // segment数量
  int iter_num_ = 0;  // iteration of the solver
  double mini_T = 0.05;
  bool is_debug_mode_ = false;
  Eigen::MatrixXd cost_vec;
  static void costFunctionCallback(void *func_data, const Eigen::VectorXd &x,
                                   Eigen::VectorXd &grad,
                                   Eigen::MatrixXd *cost);

  void VirtualTGradCost(const double RT, const double VT, const double gdRT,
                        double *gdVT, double *costT);

  /* mappings between real world time and unconstrained virtual time */

  double RealT2VirtualT(const double RT);
  double VirtualT2RealT(const double VT);

  void addPVAGradCost2CT(Eigen::MatrixXd &costs);
  void positiveSmoothedL1(const double x, double *f, double *df);

  // DFOutput df_output{};
  GlobalPath refline_;
  vector<GlobalPathPoint> ori_path_;
  vector<CartesianState> last_path_;
  
};

}  // namespace plan_manage
