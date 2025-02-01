// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <fmt/format.h>
#include <fmt/ostream.h>

#include "altro/eigentypes.hpp"
#include "altro/problem/problem.hpp"

#include "altro/augmented_lagrangian/al_solver.hpp"
#include "altro/augmented_lagrangian/al_problem.hpp"
#include "altro/common/trajectory.hpp"
#include "altro/ilqr/ilqr.hpp"
#include "altro/problem/discretized_model.hpp"
#include "examples/basic_constraints.hpp"
#include "examples/obstacle_constraints_line.hpp"
#include "examples/obstacle_constraints_line_front.hpp"
#include "examples/quadratic_cost.hpp"
#include "examples/vehicle.hpp"
#include "nmpc2.hpp"

namespace altro {
namespace problems {

class VehicleProblem {
 public:
  static constexpr int NStates = 4;
  static constexpr int NControls = 2;

   VehicleProblem(int count);

  enum Scenario { SCCFS };

	using ModelType = altro::problem::DiscretizedModel<altro::examples::Vehicle>;
	using CostFunType = altro::examples::QuadraticCost;

  // Problem Data
  static constexpr int HEAP = Eigen::Dynamic;
  const int n = NStates;
  const int m = NControls;

  int N = 60;
  ModelType model = ModelType(altro::examples::Vehicle());

  Eigen::Matrix4d Q = Eigen::Vector4d::Constant(NStates, 1e-2).asDiagonal();
  Eigen::Matrix2d R = Eigen::Vector2d::Constant(NControls, 1e-2).asDiagonal();
  Eigen::Matrix4d Qf = Eigen::Vector4d::Constant(NStates, 100).asDiagonal();
  Eigen::Vector4d xf = Eigen::Vector4d(1.5, 1.5, M_PI / 2, 0);
  Eigen::Vector4d x0 = Eigen::Vector4d(0, 0, 0, 0);
  Eigen::Vector2d u0 = Eigen::Vector2d::Constant(NControls, 0.1);
  Eigen::Vector2d uref = Eigen::Vector2d::Zero();
  std::shared_ptr<examples::QuadraticCost> qcost;
  std::shared_ptr<examples::QuadraticCost> qterm;

  double a_bnd = 1.5;  // linear velocity bound
  double delta_bnd = 30 / 180 * M_PI;  // angular velocity bound


  std::vector<double> lb;
  std::vector<double> ub;
   std::vector<double> lb_state{0,0,-std::numeric_limits<double>::infinity(),0};
  std::vector<double> ub_state{140,80,std::numeric_limits<double>::infinity(),15/3.6};
  // altro::examples::LineConstraint obstacles;


 bool IterOpt(CartesianState& current_state,
                                                        std::vector<GlobalPathPoint>& ori_states,
                                                        std::vector<CartesianState>& ref_states,
                                                        Obstacles& obs) {
    int max_iter = 3;
    u0 << current_state.acc, atan2(current_state.kappa * 2.7, 1);
    ref_states_ = ref_states;
    ori_states_ = ori_states;
    Eigen::Vector4d x_init(current_state.x, current_state.y, current_state.theta, current_state.speed);
    x0 = x_init;
    double dt_ = GetTimeStep();
    while(max_iter--) {
      altro::problem::Problem prob = MakeProblem(true);
      //set obstacle constraints
      double margin = sqrt(2);
      std::vector<float> ref_states_para_front = FrontPos(ref_states_);
      for (int i = 1; i <= N; ++i)
      {
          altro::examples::LineConstraint obstacles;
          altro::examples::LineFrontConstraint front_obstacles;
          // for j = 0; j < obs_num; ++j
          for(auto j : obs.getMoveInds()) { 
              //获取障碍物位置
              //测试用，只有一个动态障碍
              if(current_state.x > 60 && j == 9) continue;
              if(current_state.y < 65 && j == 10) continue;
              auto cur_obs = obs.getObs()[j];
              Eigen::MatrixXd poly(2, cur_obs.vertex_x.size());
              for (int k = 0; k < cur_obs.vertex_x.size(); ++k) {
                  poly(0, k) = cur_obs.vertex_x[k];
                  poly(1, k) = cur_obs.vertex_y[k];
              }
              poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.5 *  frame_count + dt_ * i); 
              
              //后轴约束
              Eigen::VectorXd alpha;
              double beta, d;
              Eigen::Vector2d ref_pos(ref_states_[i].x, ref_states_[i].y);
              d2poly(ref_pos, poly,alpha, beta, d);  // 计算到多边形的距离
              obstacles.AddObstacle(Vector3d(alpha(0), alpha(1), beta - margin));
              // cout << "alpha1:" << alpha(0) << "," << alpha(1) << "beta:" << beta << "d:" << d << endl;

              // //前轴约束
              ref_pos  = Eigen::Vector2d(ref_states_para_front[2 * i], ref_states_para_front[2 * i + 1]);
              d2poly(ref_pos, poly, alpha, beta, d); // 计算到多边形的距离 
              front_obstacles.AddObstacle(Vector3d(alpha(0), alpha(1), beta - margin));
              }
          std::shared_ptr<altro::constraints::Constraint<altro::constraints::Inequality>> obs =
              std::make_shared<altro::examples::LineConstraint>(obstacles);
          // prob.SetConstraint(obs, i); //对第k个pt设置障碍物约束
          std::shared_ptr<altro::constraints::Constraint<altro::constraints::Inequality>> front_obs =
              std::make_shared<altro::examples::LineFrontConstraint>(front_obstacles);
          // prob.SetConstraint(front_obs, i); //对第k个pt设置障碍物约束   
      }
      altro::augmented_lagrangian::AugmentedLagrangianiLQR<NStates, NControls> solver_al(prob);
      solver_al.SetTrajectory(std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
      solver_al.GetiLQRSolver().Rollout();
      solver_al.SetPenalty(10.0);
      solver_al.GetOptions().verbose = altro::LogLevel::kDebug;

      auto start_time = std::chrono::high_resolution_clock::now();
      solver_al.Solve();
      // 获取结束时间点
      auto end_time = std::chrono::high_resolution_clock::now();
      // 计算时间间隔
      std::chrono::duration<double> elapsed_seconds = end_time - start_time;
      // 输出时间间隔
      std::cout << "程序执行时间: " << elapsed_seconds.count() << " 秒" << std::endl;

      std::vector<CartesianState> opt_path;
      for (int k = 0; k <= N; ++k) {
        double px = solver_al.GetiLQRSolver().GetTrajectory()->State(k)[0];
        double py = solver_al.GetiLQRSolver().GetTrajectory()->State(k)[1];
        double ptheta = solver_al.GetiLQRSolver().GetTrajectory()->State(k)[2];
        double pspeed = solver_al.GetiLQRSolver().GetTrajectory()->State(k)[3];
        double pacc = 0.0;
        double pdelta = 0.0;
        if(k < N) {
          pacc = solver_al.GetiLQRSolver().GetTrajectory()->Control(k)[0];
          pdelta = solver_al.GetiLQRSolver().GetTrajectory()->Control(k)[1];
        }
        else{
          pacc = solver_al.GetiLQRSolver().GetTrajectory()->Control(k - 1)[0];
          pdelta = solver_al.GetiLQRSolver().GetTrajectory()->Control(k - 1)[1];
        }
        pdelta = normalizeAngle(pdelta);
        CartesianState new_state(px, py, ptheta, pspeed, pacc, tan(pdelta)/2.7 );
        opt_path.emplace_back(new_state);
      }
      double diff = 0;
      for (int i = 0; i < N + 1; ++i) {
          diff += pow(opt_path[i].x - ref_states_[i].x, 2) + pow(opt_path[i].y - ref_states_[i].y, 2);
      }
      cout << "diff: " << sqrt(diff) << endl;
      ref_states_ = opt_path;
      if(sqrt(diff) < 1.0) {
          return true;
      }
    }
    return false;
  }
  
  std::vector<CartesianState> getFinalPath() {return ref_states_;}
  altro::problem::Problem MakeProblem(const bool add_constraints = true);
  // altro::problem::Problem MakeProblem(const bool add_constraints = true, Eigen::Vector4d x_init  = Eigen::Vector4d(0, 0, 0, 0));

  template <int n_size = NStates, int m_size = NControls>
  altro::Trajectory<n_size, m_size> InitialTrajectory();

  template <int n_size = NStates, int m_size = NControls>
  altro::ilqr::iLQR<n_size, m_size> MakeSolver(const bool alcost = false);

  template <int n_size = NStates, int m_size = NControls>
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver();
  // altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver(Eigen::Vector4d x_init = Eigen::Vector4d(0, 0, 0, 0));

  void SetScenario(Scenario scenario) { scenario_ = scenario; }

  float GetTimeStep() const { return tf / N; }

 private:
  Scenario scenario_ = SCCFS;
  float tf = 6.0;
  double weight_pos = 1000;
  double weight_speed = 10;
  double weight_acc = 100;
  double weight_delta = 10;
  int frame_count = 0;

  std::vector<CartesianState> ref_states_;
  std::vector<GlobalPathPoint> ori_states_;
  void d2poly(const Eigen::Vector2d& point, const Eigen::MatrixXd& poly, Eigen::VectorXd& L, double& S, double& d) {
        d = numeric_limits<double>::infinity(); // 初始化距离为无穷大
        S = 0;
        L = Eigen::VectorXd(2);
        int nside = poly.cols();
        int ii = -1;

        for (int i = 0; i < nside; ++i) {
            Eigen::VectorXd p1 = poly.col(i);
            Eigen::VectorXd p2 = poly.col((i + 1) % nside);

            // 计算三边的距离
            double trid[3];
            trid[0] = (p1 - p2).norm();
            trid[1] = (p1 - point).norm();
            trid[2] = (p2 - point).norm();

            Eigen::Vector2d Lr(p1(1) - p2(1), p2(0) - p1(0));
            double Sr = -p1(0) * p2(1) + p2(0) * p1(1);
            double vd = fabs(Lr(0) * point(0) + Lr(1) * point(1) - Sr) / (trid[0] + numeric_limits<double>::epsilon());

            if (pow(trid[1], 2) > pow(trid[0], 2) + pow(trid[2], 2)) {
                vd = trid[2];
                Lr = point - p2;
                Sr = Lr.dot(p2);
            }

            if (pow(trid[2], 2) > pow(trid[0], 2) + pow(trid[1], 2)) {
                vd = trid[1];
                Lr = point - p1;
                Sr = Lr.dot(p1);
            }

            if (vd < d) {
                d = vd;
                L = Lr;
                S = Sr;
                ii = i;
            }
        }

        // 归一化 L
        double nL = L.norm();
        L /= nL;
        S /= nL;

        // 检查法线方向
        if (L.dot(poly.col((ii + 2) % nside)) < S) {
            L = -L;
            S = -S;
        }

        if (d == 0) {
            return;
        }

        // 计算是否在多边形内部
        double area = 0, polyarea = 0;
        for (int i = 0; i < nside; ++i) {
            area += triArea(point, poly.col(i), poly.col((i + 1) % nside));
        }
        for (int i = 1; i < nside - 1; ++i) {
            polyarea += triArea(poly.col(0), poly.col(i), poly.col((i + 1) % nside));
        }

        if (fabs(polyarea - area) < 0.01) {
            d = -d;
        }
    }

  double triArea(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2, const Eigen::Vector2d& p3) {
		return 0.5 * fabs(p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) + p3(0) * (p1(1) - p2(1)));
	}
  std::vector<float> FrontPos(const vector<CartesianState> &x) {
		std::vector<float> z_new(2 * N + 2);
		for (int i = 0; i < N + 1; ++i) {
			z_new[i * 2] = x[i].x + 2.7 * cos(x[i].theta);
			z_new[i * 2 + 1] = x[i].y + 2.7 * sin(x[i].theta);
		}
		return z_new;
	}
  double normalizeAngle(double angle) {
      while (angle > M_PI) {
          angle -= 2.0 * M_PI;
      }
      while (angle < -M_PI) {
          angle += 2.0 * M_PI;
      }
      return angle;
  }
};

template <int n_size, int m_size>
altro::Trajectory<n_size, m_size> VehicleProblem::InitialTrajectory() {
  altro::Trajectory<n_size, m_size> Z(n, m, N);
  for (int k = 0; k <= N; ++k) {
    Z.State(k) = Eigen::Vector4d(ref_states_[k].x, ref_states_[k].y, ref_states_[k].theta, ref_states_[k].speed);
    // Z.Control(k) = u0; //重新设置初始化
    if(k < N)
      Z.Control(k) = Eigen::Vector2d(ref_states_[k].acc, atan2(ref_states_[k].kappa * 2.7, 1));
  }
  float h = GetTimeStep(); 
  Z.SetUniformStep(h);
  return Z;
}

template <int n_size, int m_size>
altro::ilqr::iLQR<n_size, m_size> VehicleProblem::MakeSolver(const bool alcost) {
  altro::problem::Problem prob = MakeProblem();
  if (alcost) {
    prob = altro::augmented_lagrangian::BuildAugLagProblem<n_size, m_size>(prob);
  }
  altro::ilqr::iLQR<n_size, m_size> solver(prob);

  std::shared_ptr<altro::Trajectory<n_size, m_size>> traj_ptr =
      std::make_shared<altro::Trajectory<n_size, m_size>>(InitialTrajectory<n_size, m_size>());

  solver.SetTrajectory(traj_ptr);
  solver.Rollout();
  return solver;
}

template <int n_size, int m_size>
altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
VehicleProblem::MakeALSolver() {  //test调用了这里
  altro::problem::Problem prob = MakeProblem(true);
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
  solver_al.SetTrajectory(
      std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
  solver_al.GetiLQRSolver().Rollout();
  return solver_al;
}
// template <int n_size, int m_size>
// altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
// VehicleProblem::MakeALSolver(Eigen::Vector4d x_init) {
//   altro::problem::Problem prob = MakeProblem(true, x_init);
//   altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
//   solver_al.SetTrajectory(
//       std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
//   solver_al.GetiLQRSolver().Rollout();

//   return solver_al;
// }

}  // namespace problems
}  // namespace altro