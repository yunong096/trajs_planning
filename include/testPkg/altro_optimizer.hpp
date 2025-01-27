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
#include "examples/quadratic_cost.hpp"
#include "altro_optimizer.hpp"
#include "nmpc2.hpp"

namespace altro {
namespace problems {

class VehicleProblem {
 public:
  static constexpr int NStates = 4;
  static constexpr int NControls = 2;

   VehicleProblem();

  enum Scenario { SCCFS };

	using ModelType = altro::problem::DiscretizedModel<altro::examples::Vehicle>;
	using CostFunType = altro::examples::QuadraticCost;

  // Problem Data
  static constexpr int HEAP = Eigen::Dynamic;
  const int n = NStates;
  const int m = NControls;

  int N = 60;
  ModelType model = ModelType(altro::examples::Vehicle());

  Eigen::Matrix3d Q = Eigen::Vector3d::Constant(NStates, 1e-2).asDiagonal();
  Eigen::Matrix2d R = Eigen::Vector2d::Constant(NControls, 1e-2).asDiagonal();
  Eigen::Matrix3d Qf = Eigen::Vector3d::Constant(NStates, 100).asDiagonal();
  Eigen::Vector3d xf = Eigen::Vector3d(1.5, 1.5, M_PI / 2);
  Eigen::Vector3d x0 = Eigen::Vector3d(0, 0, 0);
  Eigen::Vector2d u0 = Eigen::Vector2d::Constant(NControls, 0.1);
  Eigen::Vector2d uref = Eigen::Vector2d::Zero();
  std::shared_ptr<examples::QuadraticCost> qcost;
  std::shared_ptr<examples::QuadraticCost> qterm;

  double a_bnd = 1.5;  // linear velocity bound
  double delta_bnd = 30 / 180 * M_PI;  // angular velocity bound


  std::vector<double> lb;
  std::vector<double> ub;
  // altro::examples::LineConstraint obstacles;


 bool IterOpt(CartesianState& current_state,
                                                        std::vector<GlobalPathPoint>& ori_states,
                                                        std::vector<CartesianState>& ref_states,
                                                        Obstacles& obs) {
    int max_iter = 3;
    ref_states_ = ref_states;
    ori_states_ = ori_states;
    altro::augmented_lagrangian::AugmentedLagrangianiLQR<NStates, NControls> solver_al = MakeALSolver(Eigen::Vector4d(current_state.x, current_state.y, current_state.thate, current_state.speed));
    solver_al.SetPenalty(10.0);
    while(max_iter--) {
      //set obstacle constraints
      int margin = sqrt(2);
      std::vector<float> ref_states_para_front = FrontPos(ref_states);
      for (int i = 1; i <= N; ++i)
      {
          altro::examples::LineConstraint obstacles;
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
              
              double margin =sqrt(2) + 0.5; //安全距离
              //后轴约束
              Eigen::VectorXd alpha;
              double beta, d;
              Eigen::Vector2d ref_pos(ref_states[i].x, ref_states[i].y);
              d2poly(ref_pos, poly,alpha, beta, d);  // 计算到多边形的距离
              obstacles.AddObstacle(alpha(0), alpha(1), beta - margin);
              // cout << "alpha1:" << alpha(0) << "," << alpha(1) << "beta:" << beta << "d:" << d << endl;

              // //前轴约束
              ref_pos  = Eigen::Vector2d(ref_states_para_front[2 * i], ref_states_para_front[2 * i + 1]);
              d2poly(ref_pos, poly, alpha, beta, d); // 计算到多边形的距离 
              }
          }
        // cout << "set obstacle constrains success" << endl;
          for (int k = 1; k <= N; ++k) {
          altro::examples::LineConstraint obstacles;
          for (int i = 0; i < num_obstacles; ++i) {
            obstacles.AddObstacle(cx(i), cy(i), cr(i));
          }
          // obstacles = std::move(obs);
            std::shared_ptr<altro::constraints::Constraint<altro::constraints::Inequality>> obs =
                std::make_shared<altro::examples::LineConstraint>(obstacles);
            solver_al.SetConstraint(obs, k); //对第k个pt设置障碍物约束
          }
  }
      solver_al.GetOptions().verbose = altro::LogLevel::kDebug;
      solver_al.Solve();
    }

  }
  // altro::problem::Problem MakeProblem(const bool add_constraints = true);
  altro::problem::Problem MakeProblem(const bool add_constraints = true, Eigen::Vector4d x_init  = Eigen::Vector4d(0, 0, 0, 0));

  template <int n_size = NStates, int m_size = NControls>
  altro::Trajectory<n_size, m_size> InitialTrajectory();

  template <int n_size = NStates, int m_size = NControls>
  altro::ilqr::iLQR<n_size, m_size> MakeSolver(const bool alcost = false);

  template <int n_size = NStates, int m_size = NControls>
  // altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver();
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> MakeALSolver(Eigen::Vector4d x_init = Eigen::Vector4d(0, 0, 0, 0));

  void SetScenario(Scenario scenario) { scenario_ = scenario; }

  float GetTimeStep() const { return tf / N; }

 private:
  Scenario scenario_ = SCCFS;
  float tf = 6.0;
  double weight_pos = 10;
  double weight_speed = 10;
  double weight_acc = 10;
  double weight_delta = 10;

  std::vector<CartesianState> ref_states_;
  std::vector<CartesianState> ori_states_;
};

template <int n_size, int m_size>
altro::Trajectory<n_size, m_size> VehicleProblem::InitialTrajectory() {
  altro::Trajectory<n_size, m_size> Z(n, m, N);
  for (int k = 0; k < N; ++k) {
    Z.Control(k) = u0;
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

// template <int n_size, int m_size>
// altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
// VehicleProblem::MakeALSolver() {  //test调用了这里
//   altro::problem::Problem prob = MakeProblem(true);
//   altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
//   solver_al.SetTrajectory(
//       std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
//   solver_al.GetiLQRSolver().Rollout();
//   return solver_al;
// }
template <int n_size, int m_size>
altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size>
VehicleProblem::MakeALSolver(Eigen::Vector4d x_init) {
  altro::problem::Problem prob = MakeProblem(true, x_init);
  altro::augmented_lagrangian::AugmentedLagrangianiLQR<n_size, m_size> solver_al(prob);
  solver_al.SetTrajectory(
      std::make_shared<altro::Trajectory<NStates, NControls>>(InitialTrajectory()));
  solver_al.GetiLQRSolver().Rollout();

  return solver_al;
}

}  // namespace problems
}  // namespace altro