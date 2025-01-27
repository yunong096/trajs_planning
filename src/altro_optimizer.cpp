// Copyright [2021] Optimus Ride Inc.

#include "examples/problems/vehicle.hpp"

namespace altro {
namespace problems {

VehicleProblem::VehicleProblem() {
}

altro::problem::Problem VehicleProblem::MakeProblem(const bool add_constraints, , Eigen::Vector4d x_init) {
  altro::problem::Problem prob(N);

  // goal = std::make_shared<altro::examples::GoalConstraint>(xf);

  float h;
   if (scenario_ ==  SCCFS) {
    tf = 6.0;
    h = GetTimeStep();

    // Q.diagonal().setConstant(1.0 * h);
    Q << weight_pos, 0,  0, 0, 
         0, weight_pos * h, 0, 0, 
         0, 0, 0, 0, 
         0, 0, 0, weight_speed * h;
    // R.diagonal().setConstant(5 * h);
    R << weight_acc * h, 0,
         0, weight_delta * h;
    // Qf.diagonal().setConstant(10.0);
    Qf << weight_pos, 0,  0, 0, 
         0, weight_pos * h, 0, 0, 
         0, 0, 0, 0, 
         0, 0, 0, weight_speed * h;
    x0 = x_init;
    xf << 100, 0, 0, 0;
    u0 << 0.0, 10.0;
    uref << 0.0, 0.0;

    lb = {0, -3};
    ub = {3, +3};
  }

  // Cost Function
  for (int k = 0; k < N; ++k) {
    qcost =
        std::make_shared<examples::QuadraticCost>(examples::QuadraticCost::LQRCost(Q, R, xf, uref)); //代价为距离目标位置和控制量大小
    prob.SetCostFunction(qcost, k);
  }
  qterm = std::make_shared<examples::QuadraticCost>(
      examples::QuadraticCost::LQRCost(Qf, R * 0, xf, uref, true));
  prob.SetCostFunction(qterm, N);

  // Dynamics
  for (int k = 0; k < N; ++k) {
    prob.SetDynamics(std::make_shared<ModelType>(model), k); //模型约束
  }

  // Constraints
  if (add_constraints) {
    for (int k = 0; k < N; ++k) {
      prob.SetConstraint(std::make_shared<altro::examples::ControlBound>(lb, ub), k); //上下界约束
    }
    prob.SetConstraint(std::make_shared<examples::GoalConstraint>(xf), N); //终点约束
  }

  // Initial State
  prob.SetInitialState(x0);

  return prob;
}

}  // namespace problems
}  // namespace altro