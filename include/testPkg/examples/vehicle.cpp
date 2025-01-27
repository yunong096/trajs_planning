// Copyright [2021] Optimus Ride Inc.

#include "examples/vehicle.hpp"

#include <cmath>

#include "altro/utils/utils.hpp"

namespace altro {
namespace examples {

void Vehicle::Evaluate(const VectorXdRef& x, const VectorXdRef& u, const float t,
                               Eigen::Ref<VectorXd> xdot) {
  ALTRO_UNUSED(t);
  double theta = x(2);  // angle
  double speed = x(3);  // speed
  double acc = u(0);      // acc
  double delta = u(1);  // steer
  xdot(0) = speed * cos(theta);
  xdot(1) = speed * sin(theta);
  xdot(2) = speed * tan(delta) / wheelbase;
  xdot(3) = acc;
}

void Vehicle::Jacobian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                        Eigen::Ref<MatrixXd> jac) {
  ALTRO_UNUSED(t);
  double theta = x(2);  // angle
  double speed = x(3);  // speed
  double delta = u(1);  // steer
  jac(0, 2) = -speed * sin(theta);
  jac(0, 3) = cos(theta);
  jac(1, 2) = speed * cos(theta);
  jac(1, 3) = sin(theta);
  jac(2, 3) = tan(delta) / wheelbase;
  jac(2, 5) = speed / (wheelbase * cos(delta) * cos(delta));
  jac(3, 4) = 1;
}

void Vehicle::Hessian(const VectorXdRef& x, const VectorXdRef& u, const float t,
                       const VectorXdRef& b, Eigen::Ref<MatrixXd> hess) {
  ALTRO_UNUSED(t);
  double theta = x(2);  // angle
  double v = x(3);      // linear velocity
  hess(2, 2) = -b(0) * v * cos(theta) - b(1) * v * sin(theta);
  hess(2, 3) = -b(0) * sin(theta) + b(1) * cos(theta);
  hess(3, 2) = hess(2, 3);
}

}  // namespace examples
}  // namespace altro