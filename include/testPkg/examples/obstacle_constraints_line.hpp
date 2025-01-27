// Copyright [2021] Optimus Ride Inc.

#pragma once

#include <cmath>
#include <vector>

#include "altro/constraints/constraint.hpp"
#include "altro/eigentypes.hpp"
#include "altro/utils/utils.hpp"

namespace altro {
namespace examples {

/**
 * @brief Constraint that keeps the position away from a list of different 
 * circular regions. Assumes that the robot is a point (i.e. you need to add a 
 * collision buffer yourself) 
 * 
 */
class LineConstraint : public constraints::Constraint<constraints::NegativeOrthant> {
 public:

  /**
   * @brief Add a circular obstacle to avoid. Arguments are passed to the constructor 
   * of the Circle object.
   */
  // template <class... Args>
  // void AddObstacle(Args&& ...args) { 
  //   obstacles_.emplace_back(std::forward<Args>(args)...);
  // }

    void AddObstacle(Eigen::Vector3d obs) { 
    line_constraints_.emplace_back(obs);
  }

  /**
   * @brief Set which state indices correspond to the x and y positions.
   * 
   * By default, the x-coordinate is assume to be the first state, and the 
   * y-coordinate to be the second.
   * 
   * @param x_index Index of the x-coordinate (0 <= x_index < state_dimension).
   * @param y_index Index of the y-coordinate (0 <= y_index < state_dimension).
   */
  void SetXYIndices(int x_index, int y_index) { 
    ALTRO_ASSERT(x_index >= 0, "X index must be non-negative");
    ALTRO_ASSERT(y_index >= 0, "Y index must be non-negative");
    x_index_ = x_index; 
    y_index_ = y_index; 
  }

  int OutputDimension() const override { return line_constraints_.size(); }

  void Evaluate(const VectorXdRef& x, const VectorXdRef& /*u*/,
                Eigen::Ref<VectorXd> c) override {
    const double px = x(x_index_);
    const double py = x(y_index_);
    for (size_t i = 0; i < line_constraints_.size(); ++i) {
      // store negative since it must be in the negative orthant
      c(i) = (line_constraints_[i](0) * px + line_constraints_[i](1) * py - line_constraints_[i](2));
    }
  }

  void Jacobian(const VectorXdRef& x, const VectorXdRef& /*u*/,
                Eigen::Ref<MatrixXd> jac) override {
    const double px = x(x_index_);
    const double py = x(y_index_);
    for (size_t i = 0; i < line_constraints_.size(); ++i) {
      // TODO(bjackson): Store Jacobians in row-major format so that the 
      // raw columns can be passed to each obstacle to compute the gradient
      jac(i, 0) = line_constraints_[i](0); //x系数
      jac(i, 1) = line_constraints_[i](1); //y系数
    }
  }

 private:
  int x_index_ = 0;
  int y_index_ = 1;
  std::vector<Eigen::Vector3d> line_constraints_;
};

}  // namespace examples
}  // namespace altro