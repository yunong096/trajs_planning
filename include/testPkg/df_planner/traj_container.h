#pragma once

#include <eigen3/Eigen/Eigen>
#include <vector>

namespace traj_utils {

struct FlatTrajData {
  std::vector<Eigen::Vector2d> inner_pts;     // 中间点
  Eigen::MatrixXd start_state;                // start flat state (2, 3)
  Eigen::MatrixXd final_state;                // end flat state (2, 3)
  std::vector<Eigen::Vector3d> corridor_pts;  // state list
  double duration;
  int singul = 1;
};

}  // namespace traj_utils
