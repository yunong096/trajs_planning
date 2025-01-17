#pragma once

#include <array>
#include <limits>

// #include "dp_planner.hpp"
#include "df_planner/traj_optimizer.h"
using namespace plan_manage;
enum ErrorType { kSuccess = 0, kWrongStatus, kIllegalInput, kUnknown };
class TrajPlanner {
 public:
  enum ExtendDir { LEFT = 0, TOP, RIGHT, BOTTOM, EXTEND_DIR_SIZE };
  using ExtendFinishTable = std::array<bool, 4>;
  std::array<std::pair<int, int>, EXTEND_DIR_SIZE> col_indices = {
      {{BOTTOM, LEFT}, {LEFT, TOP}, {TOP, RIGHT}, {RIGHT, BOTTOM}}};

  TrajPlanner() = default;
  ~TrajPlanner() = default;

  bool Run(
    const CartesianState& init_state,
    Obstacles& obs,
    GlobalPath& refline,
    const int count,
    const vector<CartesianState>& source_path, 
    const vector<GlobalPathPoint>& ori_path, 
    const vector<CartesianState>& last_path, 
    vector<CartesianState>& final_result);

  bool CalKeyPoint(const vector<CartesianState>& source_path,
                              traj_utils::FlatTrajData* trajs, double duration);
    bool CalExtremePoint(const CartesianState& init_state,
                                            const CartesianState& end_node,
                                            traj_utils::FlatTrajData* trajs);
  void GetFlatState(const Eigen::Vector4d& state,
                    const Eigen::Vector2d& control_input,
                    Eigen::MatrixXd& flat_state);

 private:
  double end_curvature_ = 0.0;
  double end_heading_ = 0.0;
  TrajOptimizer traj_opt_;
  double kRbCollisionBuffur_ = 0.0; //0.15;
  double kLbCollisionBuffur_ = 0.0; //0.15;
  bool is_debug_mode_ = false;
  bool need_extra_rb_buffer_ = false;
    
    Obstacles obs_;
    int frame_count;
    std::vector<float> FrontPos(const vector<CartesianState> &x) {
      double N_ = 60;
      std::vector<float> z_new(2 * N_ + 2);
      for (int i = 0; i < N_ + 1; ++i) {
        z_new[i * 2] = x[i].x + 2.7 * cos(x[i].theta);
        z_new[i * 2 + 1] = x[i].y + 2.7 * sin(x[i].theta);
      }
      return z_new;
    }

    double calculateEuclideanNorm(const std::vector<float>& vec) {
        double sumOfSquares = 0.0;
        for (auto value : vec) {
            sumOfSquares += value * value;
        }
        return std::sqrt(sumOfSquares);
    }
    
    void d2poly(const Vector2d& point, const MatrixXd& poly, VectorXd& L, double& S, double& d) {
        d = numeric_limits<double>::infinity(); // 初始化距离为无穷大
        S = 0;
        L = VectorXd(2);
        int nside = poly.cols();
        int ii = -1;

        for (int i = 0; i < nside; ++i) {
            VectorXd p1 = poly.col(i);
            VectorXd p2 = poly.col((i + 1) % nside);

            // 计算三边的距离
            double trid[3];
            trid[0] = (p1 - p2).norm();
            trid[1] = (p1 - point).norm();
            trid[2] = (p2 - point).norm();

            Vector2d Lr(p1(1) - p2(1), p2(0) - p1(0));
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

    double triArea(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3) {
      return 0.5 * fabs(p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) + p3(0) * (p1(1) - p2(1)));
    }

    void plotFinalPath(vector<CartesianState>& path) {
        for(auto& state : path) {
            // cout << "df_state: " << state.x << ", " << state.y << ", " << state.speed << ", " << state.theta << ", " << state.acc << ", " << state.kappa << endl;
        }
    }
};
