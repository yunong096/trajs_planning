#pragma once

#include <float.h>

#include <eigen3/Eigen/Eigen>
#include <utility>

#include "df_planner/constants.h"
// #include "dp_planner.hpp"
#include "nmpc2.hpp"
#include "GuassIter.hpp"

namespace traj_utils {
  
class VehicleParam {
 public:
  inline double width() const { return width_; }
  inline double length() const { return length_; }
  inline double wheel_base() const { return wheel_base_; }
  inline double front_suspension() const { return front_suspension_; }
  inline double rear_suspension() const { return rear_suspension_; }
  inline double max_steering_angle() const { return max_steering_angle_; }
  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }
  inline double max_lateral_acc() const { return max_lateral_acc_; }
  inline double d_cr() const { return d_cr_; }

  inline void set_width(const double val) { width_ = val; }
  inline void set_length(const double val) { length_ = val; }
  inline void set_wheel_base(const double val) { wheel_base_ = val; }
  inline void set_front_suspension(const double val) {
    front_suspension_ = val;
  }
  inline void set_rear_suspension(const double val) { rear_suspension_ = val; }
  inline void set_max_steering_angle(const double val) {
    max_steering_angle_ = val;
  }
  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }
  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }
  inline void set_d_cr(const double val) { d_cr_ = val; }

  /**
   * @brief Print info
   */
  void print() const;

 private:
  double width_ = 2.0;
  double length_ = 4.70;
  double wheel_base_ = 2.70;
  double front_suspension_ = 1.0;
  double rear_suspension_ = 1.0;
  double max_steering_angle_ = 30.0;

  double max_longitudinal_acc_ = 1.5;
  double max_lateral_acc_ = 1.5;

  double d_cr_ = 1.35;  // length between geometry center and rear axle
};
constexpr double PI = 3.1415926;
typedef Eigen::Matrix<double, 2, 6> CoefficientMat;
typedef Eigen::Matrix<double, 2, 5> VelCoefficientMat;
typedef Eigen::Matrix<double, 2, 4> AccCoefficientMat;

class Piece {
 private:  // duration + coeffMat
  double duration;
  CoefficientMat coeffMat;

  int dim = 2;
  int order = 5;

  int singul = 1;
  traj_utils::VehicleParam vp_;  // 使用container里的

 public:
  Piece() = default;

  Piece(double dur, const CoefficientMat &cMat, int s)
      : duration(dur), coeffMat(cMat), singul(s) {}
  //@yuwei
  int getDim() const { return dim; }
  //@yuwei
  inline int getOrder() const { return order; }

  inline double getDuration() const { return duration; }

  inline const CoefficientMat &getCoeffMat() const { return coeffMat; }

  VelCoefficientMat getVelCoeffMat() const {
    VelCoefficientMat velCoeffMat;
    int n = 1;
    for (int i = 4; i >= 0; i--) {
      velCoeffMat.col(i) = n * coeffMat.col(i);
      n++;
    }
    return velCoeffMat;
  }

  // the point in the rear axle center
  Eigen::Vector2d getPos(const double t) const {
    Eigen::Vector2d pos(0.0, 0.0);
    double tn = 1.0;
    for (int i = order; i >= 0; --i) {
      pos += tn * coeffMat.col(i);
      tn *= t;
    }
    return pos;
  }

  // Eigen::Vector2d getDerivative(const double t) const {
  //   Eigen::Vector2d dpos(0.0, 0.0);
  //   double tn = 1.0;
  //   for (int i = order - 1; i >= 0; --i) {
  //     dpos += tn * coeffMat.col(i) * (order - i);
  //     tn *= t;
  //   }
  //   return dpos;
  // }

  Eigen::Matrix2d getR(const double t) const {
    Eigen::Vector2d current_v = getdSigma(t);
    Eigen::Matrix2d rotation_matrix;
    rotation_matrix << current_v(0), -current_v(1), current_v(1), current_v(0);
    rotation_matrix = singul * rotation_matrix / current_v.norm();

    return rotation_matrix;
  }

  Eigen::Matrix2d getRdot(const double t) const {
    Eigen::Vector2d current_v = getdSigma(t);
    Eigen::Vector2d current_a = getddSigma(t);
    Eigen::Matrix2d temp_a_ba;
    Eigen::Matrix2d temp_v_bv;
    temp_a_ba << current_a(0), -current_a(1), current_a(1), current_a(0);
    temp_v_bv << current_v(0), -current_v(1), current_v(1), current_v(0);
    Eigen::Matrix2d R_dot = singul * (temp_a_ba / current_v.norm() -
                                      temp_v_bv / pow(current_v.norm(), 3) *
                                          (current_v.transpose() * current_a));

    return R_dot;
  }

  Eigen::Vector2d getdSigma(const double t) const {
    Eigen::Vector2d dsigma(0.0, 0.0);
    double tn = 1.0;
    int n = 1;

    for (int i = order - 1; i >= 0; i--) {
      dsigma += n * tn * coeffMat.col(i);
      tn *= t;
      n++;
    }
    return dsigma;
  }

  Eigen::Vector2d getddSigma(const double t) const {
    Eigen::Vector2d ddsigma(0.0, 0.0);
    double tn = 1.0;
    int m = 1;
    int n = 2;

    for (int i = order - 2; i >= 0; i--) {
      ddsigma += m * n * tn * coeffMat.col(i);
      tn *= t;
      m++;
      n++;
    }
    return ddsigma;
  }

  Eigen::Vector2d getdddSigma(const double t) const {
    Eigen::Vector2d dddsigma(0.0, 0.0);
    double tn = 1.0;
    int l = 1;
    int m = 2;
    int n = 3;

    // std::cout << "coeffMat is" << coeffMat << std::endl;
    for (int i = order - 3; i >= 0; i--) {
      dddsigma += l * m * n * tn * coeffMat.col(i);

      tn *= t;
      l++;
      m++;
      n++;
    }
    return dddsigma;
  }

  double getAngle(const double t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    return std::atan2(singul * dsigma(1), singul * dsigma(0));
  }

  double getCurv(const double t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    Eigen::Vector2d ddsigma = getddSigma(t);
    const double dsigma_norm = dsigma.norm();
    if (dsigma_norm < plan_manage::kEpsilon) {
      return 0.0;
    }
    return singul * (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
           (dsigma_norm * dsigma_norm * dsigma_norm);
  }

  void getState(const double relative_t, CartesianState& res) const {
    const auto pos = getPos(relative_t);
    res.x = pos[0];
    res.y = pos[1];
    Eigen::Vector2d dsigma = getdSigma(relative_t);
    Eigen::Vector2d ddsigma = getddSigma(relative_t);
    const double dsigma_norm = dsigma.norm();
    res.theta= (std::atan2(singul * dsigma(1), singul * dsigma(0)));
    res.speed = (singul * dsigma.norm());
    if (dsigma_norm < plan_manage::kEpsilon) {
      res.acc = (0.0);
      res.kappa = (0.0);
    } else {
      res.acc = (singul * (dsigma(0) * ddsigma(0) + dsigma(1) * ddsigma(1)) /
                 dsigma_norm);
      
      // //出于与nmpc2的仿真一致性需要,直接将kappa赋值为delta
      // res.kappa = atan(2.7 * (singul *
      //                    (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
      //                    (dsigma_norm * dsigma_norm * dsigma_norm)));
      res.kappa =(singul *
        (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
        (dsigma_norm * dsigma_norm * dsigma_norm));
    }
  }

  void getState(const double relative_t, GlobalPathPoint& res) const {
    const auto pos = getPos(relative_t);
    res.x = pos[0];
    res.y = pos[1];
    Eigen::Vector2d dsigma = getdSigma(relative_t);
    Eigen::Vector2d ddsigma = getddSigma(relative_t);
    Eigen::Vector2d dddsigma = getdddSigma(relative_t);
    const double dsigma_norm = dsigma.norm();
    res.theta= (std::atan2(singul * dsigma(1), singul * dsigma(0)));
    if (dsigma_norm < plan_manage::kEpsilon) {
      res.kappa = (0.0);
      res.dkappa = (0.0);
    } else {
      res.kappa = singul *
                         (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
                         (dsigma_norm * dsigma_norm * dsigma_norm);
      res.dkappa = singul *sqrt(dsigma_norm) * (
                              (ddsigma(0) * ddsigma(1) + dsigma(0) * dddsigma(1) - ddsigma(1) *ddsigma(0) -  dsigma(1) * dddsigma(0)) / (dsigma_norm * dsigma_norm * dsigma_norm)
                              + 
                              (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) * (-3/2) * pow(dsigma_norm * dsigma_norm, -5 / 2) * 2 * (dsigma(1) * ddsigma(1) + dsigma(0) * ddsigma(0)) 
                                );
    }
  }

  void pushState(const double relative_t, GlobalPath& refline) const {
    const auto pos = getPos(relative_t);
    refline.x.emplace_back(pos[0]);
    refline.y.emplace_back(pos[1]);
    Eigen::Vector2d dsigma = getdSigma(relative_t);
    Eigen::Vector2d ddsigma = getddSigma(relative_t);
    Eigen::Vector2d dddsigma = getdddSigma(relative_t);

    const double dsigma_norm = dsigma.norm();
    refline.theta.emplace_back(std::atan2(singul * dsigma(1), singul * dsigma(0)));
    // refline.speed.emplace_back(singul * dsigma.norm());
    if (dsigma_norm < plan_manage::kEpsilon) {
      // refline.acc.emplace_back (0.0);
      refline.kappa.emplace_back(0.0);
      refline.dkappa.emplace_back(0.0);
    } else {
      // refline.acc.emplace_back (singul * (dsigma(0) * ddsigma(0) + dsigma(1) * ddsigma(1)) /
      //            dsigma_norm);
      refline.kappa.emplace_back(singul *
                         (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
                         (dsigma_norm * dsigma_norm * dsigma_norm));
      refline.dkappa.emplace_back( singul *sqrt(dsigma_norm) * (
                          (ddsigma(0) * ddsigma(1) + dsigma(0) * dddsigma(1) - ddsigma(1) *ddsigma(0) -  dsigma(1) * dddsigma(0)) / (dsigma_norm * dsigma_norm * dsigma_norm)
                          + 
                          (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) * (-3/2) * pow(dsigma_norm * dsigma_norm, -5 / 2) * 2 * (dsigma(1) * ddsigma(1) + dsigma(0) * ddsigma(0)) 
                            ));
    }
  }

  double getVel(const double t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    return singul * dsigma.norm();
  }

  double getAcc(const double t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    Eigen::Vector2d ddsigma = getddSigma(t);
    const double dsigma_norm = dsigma.norm();
    if (dsigma_norm < plan_manage::kEpsilon) {
      return 0.0;
    }
    return singul * (dsigma(0) * ddsigma(0) + dsigma(1) * ddsigma(1)) /
           dsigma_norm;
  }
  double getLatAcc(const double t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    Eigen::Vector2d ddsigma = getddSigma(t);
    const double dsigma_norm = dsigma.norm();
    if (dsigma_norm < plan_manage::kEpsilon) {
      return 0.0;
    }
    return singul * (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
           dsigma_norm;
  }
  inline double getSteer(const double t) const {
    return std::atan(vp_.wheel_base() * getCurv(t));  //[-PI/2, PI/2]
  }
  Eigen::VectorXd getStateExpPos(const double t) const {
    Eigen::Vector2d dsigma = getdSigma(t);
    Eigen::Vector2d ddsigma = getddSigma(t);

    // state
    Eigen::VectorXd otherstate(5);

    // theta  vel
    otherstate[0] = std::atan2(singul * dsigma(1), singul * dsigma(0));
    otherstate[2] = singul * dsigma.norm();

    if (std::fabs(otherstate[2]) < plan_manage::kEpsilon) {
      otherstate[1] = 0.0;
      otherstate[3] = 0.0;
      otherstate[4] = 0.0;

    } else {
      // curv
      otherstate[1] = (dsigma(0) * ddsigma(1) - dsigma(1) * ddsigma(0)) /
                      std::pow(otherstate[2], 3);
      // acc
      otherstate[3] =
          (dsigma(0) * ddsigma(0) + dsigma(1) * ddsigma(1)) / otherstate[2];
      // phi
      otherstate[4] = std::atan(vp_.wheel_base() * otherstate[1]);
    }
    return otherstate;
  }

  CoefficientMat normalizePosCoeffMat() const {
    CoefficientMat nPosCoeffsMat;
    double t = 1.0;
    for (int i = order; i >= 0; i--) {
      nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
      t *= duration;
    }
    return nPosCoeffsMat;
  }
};
class Trajectory {
 private:
  typedef std::vector<Piece> Pieces;
  Pieces pieces;

 public:
  Trajectory() = default;

  Trajectory(const std::vector<double> &durs,
             const std::vector<CoefficientMat> &cMats, int s) {
    int N = std::min(durs.size(), cMats.size());
    pieces.reserve(N);
    for (int i = 0; i < N; i++) {
      pieces.emplace_back(durs[i], cMats[i], s);
    }
  }

  std::vector<double> Pieces_S;
  std::vector<double> Pieces_allS;

  GlobalPathPoint getState(double relative_t, int piece_idx) const
  {
     GlobalPathPoint res;
      pieces[piece_idx].getState(relative_t, res);
      double relative_s = computeArcLength2(piece_idx, 0.0, relative_t);
      if(piece_idx > 0) res.s = relative_s + Pieces_allS[piece_idx - 1];
      else res.s = relative_s;
    // ROS_WARN("left getState");
      return res;
  }

  void setPiece_S() {
    int N = pieces.size();
    Pieces_S = vector<double>(N, 0);
    Pieces_allS = vector<double>(N, 0);
    for(int i = 0; i < N; ++i) {
      double piece_t = pieces[i].getDuration();
      // Pieces_S[i] = computeArcLength(i, 0.0, piece_t, 1000);
      Pieces_S[i] = computeArcLength2(i, 0.0, piece_t, 5);
      if(i == 0) Pieces_allS[i] = Pieces_S[i];
      else Pieces_allS[i] = Pieces_allS[i - 1] + Pieces_S[i];
      // cout << Pieces_S[i] << ", " << Pieces_allS[i] << endl;
    }
  }

  double computeArcLength(int piece_idx, double t_start, double t_end, int steps = 100) const {
        double length = 0.0;
        double dt = (t_end - t_start) / steps;
        for (int i = 0; i < steps; ++i) {
            double t = t_start + i * dt;
            auto dpos = pieces[piece_idx].getdSigma(t);
            length += std::hypot(dpos(0), dpos(1)) * dt;
        }
        // ROS_WARN("left computeArcLength");
        return length;
    }

    double computeArcLength2(int piece_idx, double t_start, double t_end, int num_node = 3) const {
      std::function<double(double)> func = [this, piece_idx](double t) {
        auto dpos = pieces[piece_idx].getdSigma(t);
        return std::sqrt(dpos(0) * dpos(0) +  dpos(1) * dpos(1));
      };
      GaussLegendreIntegration integrator;
      if (num_node == 3)
        integrator = GaussLegendreIntegration(GaussLegendreIntegration::NodeCount::Three);
       else if (num_node == 5)
        integrator = GaussLegendreIntegration(GaussLegendreIntegration::NodeCount::Five);
      integrator.func = func;
      double result = integrator.integrate(t_start, t_end);
      return result;
  }

    // 在段内通过二分法求解s对应的t
    double findTInSegment(const int piece_idx, double s_local, double eps = 1e-3) const {
        double low = 0.0, high = pieces[piece_idx].getDuration();
        while (high - low > eps) {
            double mid = (low + high) / 2;
            double len = computeArcLength2(piece_idx, 0, mid);
            (len < s_local) ? low = mid : high = mid;
        }
        return (low + high) / 2;
    }

    double distanceSquared(double x1, double y1, double x2, double y2) const {
        return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
    }

    // 在段内通过牛顿法求解最近点对应的t
    Vector2d findMinPtInSegment(const int piece_idx, Vector2d pos, double init_t) const {
      // ROS_WARN("enter NewTon Method");
      double closest_t = init_t;
      double t = closest_t; // 初始猜测
      double tolerance = 1e-3;
      double epsilon = 1e-8; // 防止除零
      double minDistanceSquared = std::numeric_limits<double>::max();
      double x0 = pos(0), y0 = pos(1);
      GlobalPathPoint res();
      int max_iter = 100;
      while (max_iter--) {
          double x = pieces[piece_idx].getPos(t)(0);
          double y = pieces[piece_idx].getPos(t)(1);
          double currentDistanceSquared = distanceSquared(x, y, x0, y0);
  
          if (currentDistanceSquared < minDistanceSquared) {
              minDistanceSquared = currentDistanceSquared;
              closest_t = t;
          }
  
          double dx = pieces[piece_idx].getdSigma(t)(0);
          double dy = pieces[piece_idx].getdSigma(t)(1);
          double ddx = pieces[piece_idx].getddSigma(t)(0);
          double ddy = pieces[piece_idx].getddSigma(t)(1);
  
          double gradD = 2 * ((x - x0) * dx + (y - y0) * dy);
          double hessD = 2 * ((dx * dx + dy * dy) + (x - x0) * ddx + (y - y0) * ddy);
  
          if (std::abs(gradD) < tolerance || std::abs(hessD) < epsilon) {
              // cout << "find cloest!" << endl;
              return Vector2d(std::sqrt(minDistanceSquared), closest_t);
          }
  
          double deltaT = -gradD / hessD;
          t += deltaT;
  
          if (t < 0) {
            t = 0;
          }
          if (t > pieces[piece_idx].getDuration()) {
              t = pieces[piece_idx].getDuration();
            }
      }
  
          return Vector2d(hypot(pieces[piece_idx].getPos(init_t)(0) - x0, pieces[piece_idx].getPos(init_t)(1) - y0), init_t);
    }

    vector<CartesianState> GetResult(double gap) {
      // 优化结果密集采样输出
      vector<CartesianState> final_path;
      const double total_t = getTotalDuration();
      const int duration_size = std::ceil(total_t / gap);
      int count = 0;
      for (double t = 0.0; t < total_t; t += gap, ++count) {
        CartesianState state;
        const auto pieceIdx = locatePieceIdx(t);
        // cout << "relative_t: " << pieceIdx.first << ", piece_idx" << pieceIdx.second << endl;
        const auto &piece = pieces[pieceIdx.first];
        const double relative_t = pieceIdx.second;
        piece.getState(relative_t, state);
        // state->set_t(t);
        final_path.emplace_back(state);
      }
      return final_path;
    }
    
  
  inline int getPieceNum() const { return pieces.size(); }

  Eigen::VectorXd getDurations() const {
    int N = getPieceNum();
    Eigen::VectorXd durations(N);
    for (int i = 0; i < N; i++) {
      durations(i) = pieces[i].getDuration();
    }
    return durations;
  }

  

  double getTotalDuration() const {
    int N = getPieceNum();
    double totalDuration = 0.0;
    for (int i = 0; i < N; i++) {
      totalDuration += pieces[i].getDuration();
    }
    return totalDuration;
  }

  Eigen::MatrixXd getPositions() const {
    int N = getPieceNum();
    Eigen::MatrixXd positions(2, N + 1);
    for (int i = 0; i < N; i++) {
      positions.col(i) = pieces[i].getCoeffMat().col(5);
    }
    positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
    return positions;
  }

  const Piece &operator[](int i) const { return pieces[i]; }

  Piece &operator[](int i) { return pieces[i]; }

  inline void clear(void) { pieces.clear(); }

  Pieces::const_iterator begin() const { return pieces.begin(); }

  Pieces::const_iterator end() const { return pieces.end(); }

  Pieces::iterator begin() { return pieces.begin(); }

  Pieces::iterator end() { return pieces.end(); }

  void reserve(const int n) { pieces.reserve(n); }

  inline void emplace_back(const Piece &piece) { pieces.emplace_back(piece); }

  inline void emplace_back(const double dur, const CoefficientMat &cMat,
                           int s) {
    pieces.emplace_back(dur, cMat, s);
  }

  inline void append(const Trajectory &traj) {
    pieces.insert(pieces.end(), traj.begin(), traj.end());
  }

  std::pair<int, double> locatePieceIdx(double t) const {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces[idx].getDuration();
    }
    return {idx, t};
  }

  Eigen::Vector2d getPos(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getPos(piece_info.second);
  }

  Eigen::Matrix2d getR(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getR(piece_info.second);
  }

  Eigen::Matrix2d getRdot(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getRdot(piece_info.second);
  }

  Eigen::Vector2d getdSigma(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getdSigma(piece_info.second);
  }

  Eigen::Vector2d getddSigma(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getddSigma(piece_info.second);
  }

  double getVel(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getVel(piece_info.second);
  }

  double getAcc(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getAcc(piece_info.second);
  }
  double getLatAcc(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getLatAcc(piece_info.second);
  }

  double getAngle(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getAngle(piece_info.second);
  }

  double getCurv(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getCurv(piece_info.second);
  }

  double getSteer(double t) const {
    const auto piece_info = locatePieceIdx(t);
    return pieces[piece_info.first].getSteer(piece_info.second);
  }

  Eigen::Vector2d getJuncPos(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces[juncIdx].getCoeffMat().col(5);
    }
    return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
  }

  Eigen::Vector2d getJuncdSigma(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces[juncIdx].getCoeffMat().col(4);
    }
    return pieces[juncIdx - 1].getdSigma(pieces[juncIdx - 1].getDuration());
  }

  Eigen::Vector2d getJuncddSigma(int juncIdx) const {
    if (juncIdx != getPieceNum()) {
      return pieces[juncIdx].getCoeffMat().col(3) * 2.0;
    }
    return pieces[juncIdx - 1].getddSigma(pieces[juncIdx - 1].getDuration());
  }

  std::pair<int, double> locatePieceIdxWithRatio(double &t) const {
    int N = getPieceNum();
    int idx;
    double dur;
    for (idx = 0; idx < N && t > (dur = pieces[idx].getDuration()); idx++) {
      t -= dur;
    }
    if (idx == N) {
      idx--;
      t += pieces[idx].getDuration();
    }
    return {idx, t / dur};
  }

  Eigen::Vector2d getPoswithIdxRatio(double t,
                                     std::pair<int, double> &idx_ratio) const {
    idx_ratio = locatePieceIdxWithRatio(t);
    return pieces[idx_ratio.first].getPos(t);
  }
};

class BandedSystem {
 public:
  // The size of A, as well as the lower/upper
  // banded width p/q are needed
  void create(const int n, const int p, const int q) {
    N = n;
    lowerBw = p;
    upperBw = q;
    int actualSize = N * (lowerBw + upperBw + 1);
    ptrData.resize(actualSize);
    std::fill(ptrData.begin(), ptrData.end(), 0.0);
  }

 private:
  int N;
  int lowerBw;
  int upperBw;
  // Compulsory nullptr initialization here
  std::vector<double> ptrData{};

 public:
  // The band matrix is stored as suggested in "Matrix Computation"
  const double &operator()(const int i, const int j) const {
    return ptrData[(i - j + upperBw) * N + j];
  }

  inline double &operator()(const int i, const int j) {
    return ptrData[(i - j + upperBw) * N + j];
  }

  // This function conducts banded LU factorization in place
  // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
  void factorizeLU() {
    int iM;
    int jM;
    double cVl;
    for (int k = 0; k <= N - 2; k++) {
      iM = std::min(k + lowerBw, N - 1);
      cVl = operator()(k, k);
      for (int i = k + 1; i <= iM; i++) {
        if (std::fabs(operator()(i, k)) > DBL_EPSILON) {
          operator()(i, k) /= cVl;
        }
      }
      jM = std::min(k + upperBw, N - 1);
      for (int j = k + 1; j <= jM; j++) {
        cVl = operator()(k, j);
        if (std::fabs(cVl) > DBL_EPSILON) {
          for (int i = k + 1; i <= iM; i++) {
            if (std::fabs(operator()(i, k)) > DBL_EPSILON) {
              operator()(i, j) -= operator()(i, k) * cVl;
            }
          }
        }
      }
    }
  }

  // This function solves Ax=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT>
  void solve(EIGENMAT &b) const {
    int iM;
    for (int j = 0; j <= N - 1; j++) {
      iM = std::min(j + lowerBw, N - 1);
      for (int i = j + 1; i <= iM; i++) {
        if (std::fabs(operator()(i, j)) > DBL_EPSILON) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; j--) {
      b.row(j) /= operator()(j, j);
      iM = std::max(0, j - upperBw);
      for (int i = iM; i <= j - 1; i++) {
        if (std::fabs(operator()(i, j)) > DBL_EPSILON) {
          b.row(i) -= operator()(i, j) * b.row(j);
        }
      }
    }
  }

  // This function solves ATx=b, then stores x in b
  // The input b is required to be N*m, i.e.,
  // m vectors to be solved.
  template <typename EIGENMAT>
  void solveAdj(EIGENMAT &b) const {
    int iM;
    for (int j = 0; j <= N - 1; j++) {
      b.row(j) /= operator()(j, j);
      iM = std::min(j + upperBw, N - 1);
      for (int i = j + 1; i <= iM; i++) {
        if (std::fabs(operator()(j, i)) > DBL_EPSILON) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
    for (int j = N - 1; j >= 0; j--) {
      iM = std::max(0, j - lowerBw);
      for (int i = iM; i <= j - 1; i++) {
        if (std::fabs(operator()(j, i)) > DBL_EPSILON) {
          b.row(i) -= operator()(j, i) * b.row(j);
        }
      }
    }
  }
};
class MinJerkOpt {
 public:
  MinJerkOpt() = default;
  ~MinJerkOpt() = default;

 private:
  int N;  // pieceNum
  Eigen::MatrixXd headPVA;
  Eigen::MatrixXd tailPVA;
  Eigen::MatrixXd b;
  Eigen::MatrixXd c;
  Eigen::MatrixXd adjScaledGrad;  // 6*N, 2
  BandedSystem A;                 // 6 * N, 6 * N
  Eigen::Matrix<double, 6, 1> t;
  Eigen::Matrix<double, 6, 1> tInv;

  // for outside use
  /*polynomial descrips*/
  Eigen::MatrixXd gdC;
  double gdT;
  /*MINCO descrips*/
  Eigen::MatrixXd gdHead;
  Eigen::MatrixXd gdTail;
  Eigen::MatrixXd gdP;

 public:
  double getHeadX() const{
    double res = headPVA(0, 0);
    return res;
  }
  double getHeadY() const {
    double res = headPVA(1, 0);
    return res;
  } 
  double getHeadTheta() const {
    double res =  atan2(headPVA(1, 1), headPVA(0, 1));
    return res;
  }
  double getTailX() const{
    double res = tailPVA(0, 0);
    return res;
  }
  double getTailY() const {
    double res = tailPVA(1, 0);
    return res;
  } 
  double getTailTheta() const {
    double res =  atan2(tailPVA(1, 1), tailPVA(0, 1));
    return res;
  }
  void reset(const int pieceNum) {
    N = pieceNum;
    A.create(6 * N, 6, 6);
    b.resize(6 * N, 2);
    c.resize(6 * N, 2);
    adjScaledGrad.resize(6 * N, 2);
    gdC.resize(6 * N, 2);
    gdP.resize(2, N - 1);

    gdHead.resize(2, 3);
    gdTail.resize(2, 3);

    t(0) = 1.0;

    A(0, 0) = 1.0;
    A(1, 1) = 1.0;
    A(2, 2) = 2.0;
    for (int i = 0; i < N - 1; i++) {
      A(6 * i + 3, 6 * i + 3) = 6.0;
      A(6 * i + 3, 6 * i + 4) = 24.0;
      A(6 * i + 3, 6 * i + 5) = 60.0;
      A(6 * i + 3, 6 * i + 9) = -6.0;
      A(6 * i + 4, 6 * i + 4) = 24.0;
      A(6 * i + 4, 6 * i + 5) = 120.0;
      A(6 * i + 4, 6 * i + 10) = -24.0;
      A(6 * i + 5, 6 * i) = 1.0;
      A(6 * i + 5, 6 * i + 1) = 1.0;
      A(6 * i + 5, 6 * i + 2) = 1.0;
      A(6 * i + 5, 6 * i + 3) = 1.0;
      A(6 * i + 5, 6 * i + 4) = 1.0;
      A(6 * i + 5, 6 * i + 5) = 1.0;
      A(6 * i + 6, 6 * i) = 1.0;
      A(6 * i + 6, 6 * i + 1) = 1.0;
      A(6 * i + 6, 6 * i + 2) = 1.0;
      A(6 * i + 6, 6 * i + 3) = 1.0;
      A(6 * i + 6, 6 * i + 4) = 1.0;
      A(6 * i + 6, 6 * i + 5) = 1.0;
      A(6 * i + 6, 6 * i + 6) = -1.0;
      A(6 * i + 7, 6 * i + 1) = 1.0;
      A(6 * i + 7, 6 * i + 2) = 2.0;
      A(6 * i + 7, 6 * i + 3) = 3.0;
      A(6 * i + 7, 6 * i + 4) = 4.0;
      A(6 * i + 7, 6 * i + 5) = 5.0;
      A(6 * i + 7, 6 * i + 7) = -1.0;
      A(6 * i + 8, 6 * i + 2) = 2.0;
      A(6 * i + 8, 6 * i + 3) = 6.0;
      A(6 * i + 8, 6 * i + 4) = 12.0;
      A(6 * i + 8, 6 * i + 5) = 20.0;
      A(6 * i + 8, 6 * i + 8) = -2.0;
    }
    A(6 * N - 3, 6 * N - 6) = 1.0;
    A(6 * N - 3, 6 * N - 5) = 1.0;
    A(6 * N - 3, 6 * N - 4) = 1.0;
    A(6 * N - 3, 6 * N - 3) = 1.0;
    A(6 * N - 3, 6 * N - 2) = 1.0;
    A(6 * N - 3, 6 * N - 1) = 1.0;
    A(6 * N - 2, 6 * N - 5) = 1.0;
    A(6 * N - 2, 6 * N - 4) = 2.0;
    A(6 * N - 2, 6 * N - 3) = 3.0;
    A(6 * N - 2, 6 * N - 2) = 4.0;
    A(6 * N - 2, 6 * N - 1) = 5.0;
    A(6 * N - 1, 6 * N - 4) = 2.0;
    A(6 * N - 1, 6 * N - 3) = 6.0;
    A(6 * N - 1, 6 * N - 2) = 12.0;
    A(6 * N - 1, 6 * N - 1) = 20.0;
    A.factorizeLU();
  }
  void generate(const Eigen::MatrixXd &inPs, const double dT,
                const Eigen::MatrixXd &headState,
                const Eigen::MatrixXd &tailState) {
    // inPs,每段seg的innerpoints，会调用多次
    headPVA = headState;
    tailPVA = tailState;

    t(1) = dT;
    t(2) = t(1) * t(1);
    t(3) = t(2) * t(1);
    t(4) = t(2) * t(2);
    t(5) = t(4) * t(1);
    tInv = t.cwiseInverse();

    b.setZero();  // b是6N x 2，因为s=3，Mi x 2s
    b.row(0) = headPVA.col(0).transpose();
    b.row(1) = headPVA.col(1).transpose() * t(1);
    b.row(2) = headPVA.col(2).transpose() * t(2);
    for (int i = 0; i < N - 1; i++) {  // N是piece num，相当于论文中的Mi
      b.row(6 * i + 5) = inPs.col(i).transpose();
      // 为什么要+5，不是+3？明白了，这个前面两行是三阶和四阶段的连续约束，后续是中间点，0阶段，一阶，二阶，加起来一共6行。
    }
    b.row(6 * N - 3) = tailPVA.col(0).transpose();
    b.row(6 * N - 2) = tailPVA.col(1).transpose() * t(1);
    b.row(6 * N - 1) = tailPVA.col(2).transpose() * t(2);  // b矩阵是Mc=b的b

    A.solve(b);

    for (int i = 0; i < N; i++) {  // 求解结果是赋给了b，所以需要把b的值传给c
      c.block<6, 2>(6 * i, 0) =
          b.block<6, 2>(6 * i, 0).array().colwise() * tInv.array();
    }
  }

  Trajectory getTraj(int s) const {
    Trajectory traj;
    traj.reserve(N);
    for (int i = 0; i < N; i++) {
      traj.emplace_back(
          t(1), c.block<6, 2>(6 * i, 0).transpose().rowwise().reverse(), s);
    }
    return traj;
  }
  Eigen::VectorXd getTrajJerkCost() const {
    Eigen::VectorXd energy(N);
    energy.setZero();
    for (int i = 0; i < N; i++) {
      energy[i] += 36.0 * c.row(6 * i + 3).squaredNorm() * t(1) +
                   144.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * t(2) +
                   192.0 * c.row(6 * i + 4).squaredNorm() * t(3) +
                   240.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * t(3) +
                   720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * t(4) +
                   720.0 * c.row(6 * i + 5).squaredNorm() * t(5);  // jerk项
    }
    return energy;
  }

  inline double getTrajJerkCost(int flag) const {
      double energy = 0.0;
      for (int i = 0; i < N; i++) {
      energy += 36.0 * c.row(6 * i + 3).squaredNorm() * t(1) +
          144.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * t(2) +
          192.0 * c.row(6 * i + 4).squaredNorm() * t(3) +
          240.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * t(3) +
          720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * t(4) +
          720.0 * c.row(6 * i + 5).squaredNorm() * t(5);
      }
      return energy;
  }

  Eigen::VectorXd getTrajAccCost() const {
    Eigen::VectorXd energy(N);
    energy.setZero();
    for (int i = 0; i < N; i++) {
      energy[i] =
          4.0 * c.row(6 * i + 2).squaredNorm() * t(1) +
          12.0 * c.row(6 * i + 3).squaredNorm() * t(3) +
          28.8 * c.row(6 * i + 4).squaredNorm() * t(5) +
          57.1429 * c.row(6 * i + 5).squaredNorm() * t(2) * t(5) +
          12.0 * c.row(6 * i + 2).dot(c.row(6 * i + 3)) * t(2) +
          16.0 * c.row(6 * i + 2).dot(c.row(6 * i + 4)) * t(3) +
          20.0 * c.row(6 * i + 2).dot(c.row(6 * i + 5)) * t(4) +
          36.0 * c.row(6 * i + 3).dot(c.row(6 * i + 4)) * t(4) +
          48.0 * c.row(6 * i + 3).dot(c.row(6 * i + 5)) * t(5) +
          80.0 * c.row(6 * i + 4).dot(c.row(6 * i + 5)) * t(5) * t(1);  // acc项
    }
    return energy;
  }

  inline void initSmGradCost() {
      for (int i = 0; i < N; i++) {
          gdC.row(6 * i + 5) = 240.0 * c.row(6 * i + 3) * t(3) +
                              720.0 * c.row(6 * i + 4) * t(4) +
                              1440.0 * c.row(6 * i + 5) * t(5);
          gdC.row(6 * i + 4) = 144.0 * c.row(6 * i + 3) * t(2) +
                              384.0 * c.row(6 * i + 4) * t(3) +
                              720.0 * c.row(6 * i + 5) * t(4);
          gdC.row(6 * i + 3) = 72.0 * c.row(6 * i + 3) * t(1) +
                              144.0 * c.row(6 * i + 4) * t(2) +
                              240.0 * c.row(6 * i + 5) * t(3);
          gdC.block<3, 2>(6 * i, 0).setZero();
      }
      gdT = 0.0;
      for (int i = 0; i < N; i++) {
          gdT += 36.0 * c.row(6 * i + 3).squaredNorm() +
              288.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * t(1) +
              576.0 * c.row(6 * i + 4).squaredNorm() * t(2) +
              720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * t(2) +
              2880.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * t(3) +
              3600.0 * c.row(6 * i + 5).squaredNorm() * t(4);
      }
      return;
  }

  void initSmGradCost(double wei_jerk_, double wei_acc_) {
    for (int i = 0; i < N; i++) {
      gdC.block<6, 2>(6 * i, 0).setZero();

      gdC.row(6 * i + 5) +=
          (240.0 * c.row(6 * i + 3) * t(3) + 720.0 * c.row(6 * i + 4) * t(4) +
           1440.0 * c.row(6 * i + 5) * t(5)) *
          wei_jerk_;
      gdC.row(6 * i + 4) +=
          (144.0 * c.row(6 * i + 3) * t(2) + 384.0 * c.row(6 * i + 4) * t(3) +
           720.0 * c.row(6 * i + 5) * t(4)) *
          wei_jerk_;
      gdC.row(6 * i + 3) +=
          (72.0 * c.row(6 * i + 3) * t(1) + 144.0 * c.row(6 * i + 4) * t(2) +
           240.0 * c.row(6 * i + 5) * t(3)) *
          wei_jerk_;
      // acc项的gDC
      // gdC.row(6 * i + 5) +=
      //     (20.0 * c.row(6 * i + 2) * t(4) + 48.0 * c.row(6 * i + 3) * t(5) +
      //      80.0 * c.row(6 * i + 4) * t(5) * t(1) +
      //      114.2857 * c.row(6 * i + 5) * t(5) * t(2)) *
      //     wei_acc_;
      // gdC.row(6 * i + 4) +=
      //     (16.0 * c.row(6 * i + 2) * t(3) + 36.0 * c.row(6 * i + 3) * t(4) +
      //      57.6 * c.row(6 * i + 4) * t(5) +
      //      80.0 * c.row(6 * i + 5) * t(5) * t(1)) *
      //     wei_acc_;
      // gdC.row(6 * i + 3) +=
      //     (12.0 * c.row(6 * i + 2) * t(2) + 24.0 * c.row(6 * i + 3) * t(3) +
      //      36.0 * c.row(6 * i + 4) * t(4) + 48.0 * c.row(6 * i + 5) * t(5)) *
      //     wei_acc_;
      // gdC.row(6 * i + 2) +=
      //     (8.0 * c.row(6 * i + 3) * t(1) + 12.0 * c.row(6 * i + 3) * t(2) +
      //      16.0 * c.row(6 * i + 4) * t(3) + 20.0 * c.row(6 * i + 5) * t(4)) *
      //     wei_acc_;
      // snap项的gDC
      // gdC.block<6, 2>(6 * i, 0).setZero();
      // gdC.row(6 * i + 5) =
      //     2880.0 * c.row(6 * i + 4) * t(2) + 9600.0 * c.row(6 * i + 5) *
      //     t(3);
      // gdC.row(6 * i + 4) =
      //     1152.0 * c.row(6 * i + 4) * t(1) + 2880.0 * c.row(6 * i + 5) *
      //     t(2);
    }
    gdT = 0.0;
    for (int i = 0; i < N; i++) {
      gdT += (36.0 * c.row(6 * i + 3).squaredNorm() +
              288.0 * c.row(6 * i + 4).dot(c.row(6 * i + 3)) * t(1) +
              576.0 * c.row(6 * i + 4).squaredNorm() * t(2) +
              720.0 * c.row(6 * i + 5).dot(c.row(6 * i + 3)) * t(2) +
              2880.0 * c.row(6 * i + 5).dot(c.row(6 * i + 4)) * t(3) +
              3600.0 * c.row(6 * i + 5).squaredNorm() * t(4)) *
             wei_jerk_;
      // acc的gDT
      // gdT += (4.0 * c.row(6 * i + 2).squaredNorm() +
      //         36.0 * c.row(6 * i + 3).squaredNorm() * t(2) +
      //         144.0 * c.row(6 * i + 4).squaredNorm() * t(4) +
      //         400.0 * c.row(6 * i + 5).squaredNorm() * t(5) * t(1) +
      //         24.0 * c.row(6 * i + 2).dot(c.row(6 * i + 3)) * t(1) +
      //         48.0 * c.row(6 * i + 2).dot(c.row(6 * i + 4)) * t(2) +
      //         80.0 * c.row(6 * i + 2).dot(c.row(6 * i + 5)) * t(3) +
      //         144.0 * c.row(6 * i + 3).dot(c.row(6 * i + 4)) * t(3) +
      //         240.0 * c.row(6 * i + 3).dot(c.row(6 * i + 5)) * t(4) +
      //         480.0 * c.row(6 * i + 4).dot(c.row(6 * i + 5)) * t(5)) *
      //        wei_acc_;
    }
  }
  void calGrads_PT() {
    for (int i = 0; i < N; i++) {
      adjScaledGrad.block<6, 2>(6 * i, 0) =
          gdC.block<6, 2>(6 * i, 0).array().colwise() *
          tInv.array();  // adjScaledGrad是什么矩阵？
    }
    A.solveAdj(adjScaledGrad);

    for (int i = 0; i < N - 1; i++) {
      gdP.col(i) = adjScaledGrad.row(6 * i + 5).transpose();
    }
    gdHead = adjScaledGrad.topRows(3).transpose() * t.head<3>().asDiagonal();
    gdTail = adjScaledGrad.bottomRows(3).transpose() * t.head<3>().asDiagonal();

    gdT += headPVA.col(1).dot(adjScaledGrad.row(1));
    gdT += headPVA.col(2).dot(adjScaledGrad.row(2)) * 2.0 * t(1);
    gdT += tailPVA.col(1).dot(adjScaledGrad.row(6 * N - 2));
    gdT += tailPVA.col(2).dot(adjScaledGrad.row(6 * N - 1)) * 2.0 * t(1);
    Eigen::Matrix<double, 6, 1> gdtInv;
    gdtInv(0) = 0.0;
    gdtInv(1) = -1.0 * tInv(2);
    gdtInv(2) = -2.0 * tInv(3);
    gdtInv(3) = -3.0 * tInv(4);
    gdtInv(4) = -4.0 * tInv(5);
    gdtInv(5) = -5.0 * tInv(5) * tInv(1);
    const Eigen::VectorXd gdcol = gdC.cwiseProduct(b).rowwise().sum();
    for (int i = 0; i < N; i++) {
      gdT += gdtInv.dot(gdcol.segment<6>(6 * i));
    }
  }
  const Eigen::MatrixXd &getCoeffs() const { return c; }
  const double &getDt() const { return t(1); }

  Eigen::MatrixXd &get_gdC() { return gdC; }
  double &get_gdT() { return gdT; }

  Eigen::MatrixXd get_gdHead() { return gdHead; }
  Eigen::MatrixXd get_gdTail() { return gdTail; }
  const Eigen::MatrixXd &get_gdP() { return gdP; }
};

}  // namespace traj_utils
