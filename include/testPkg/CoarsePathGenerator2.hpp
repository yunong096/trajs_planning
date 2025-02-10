#include <vector>
#include "Eigen/Dense"
// #include "waypoint_msgs/refrence_point"
#include "CoarsePathGenerator2.hpp"
#include "modules/planning/planning_base/reference_line/spiral_problem_interface.h"
#include "modules/planning/planning_base/math/curve1d/quintic_spiral_path.h"
using namespace apollo::planning;
// using namespace google::base;

class  CoarsePathGenerator2  {
public:
  CoarsePathGenerator();
  CoarsePathGenerator(SmoothConfig config) {config_ = config;}
  // virtual ~CoarsePathGenerator();
  int SmoothStandAlone(std::vector<Eigen::Vector2d> point2d, std::vector<double>* ptr_theta,
                                                  std::vector<double>* ptr_kappa, std::vector<double>* ptr_dkappa,
                                                  std::vector<double>* ptr_s, std::vector<double>* ptr_x,
                                                  std::vector<double>* ptr_y) const;
  static std::vector<GlobalPathPoint> Interpolate(const std::vector<double>& theta, const std::vector<double>& kappa,
                                                                                           const std::vector<double>& dkappa, const std::vector<double>& s,
                                                                                           const std::vector<double>& x, const std::vector<double>& y,
                                                                                           const double resolution) ;
  static std::vector<GlobalPathPoint> Interpolate(const double start_x, const double start_y, const double start_s,
                                                                                           const double theta0, const double kappa0, const double dkappa0,
                                                                                           const double theta1, const double kappa1, const double dkappa1,
                                                                                           const double delta_s, const double resolution) ;
  static GlobalPathPoint FindRefPt(const double start_x, const double start_y, const double start_s,
                                                                                                            const double theta0, const double kappa0, const double dkappa0,
                                                                                                            const double theta1, const double kappa1, const double dkappa1,
                                                                                                            const double delta_s, const double new_s);
  static GlobalPathPoint FindNearestPt(const double start_x1, const double start_y1, const double start_s1,
                                                                                                                    const double theta1, const double kappa1, const double dkappa1,
                                                                                                                    const double start_x2, const double start_y2, const double start_s2,
                                                                                                                    const double theta2, const double kappa2, const double dkappa2,
                                                                                                                    const double theta3, const double kappa3, const double dkappa3,
                                                                                                                    const double delta_s1, const double delta_s2, const double pt_x, const double pt_y) ;
  static GlobalPathPoint FindNearestPt(const double start_x1, const double start_y1, const double start_s1,
                                                                                                                                const double theta1, const double kappa1, const double dkappa1,
                                                                                                                                const double theta2, const double kappa2, const double dkappa2,
                                                                                                                                const double delta_s, const double pt_x, const double pt_y);                                                                                                               
  static double NormalizeAngle(const double angle) {
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);
    if (a < 0.0) {
      a += (2.0 * M_PI);
    }
    return a - M_PI;
  }

  static double AngleDiff(const double from, const double to) {
    return NormalizeAngle(to - from);
  }
  private:
  SmoothConfig config_;

  bool fixed_start_point_ = false;

  double fixed_start_x_ = 0.0;

  double fixed_start_y_ = 0.0;

  double fixed_start_theta_ = 0.0;

  double fixed_start_kappa_ = 0.0;

  double fixed_start_dkappa_ = 0.0;

  double fixed_end_x_ = 0.0;

  double fixed_end_y_ = 0.0;

  double zero_x_ = 0.0;

  double zero_y_ = 0.0;
};