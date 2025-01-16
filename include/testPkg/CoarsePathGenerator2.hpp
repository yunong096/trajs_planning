#include <vector>
#include "Eigen/Dense"
// #include "waypoint_msgs/refrence_point"
#include "modules/planning/planning_base/reference_line/spiral_problem_interface.h"
#include "modules/planning/planning_base/math/curve1d/quintic_spiral_path.h"
using namespace apollo::planning;
// using namespace google::base;

struct SmoothConfig {
  double piecewise_length;
  double max_deviation;
  double max_iteration;
  double weight_curve_length;
  double weight_kappa;
  double weight_dkappa;
  double resolution;
  double opt_tol;
  double opt_acceptable_tol;

  // mutable int _cached_size_;
  // double max_deviation_;
  // double piecewise_length_;
  // double opt_tol_;
  // ::google::protobuf::uint32 max_iteration_;
  // ::google::protobuf::uint32 opt_acceptable_iteration_;
  // double opt_acceptable_tol_;
  // double weight_curve_length_;
  // double weight_kappa_;
  // double weight_dkappa_;
};

struct GlobalPathPoint {
  double theta;
  double kappa;
  double dkappa;
  double s;
  double x; 
  double y;
  GlobalPathPoint(double t, double k, double dk, double s_, double x_, double y_) : theta(t), kappa(k), dkappa(dk), s(s_), x(x_), y(y_){}
};

struct GlobalPath {
  std::vector<double> theta;
  std::vector<double> kappa;
  std::vector<double> dkappa;
  std::vector<double> s;
  std::vector<double> x;
  std::vector<double> y;
}; 

class  CoarsePathGenerator  {
public:
  CoarsePathGenerator();
  CoarsePathGenerator(SmoothConfig config) {config_ = config;}
  // virtual ~CoarsePathGenerator();
  int SmoothStandAlone(std::vector<Eigen::Vector2d> point2d, std::vector<double>* ptr_theta,
                                                  std::vector<double>* ptr_kappa, std::vector<double>* ptr_dkappa,
                                                  std::vector<double>* ptr_s, std::vector<double>* ptr_x,
                                                  std::vector<double>* ptr_y) const;
  std::vector<GlobalPathPoint> Interpolate(const std::vector<double>& theta, const std::vector<double>& kappa,
                                                                                           const std::vector<double>& dkappa, const std::vector<double>& s,
                                                                                           const std::vector<double>& x, const std::vector<double>& y,
                                                                                           const double resolution) const;
  std::vector<GlobalPathPoint> Interpolate(const double start_x, const double start_y, const double start_s,
                                                                                           const double theta0, const double kappa0, const double dkappa0,
                                                                                           const double theta1, const double kappa1, const double dkappa1,
                                                                                           const double delta_s, const double resolution) const;
  // static GlobalPathPoint Interpolate3(const double start_x, const double start_y, const double start_s,
  //                                                                                                             const double theta0, const double kappa0, const double dkappa0,
  //                                                                                                             const double theta1, const double kappa1, const double dkappa1,
  //                                                                                                             const double delta_s, const double new_s);
  // static GlobalPathPoint FindNearestPt(const double start_x1, const double start_y1, const double start_s1,
  //                                                                                                                   const double theta1, const double kappa1, const double dkappa1,
  //                                                                                                                   const double start_x2, const double start_y2, const double start_s2,
  //                                                                                                                   const double theta2, const double kappa2, const double dkappa2,
  //                                                                                                                   const double theta3, const double kappa3, const double dkappa3,
  //                                                                                                                   const double delta_s1, const double delta_s2, const double pt_x, const double pt_y) ;

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