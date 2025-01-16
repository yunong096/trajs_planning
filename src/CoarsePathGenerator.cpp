#include "CoarsePathGenerator.hpp"
#include <algorithm>
#include <float.h>
#include <utility>
#include <coin/IpIpoptApplication.hpp>
#include <coin/IpSolveStatistics.hpp>
using namespace std;
using namespace apollo::planning;
// using namespace google::base;

int CoarsePathGenerator::SmoothStandAlone(
  std::vector<Eigen::Vector2d> point2d, std::vector<double>* ptr_theta,
  std::vector<double>* ptr_kappa, std::vector<double>* ptr_dkappa,
  std::vector<double>* ptr_s, std::vector<double>* ptr_x,
  std::vector<double>* ptr_y) const {
  // CHECK_GT(point2d.size(), 1U);

  vector<int> sharp_points {17};
  // SpiralProblemInterface* ptop = new SpiralProblemInterface(point2d, sharp_points);
  SpiralProblemInterface* ptop = new SpiralProblemInterface(point2d);


  ptop->set_default_max_point_deviation(config_.max_deviation);
  ptop->set_element_weight_curve_length(config_.weight_curve_length);
  ptop->set_element_weight_kappa(config_.weight_kappa);
  ptop->set_element_weight_dkappa(config_.weight_dkappa);

  Ipopt::SmartPtr<Ipopt::TNLP> problem = ptop;

  // Create an instance of the IpoptApplication
  Ipopt::SmartPtr<Ipopt::IpoptApplication> app = IpoptApplicationFactory();

  app->Options()->SetStringValue("hessian_approximation", "limited-memory");
  app->Options()->SetIntegerValue("max_iter", config_.max_iteration);
  app->Options()->SetNumericValue("tol", config_.opt_tol);
  app->Options()->SetNumericValue("acceptable_tol",
                                  config_.opt_acceptable_tol);

  Ipopt::ApplicationReturnStatus status = app->Initialize();
  if (status != Ipopt::Solve_Succeeded) {
    // ADEBUG << "*** Error during initialization!";
    return -1;
  }

  status = app->OptimizeTNLP(problem);

  if (status == Ipopt::Solve_Succeeded ||
      status == Ipopt::Solved_To_Acceptable_Level) {
    // Retrieve some statistics about the solve
    Ipopt::Index iter_count = app->Statistics()->IterationCount();
    // ADEBUG << "*** The problem solved in " << iter_count << " iterations!";

    Ipopt::Number final_obj = app->Statistics()->FinalObjective();
    // ADEBUG << "*** The final value of the objective function is " << final_obj << '.';
  } else {
    // ADEBUG << "Return status: " << int(status);
  }

  ptop->get_optimization_results(ptr_theta, ptr_kappa, ptr_dkappa, ptr_s, ptr_x,
                                 ptr_y);

  if (!(status == Ipopt::Solve_Succeeded) &&
      !(status == Ipopt::Solved_To_Acceptable_Level)) {
    return -1;
  }
  return app->Statistics()->IterationCount();
}

std::vector<GlobalPathPoint> CoarsePathGenerator::Interpolate(
    const std::vector<double>& theta, const std::vector<double>& kappa,
    const std::vector<double>& dkappa, const std::vector<double>& s,
    const std::vector<double>& x, const std::vector<double>& y,
    const double resolution)  {
  std::vector<GlobalPathPoint> smoothed_point2d;
  double start_s = 0.0;
  GlobalPathPoint first_point(theta.front(), kappa.front(), dkappa.front(), start_s, x.front(), y.front());
  smoothed_point2d.push_back(first_point);

  for (size_t i = 0; i + 1 < theta.size(); ++i) {
    double start_x = x[i];
    double start_y = y[i];

    auto path_point_seg = Interpolate(
        start_x, start_y, start_s, theta[i], kappa[i], dkappa[i], theta[i + 1],
        kappa[i + 1], dkappa[i + 1], s[i], resolution);

    smoothed_point2d.insert(smoothed_point2d.end(), path_point_seg.begin(),
                            path_point_seg.end());

    start_s = smoothed_point2d.back().s;
  }
  return smoothed_point2d;
}

std::vector<GlobalPathPoint> CoarsePathGenerator::Interpolate(
    const double start_x, const double start_y, const double start_s,
    const double theta0, const double kappa0, const double dkappa0,
    const double theta1, const double kappa1, const double dkappa1,
    const double delta_s, const double resolution)  {
  std::vector<GlobalPathPoint> path_points;

  const auto angle_diff = AngleDiff(theta0, theta1);

  QuinticSpiralPath spiral_curve(theta0, kappa0, dkappa0, theta0 + angle_diff,
                                 kappa1, dkappa1, delta_s); //五次多项式拟合计算该段的五次螺旋曲线theta(s)
  // std::cout << "五次多项式拟合成功" << endl;
  size_t num_of_points =
      static_cast<size_t>(std::ceil(delta_s / resolution)); //static_cast<size_t>(std::ceil(delta_s / resolution) + 1);
  for (size_t i = 1; i <= num_of_points; ++i) {
    const double inter_s =
        delta_s / static_cast<double>(num_of_points) * static_cast<double>(i);
    const double dx = spiral_curve.ComputeCartesianDeviationX<10>(inter_s);
    const double dy = spiral_curve.ComputeCartesianDeviationY<10>(inter_s);

    const double theta = NormalizeAngle(spiral_curve.Evaluate(0, inter_s));
    const double kappa = spiral_curve.Evaluate(1, inter_s);
    const double dkappa = spiral_curve.Evaluate(2, inter_s);

    GlobalPathPoint path_point(theta, kappa, dkappa, start_s + inter_s, start_x + dx, start_y + dy);
    path_points.push_back(std::move(path_point));
  }
  return path_points;
}

GlobalPathPoint CoarsePathGenerator::FindRefPt(
    const double start_x, const double start_y, const double start_s,
    const double theta0, const double kappa0, const double dkappa0,
    const double theta1, const double kappa1, const double dkappa1,
    const double delta_s, const double new_s) {
  const auto angle_diff = AngleDiff(theta0, theta1);
  QuinticSpiralPath spiral_curve(theta0, kappa0, dkappa0, theta0 + angle_diff,
                                 kappa1, dkappa1, delta_s); //五次多项式拟合计算该段的五次螺旋曲线theta(s)
  const double inter_s = new_s - start_s;
  // std::cout << "new_s: " << new_s << " start_s: " << start_s << " inter_s: " << inter_s << endl;
  const double dx = spiral_curve.ComputeCartesianDeviationX<10>(inter_s);
  const double dy = spiral_curve.ComputeCartesianDeviationY<10>(inter_s);
  const double theta = NormalizeAngle(spiral_curve.Evaluate(0, inter_s));
  const double kappa = spiral_curve.Evaluate(1, inter_s);
  const double dkappa = spiral_curve.Evaluate(2, inter_s);
  GlobalPathPoint path_point(theta, kappa, dkappa, start_s + inter_s, start_x + dx, start_y + dy);
  // cout << "path_point: " << path_point.x << " " << path_point.y << " " << path_point.s << " " << path_point.theta << " " << path_point.kappa << " " << path_point.dkappa << endl;
  return path_point;
}

GlobalPathPoint CoarsePathGenerator::FindNearestPt(
    const double start_x1, const double start_y1, const double start_s1,
    const double theta1, const double kappa1, const double dkappa1,
    const double start_x2, const double start_y2, const double start_s2,
    const double theta2, const double kappa2, const double dkappa2,
    const double theta3, const double kappa3, const double dkappa3,
    const double delta_s1, const double delta_s2, const double pt_x, const double pt_y)  {
    double angle_diff1 = AngleDiff(theta1, theta2);
    double angle_diff2 = AngleDiff(theta2, theta3);
  QuinticSpiralPath spiral_curve1(theta1, kappa1, dkappa1, theta1 + angle_diff1,
                                 kappa2, dkappa2, delta_s1); //五次多项式拟合计算该段的五次螺旋曲线theta(s)
  QuinticSpiralPath spiral_curve2(theta2, kappa2, dkappa2, theta2 + angle_diff2,
                                 kappa3, dkappa3, delta_s2); //五次多项式拟合计算该段的五次螺旋曲线theta(s)
  double new_s = 0, min_dis = DBL_MAX; //new_s是新的s值，min_dis是最小距离
  double  s, x, y, theta, kappa, dkappa; //存储最后结果
  while(new_s < delta_s1 + delta_s2) {
    if (new_s < delta_s1) {
      const double inter_s = new_s;
      const double dx = spiral_curve1.ComputeCartesianDeviationX<10>(inter_s);
      const double dy = spiral_curve1.ComputeCartesianDeviationY<10>(inter_s);
      double new_dis = hypot(start_x1 + dx -pt_x, start_y1 + dy - pt_y);
      if (new_dis < min_dis) {
        min_dis = new_dis;
        s = new_s;
      }
    }
    else {
      const double inter_s = new_s - delta_s1;
      const double dx = spiral_curve2.ComputeCartesianDeviationX<10>(inter_s);
      const double dy = spiral_curve2.ComputeCartesianDeviationY<10>(inter_s);
      double new_dis = hypot(start_x2 + dx -pt_x, start_y2 + dy - pt_y);
      if (new_dis < min_dis) {
        min_dis = new_dis;
        s = new_s;
      }
    }
    new_s += 0.1;
  }
  if (s < delta_s1) {
    // cout << "min_ind is in first segment" << ", s: " << s << endl;
    const double inter_s = s;
    const double dx = spiral_curve1.ComputeCartesianDeviationX<10>(inter_s);
    const double dy = spiral_curve1.ComputeCartesianDeviationY<10>(inter_s);
    theta = NormalizeAngle(spiral_curve1.Evaluate(0, inter_s));
    kappa = spiral_curve1.Evaluate(1, inter_s);
    dkappa = spiral_curve1.Evaluate(2, inter_s);
    x = start_x1 + dx;
    y = start_y1 + dy;
  }
  else {
    // cout << "min_ind is in second segment"  << ", s: " << s << endl;
    const double inter_s = s - delta_s1;
    const double dx = spiral_curve2.ComputeCartesianDeviationX<10>(inter_s);
    const double dy = spiral_curve2.ComputeCartesianDeviationY<10>(inter_s);
    theta = NormalizeAngle(spiral_curve2.Evaluate(0, inter_s));
    kappa = spiral_curve2.Evaluate(1, inter_s);
    dkappa = spiral_curve2.Evaluate(2, inter_s);
    x = start_x2 + dx;
    y = start_y2 + dy;
  }
  GlobalPathPoint path_point(theta, kappa, dkappa, start_s1 + s, x, y);
  return path_point;
}

GlobalPathPoint CoarsePathGenerator::FindNearestPt(
  const double start_x1, const double start_y1, const double start_s1,
  const double theta1, const double kappa1, const double dkappa1,
  const double theta2, const double kappa2, const double dkappa2,
  const double delta_s1, const double pt_x, const double pt_y)  {
  double angle_diff1 = AngleDiff(theta1, theta2);
  QuinticSpiralPath spiral_curve1(theta1, kappa1, dkappa1, theta1 + angle_diff1,
                                kappa2, dkappa2, delta_s1); //五次多项式拟合计算该段的五次螺旋曲线theta(s)
  double new_s = 0, min_dis = DBL_MAX; //new_s是新的s值，min_dis是最小距离
  double  s, x, y, theta, kappa, dkappa; //存储最后结果
  while(new_s < delta_s1) {
    const double inter_s = new_s;
    const double dx = spiral_curve1.ComputeCartesianDeviationX<10>(inter_s);
    const double dy = spiral_curve1.ComputeCartesianDeviationY<10>(inter_s);
    double new_dis = hypot(start_x1 + dx -pt_x, start_y1 + dy - pt_y);
    if (new_dis < min_dis) {
      min_dis = new_dis;
      s = new_s;
    }
    new_s += 0.1;
  }
  
    const double inter_s = s;
    const double dx = spiral_curve1.ComputeCartesianDeviationX<10>(inter_s);
    const double dy = spiral_curve1.ComputeCartesianDeviationY<10>(inter_s);
    theta = NormalizeAngle(spiral_curve1.Evaluate(0, inter_s));
    kappa = spiral_curve1.Evaluate(1, inter_s);
    dkappa = spiral_curve1.Evaluate(2, inter_s);
    x = start_x1 + dx;
    y = start_y1 + dy;

    GlobalPathPoint path_point(theta, kappa, dkappa, start_s1 + s, x, y);
    return path_point;
}