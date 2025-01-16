#include <vector>
struct FrenetState {
    double s, l, ds, dds, dl, ddl;
    FrenetState(){}; // 默认构造函数
    FrenetState(double s_, double l_, double ds_, double dds_, double dl_, double ddl_) : s(s_), l(l_), ds(ds_), dds(dds_), dl(dl_), ddl(ddl_) {}
};

struct CartesianState {
    double x, y, theta, speed, acc, kappa;
    CartesianState(){};
    CartesianState(double x_, double y_, double theta_, double speed_, double acc_, double kappa_) : x(x_), y(y_), theta(theta_), speed(speed_), acc(acc_), kappa(kappa_) {}
};

// 车辆位置信息
struct car_state
{
  double x;
  double y;
  double z;
  double yaw; // 横摆角
  double vx;  // x方向速度值
  double vy;  // y方向速度值
  double v;   // 合速度
  double cur; // 曲率
};

// struct RefLine {
//     std::vector<double> s, x, y, theta, kappa, dkappa;
//     std::vector<double> all_s;
//     void generateAllS() {
//         all_s.push_back(0);
//         for (int i = 1; i < s.size(); ++i) {
//             all_s.push_back(all_s.back() + s[i]);
//         }
//     }
// };