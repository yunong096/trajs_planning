#pragma once

#include <array>
#include <limits>

// #include "dp_planner.hpp"
#include "df_planner/cubic_spline.hpp"
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

  bool RunGlobalOpt(const vector<Vector2d>& raw_pt);
  bool RunGlobalOpt2(const vector<Vector2d>& raw_pt);

  bool CalKeyPoint(const vector<CartesianState>& source_path,
                              traj_utils::FlatTrajData* trajs, double duration);
    bool CalExtremePoint(const CartesianState& init_state,
                                            const CartesianState& end_node,
                                            traj_utils::FlatTrajData* trajs);
  void GetFlatState(const Eigen::Vector4d& state,
                    const Eigen::Vector2d& control_input,
                    Eigen::MatrixXd& flat_state);
    void getPolyTrajOpt(std::shared_ptr<PolyTrajOptimizer>& a){a = std::move(ploy_traj_opt_);};

 private:
  double end_curvature_ = 0.0;
  double end_heading_ = 0.0;
  TrajOptimizer traj_opt_;
  PolyTrajOptimizer::Ptr ploy_traj_opt_;
  double kRbCollisionBuffur_ = 0.0; //0.15;
  double kLbCollisionBuffur_ = 0.0; //0.15;
  bool is_debug_mode_ = false;
  bool need_extra_rb_buffer_ = false;

  std::vector<Eigen::MatrixXd> hPolys_, display_hPolys_;
    
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

    void findCorners(const std::vector<Vector2d>& pt_init, std::vector<int>& index_corner) {
        index_corner.clear();
        index_corner.emplace_back(0);
        int count = 0;
        for (size_t i = 1; i < pt_init.size() - 1; ++i) {
            double dx1 = pt_init[i](0) - pt_init[i - 1](0);
            double dx2 = pt_init[i + 1](0) - pt_init[i](0);
            double dy1 = pt_init[i](1) - pt_init[i - 1](1);
            double dy2 = pt_init[i + 1](1) - pt_init[i](1);
            double inner_product = dx1 * dx2 + dy1 * dy2;
            if (inner_product < 0) {
                count++; // 记录尖点个数
                index_corner.emplace_back(i);
            }
        }
        index_corner.emplace_back(pt_init.size() - 1);
        std::cout << "Number of corners: " << count << std::endl;
    }

    bool getRectangleConst(std::vector<Eigen::Vector3d> statelist){
        hPolys_.clear();
        double resolution = 0.1; //x1 y0.2   原来为1
        double step = resolution * 1.0;
        double limitBound = 30.0;
        //generate a rectangle for this state px py yaw
        for(const auto state : statelist){
            //generate a hPoly
            Eigen::MatrixXd hPoly;
            hPoly.resize(4, 4);
            Eigen::Matrix<int,4,1> NotFinishTable = Eigen::Matrix<int,4,1>(1,1,1,1);      
            Eigen::Vector2d sourcePt = state.head(2);
            Eigen::Vector2d rawPt = sourcePt;
            double yaw = state[2];
            bool test = false;
            traj_utils::VehicleParam sourceVp,rawVp;
            Eigen::Matrix2d egoR;
            egoR << cos(yaw), -sin(yaw),
                    sin(yaw), cos(yaw);
            traj_utils::VehicleParam vptest;
            // map_itf_->CheckIfCollisionUsingPosAndYaw(vptest,state,&test); //检测给的初始点是否合规
    
            Eigen::Vector4d expandLength;
            expandLength << 0.0, 0.0, 0.0, 0.0;
            //dcr width length
            while(NotFinishTable.norm()>0){ 
            //+dy  +dx -dy -dx  
                for(int i = 0; i<4; i++){
                    if(!NotFinishTable[i]) continue;
                    //get the new source and vp
                    Eigen::Vector2d NewsourcePt = sourcePt;
                    traj_utils::VehicleParam NewsourceVp = sourceVp;
                    Eigen::Vector2d point1,point2,newpoint1,newpoint2;

                    bool isocc = false;
                    switch (i)
                    {
                    //+dy
                    case 0: //左上左下，以及他们沿着横向扩张后的位置
                        point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
                        point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
                        newpoint1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);   
                        newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0+step);
                        //1 new1 new1 new2 new2 2
                        CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        expandLength[i] += step;
                        if(expandLength[i] >= limitBound){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,step/2.0);
                        NewsourceVp.set_width(NewsourceVp.width() + step);
                        sourcePt = NewsourcePt;
                        sourceVp = NewsourceVp;
                        break;
                    //+dx
                    case 1: //右上左上，沿纵向扩张
                        point1 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
                        point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
                        newpoint1 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);   
                        newpoint2 = sourcePt + egoR * Eigen::Vector2d(step+sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
                        //1 new1 new1 new2 new2 2
                        CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        expandLength[i] += step;
                        if(expandLength[i] >= limitBound){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(step/2.0,0.0);
                        NewsourceVp.set_length(NewsourceVp.length() + step);
                        sourcePt = NewsourcePt;
                        sourceVp = NewsourceVp;
                        break;
                    //-dy
                    case 2: //右下右上，沿横向扩张
                        point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
                        point2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
                        newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);   
                        newpoint2 = sourcePt + egoR * Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0-step);
                        //1 new1 new1 new2 new2 2
                        CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        expandLength[i] += step;
                        if(expandLength[i] >= limitBound){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(0,-step/2.0);
                        NewsourceVp.set_width(NewsourceVp.width() + step);
                        sourcePt = NewsourcePt;
                        sourceVp = NewsourceVp;
                        break;
                    //-dx
                    case 3: //左下右下，沿纵向扩张
                        point1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
                        point2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
                        newpoint1 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,sourceVp.width()/2.0);
                        newpoint2 = sourcePt + egoR * Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr()-step,-sourceVp.width()/2.0);
                        //1 new1 new1 new2 new2 2
                        CheckIfCollisionUsingLine(point1,newpoint1,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint1,newpoint2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        CheckIfCollisionUsingLine(newpoint2,point2,&isocc,resolution/2.0);
                        if(isocc){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        expandLength[i] += step;
                        if(expandLength[i] >= limitBound){
                        NotFinishTable[i] = 0.0;
                        break;
                        }
                        NewsourcePt = NewsourcePt + egoR * Eigen::Vector2d(-step/2.0,0.0);
                        NewsourceVp.set_length(NewsourceVp.length() + step);
                        sourcePt = NewsourcePt;
                        sourceVp = NewsourceVp;
                        break;
                    }   
                }
            }
            Eigen::Vector2d point1,norm1;
            point1 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],rawVp.width()/2.0+expandLength[0]);
            norm1 << -sin(yaw), cos(yaw);
            hPoly.col(0).head<2>() = norm1;
            hPoly.col(0).tail<2>() = point1;
            Eigen::Vector2d point2,norm2;
            // point2 = sourcePt+egoR*Eigen::Vector2d(sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            point2 = rawPt+egoR*Eigen::Vector2d(rawVp.length()/2.0+rawVp.d_cr()+expandLength[1],-rawVp.width()/2.0-expandLength[2]);
            norm2 << cos(yaw), sin(yaw);
            hPoly.col(1).head<2>() = norm2;
            hPoly.col(1).tail<2>() = point2;
            Eigen::Vector2d point3,norm3;
            // point3 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),-sourceVp.width()/2.0);
            point3 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],-rawVp.width()/2.0-expandLength[2]);
            norm3 << sin(yaw), -cos(yaw);
            hPoly.col(2).head<2>() = norm3;
            hPoly.col(2).tail<2>() = point3;
            Eigen::Vector2d point4,norm4;
            // point4 = sourcePt+egoR*Eigen::Vector2d(-sourceVp.length()/2.0+sourceVp.d_cr(),sourceVp.width()/2.0);
            point4 = rawPt+egoR*Eigen::Vector2d(-rawVp.length()/2.0+rawVp.d_cr()-expandLength[3],rawVp.width()/2.0+expandLength[0]);
            norm4 << -cos(yaw), -sin(yaw);
            hPoly.col(3).head<2>() = norm4;
            hPoly.col(3).tail<2>() = point4;
            hPolys_.push_back(hPoly);
        };
        return true;
    }

    void CheckIfCollisionUsingLine(const Eigen::Vector2d p1, 
                                           const Eigen::Vector2d p2, bool* res, double checkl){
        for(double dl = 0.0; dl < (p2-p1).norm(); dl+=checkl){
            Eigen::Vector2d pos = (p2-p1)*dl/(p2-p1).norm()+p1;
            if(CheckCollisionUsingGlobalPosition(pos)){
                *res = true;
                return;
            }
        }
        *res =  CheckCollisionUsingGlobalPosition(p2);
    }

    bool CheckCollisionUsingGlobalPosition(const Eigen::Vector2d p1) {
        double x = p1(0);
        double y = p1(1);
        // if(x >= 7.7 && x <= 83.8 && y >= 61.3 && y <= 68.3)
        //     return false;
        // else if (x >= 76.7 && x <= 83.8 && y >= 50.2 && y <= 61.3)
        //     return false;
        // else if (x >= 76.7 && x <= 138.5 && y >= 43.1 && y <= 50.2)
        //     return false;
        // else if (x >= 99.3 && x <= 101.9 && y >= 50.2 && y <= 55.9)
        //     return false;
        // else if (x >= 0 && x <= 7.7 && y >= 61.3 && y <=80)
        //     return false;

        if(x >= 0 && x <= 7.7 && y >= 0 && y <= 80)
            return false;
        else if (x >= 7.7 && x <= 28.5 && y >= 61.3 && y <=80)
            return false;
        else if (x >= 7.7 && x <= 83.8 && y >= 6.3 && y <= 13.4)
            return false;
        else if (x >= 51.84 && x <= 54.44 && y >= 13.4 && y <= 19.1)
            return false;
        return true;
    }

    Vector2d interpolate(const std::vector<double>& x,
        const std::vector<double>& s,
        double s0) {
        // 检查边界
        int n = s.size();
        if (s0 <= s.front()) {
        return Vector2d{x.front(), (x[1] - x[0]) / (s[1] - s[0])};
        }
        if (s0 >= s.back()) {
        return Vector2d{x.back(), (x[n-1] - x[n-2]) / (s[n-1] - s[n-2])};
        }

        // 查找区间
        size_t i = 0;
        while (i < s.size() - 1 && s[i + 1] < s0) {
        i++;
        }

        // 线性插值
        double ratio = (s0 - s[i]) / (s[i + 1] - s[i]);
        double x_interp = x[i] + ratio * (x[i + 1] - x[i]);

        return Vector2d{x_interp, (x[i+1] - x[i]) / (s[i+1] - s[i])};
    }

    // 计算勒让德多项式 P_n(x)
    double legendre(int n, double x) {
        if (n == 0) return 1.0;
        if (n == 1) return x;

        double P0 = 1.0;
        double P1 = x;
        double Pn = 0.0;

        for (int i = 2; i <= n; ++i) {
            Pn = ((2 * i - 1) * x * P1 - (i - 1) * P0) / i;
            P0 = P1;
            P1 = Pn;
        }

        return Pn;
    }

    // 计算勒让德多项式的导数 dP_n(x)/dx
    double legendreDerivative(int n, double x) {
        if (n == 0) return 0.0;
        if (n == 1) return 1.0;

        return (n * (legendre(n - 1, x) - x * legendre(n, x))) / (1 - x * x);
    }

    // 使用牛顿迭代法计算勒让德多项式的根
    double findLegendreRoot(int n, double initialGuess, double tolerance = 1e-10) {
        double x = initialGuess;
        double delta;

        do {
            double Pn = legendre(n, x);
            double dPn = legendreDerivative(n, x);
            delta = Pn / dPn;
            x -= delta;
        } while (std::abs(delta) > tolerance);

        return x;
    }

    // 计算高斯节点和权重
    void gaussNodesAndWeights(int n, std::vector<double>& nodes, std::vector<double>& weights) {
        nodes.resize(n);
        weights.resize(n);

        for (int i = 0; i < n; ++i) {
            // 初始猜测值：使用切比雪夫节点的近似值
            double initialGuess = std::cos(M_PI * (i + 0.75) / (n + 0.5));
            nodes[i] = findLegendreRoot(n, initialGuess);

            // 计算权重
            weights[i] = 2.0 / ((1 - nodes[i] * nodes[i]) * std::pow(legendreDerivative(n, nodes[i]), 2));
        }
    }
};
