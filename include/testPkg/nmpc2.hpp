#ifndef NMPC1_H_
#define NMPC1_H_

#include <casadi/casadi.hpp>
#include <iostream>
// #include "dp_planner.hpp"
#include <ros/ros.h>
#include <iostream>
#include "DynamicPathStruct.hpp"
#include "Obstacles.hpp"
#include "CoarsePathGenerator.hpp"
using namespace casadi;

class Mpc
{
private:
    double N_ = 60;
    double dt_ = 0.1;
    double v_max_ = 15/3.6;
    double acc_max_ = 6;
    double delta_max_ = 0.52;
    double acc_min_ ;
    double delta_min_ ;
    DM Q_, R_, S_;
    MX X, U, S;
    std::unique_ptr<casadi::OptiSol> solution_;
    vector<CartesianState> opt_path;
    Function kinematic_equation_;
    Function setKinematicEquation();
    int obs_num = 0, frame_count = 0; 
    double triArea(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3) {
		return 0.5 * fabs(p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) + p3(0) * (p1(1) - p2(1)));
	}
    std::vector<float> FrontPos(const vector<CartesianState> &x) {
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
public:
    Mpc(int count);
    ~Mpc() {};
    bool solve(CartesianState& current_state,
                                                        std::vector<GlobalPathPoint>& ori_states,
                                                        std::vector<CartesianState>& ref_states,
                                                        Obstacles& obs);

    void setWeights(vector<double> weights);
    
    std::vector<Eigen::Matrix<float,3,1>> get_predict_trajectory();

    std::vector<CartesianState> getSoluPath() {
        // cout << "start to get final path" << endl;
        std::vector<CartesianState> best_path;
        for(int i = 0; i < N_ + 1; ++i) {
            // cout << "i: " << i << endl;
            CartesianState state;
            state.x = static_cast<double>(solution_->value(X)(0, i));
            state.y = static_cast<double>(solution_->value(X)(1, i));
            state.speed = static_cast<double>(solution_->value(X)(2, i));
            state.theta = static_cast<double>(solution_->value(X)(3, i));
            if (i < N_) {
                state.acc = static_cast<double>(solution_->value(U)(0, i));
                // state.kappa = static_cast<double>(solution_->value(U)(1, i));
                state.kappa = tan(static_cast<double>(solution_->value(U)(1, i))) / 2.7;
            }
            else {
                state.acc = 0;
                state.kappa = 0;
            }
            best_path.emplace_back(state);
            // cout << "state: " << state.x << " " << state.y << " " << state.speed << " " << state.theta << " " << state.acc << " " << state.kappa << endl;
        }
        return best_path;
    }

    std::vector<CartesianState> getFinalPath() {
        for(int i = 0; i < 8; ++i) {
            auto state = opt_path[i];
            cout << "state: " << state.x << " " << state.y << " " << state.speed << " " << state.theta << " " << state.acc << " " << state.kappa << endl;
        }
        return opt_path;
    }
};



#endif