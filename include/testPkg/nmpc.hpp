#ifndef NMPC1_H_
#define NMPC1_H_

#include <casadi/casadi.hpp>
#include <iostream>
// #include "DynamicPathStruct.hpp"
// #include "CoarsePathGenerator.hpp"
// #include "Obstacles.hpp"
// #include "comfun.h"
#include "dp_planner.hpp"

class NMPC1
{
private:
    /* data */
    // MPC参数
    int m_predict_step; //一次采样间隔预测的步数
    float m_sample_time; //采样时间
    int m_predict_stage; //存储第几次调用nmpc
    int n_states; //共有n个状态-----3
    int n_controls; //共有n个控制输入-----2
    std::vector<Eigen::Matrix<float,3,1>> m_predict_trajectory; //预测轨迹 
    std::vector<float> m_initial_guess; //初始猜测解
    //符号定义
    casadi::Function m_solver; //求解器
    casadi::Function m_predict_fun; //预测函数
    std::map<std::string, casadi::DM> m_res; //求解结果
    std::map<std::string, casadi::DM> m_args; //求解参数
    //求解得到的控制量
    Eigen::Matrix<float, 2, 1> m_control_command; 
    std::vector<float> best_result; //最优状态


    int obs_num = 0, frame_count = 0; 
    double triArea(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3) {
		return 0.5 * fabs(p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) + p3(0) * (p1(1) - p2(1)));
	}
    void d2poly(const Vector2d& point, const MatrixXd& poly, VectorXd& L, double& S, double& d);
    std::vector<float> FrontPos(const vector<float> &x) {
		std::vector<float> z_new(2 * m_predict_step + 2);
		double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		double dis = 0, cos_the = 0, sin_the = 0;
		for (int i = 0; i < m_predict_step + 1; ++i) {
			x1 = x[i * 4];
			y1 = x[i * 4 + 1];
			if (i <  m_predict_step) {
				x2 = x[(i + 1) * 4];
				y2 = x[(i + 1) * 4 + 1];
				dis = hypot(x1 - x2, y1 - y2);
				cos_the = (x2 - x1) / dis;
				sin_the = (y2 - y1) / dis;
			}
			double x_new = x1 + 2.7 * cos_the;
			double y_new = y1 + 2.7 * sin_the;
			z_new[i * 2] = x_new;
			z_new[i * 2 + 1] = y_new;
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
 
public:
    NMPC1(int predict_step, float sample_time, int num, int f_count);
    ~NMPC1();
    //定义求解器
    void set_my_nmpc_solver();
    //优化求解
    void opti_solution(CartesianState& current_state,
                                                        std::vector<GlobalPathPoint>& ori_states,
                                                        std::vector<CartesianState>& ref_states,
                                                        Obstacles& obs);
    //获取最优控制向量
    Eigen::Matrix<float,2,1> get_controls();
    //获取预测轨迹
    std::vector<Eigen::Matrix<float,3,1>> get_predict_trajectory();

    std::vector<CartesianState> getFinalPath() {
        cout << "start to get final path" << endl;
        std::vector<CartesianState> best_path;
        for(int i = 0; i < m_predict_step + 1; ++i) {
            // cout << "i: " << i << endl;
            CartesianState state;
            state.x = best_result[i * 4];
            state.y = best_result[i * 4 + 1];
            state.speed = best_result[i * 4 + 2];
            state.theta = best_result[i * 4 + 3];
            if (i < m_predict_step) {
                state.acc = best_result[(m_predict_step + 1) * 6 + 2 * i];
                state.kappa = tan(best_result[(m_predict_step + 1) * 6 + 2 * i + 1]) / 2.7;
            }
            else {
                state.acc = 0;
                state.kappa = 0;
            }
            best_path.emplace_back(state);
            cout << "state: " << state.x << " " << state.y << " " << state.speed << " " << state.theta << " " << state.acc << " " << state.kappa << endl;
        }
        return best_path;
    }
};



#endif