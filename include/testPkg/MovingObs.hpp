#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include </usr/local/include/eigen3/Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
using namespace Eigen;
using namespace std;

struct obs_state {
    double x, y, v, theta;
    // 默认构造函数
    obs_state() : x(0.0), y(0.0), v(0.0), theta(0.0) {}
    // 带参数的构造函数
    obs_state(double x, double y, double v, double theta) 
        : x(x), y(y), v(v), theta(theta) {}
};

// 预测轨迹结构，20s轨迹就可以
struct Trajectory {
    double time;  // 时间戳
    obs_state cur_state;
    std::vector<std::pair<double, double>> points; // 每 0.1s 的预测位置 (x, y)
    obs_state getPosWithRelativeTime(double rel_t) {
        obs_state pos; //x,y,theta,v
        double ind_double = rel_t * 10;
        int ind = static_cast<int>(std::floor(ind_double)); // 向下取整

        // 确保索引在有效范围内
        if (ind < 0 || ind >= points.size() - 1) {
            std::cerr << "Index out of range" << std::endl;
            return pos;
        }

        if (ind >= points.size() - 1) {
            // 如果索引超出有效范围，使用最后一段进行线性插值
            ind = points.size() - 2;
            ind_double = ind; // 更新 ind_double 以便计算 t
        }

        // 计算插值比例
        // double t = (rel_t - ind / 10) / 0.1;
        double t = ind_double - ind;

        // 获取索引为 ind 和 ind+1 的点
        auto& p0 = points[ind];
        auto& p1 = points[ind + 1];

        // 进行线性插值
        pos.x = (1 - t) * p0.first + t * p1.first;
        pos.y = (1 - t) * p0.second + t * p1.second;

        // 计算 theta 和 v
        double dx = p1.first - p0.first;
        double dy = p1.second - p0.second;
        pos.theta = std::atan2(dy, dx);
        pos.v = std::sqrt(dx * dx + dy * dy) / 0.1; // 假设时间间隔为 0.1s

        return pos;
    };
};

class MovingObs {
public:
	MovingObs() {
		// num = 2;
		// obs_traj.resize(2);
        // obs_traj[0] = loadTrajectoryData("opposite.csv");
        // obs_traj[1] = loadTrajectoryData("following.csv");
        // obs_traj[2] = loadTrajectoryData("stoping.csv");
	};
	static int num;
	static vector<vector<Trajectory>> obs_traj;

    // 加载 CSV 数据
    static std::vector<Trajectory> loadTrajectoryData(const std::string& filename) {
        std::vector<Trajectory> data;
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file: " + filename);
        }
        
        std::string line;
        bool header = true; // 跳过第一行标题
        while (std::getline(file, line)) {
            if (header) {
                header = false;
                continue;
            }
            std::stringstream ss(line);
            std::string cell;
            Trajectory traj;
            
            // 读取时间戳
            std::getline(ss, cell, ',');
            traj.time = std::stod(cell);
            
            double last_theta = 0;
            // 读取 (x, y) 点对
            while (std::getline(ss, cell, ',')) {
                double x = std::stod(cell);
                if (!std::getline(ss, cell, ',')) break;
                double y = std::stod(cell);
                traj.points.emplace_back(x, y);
            }
             traj.cur_state.x =  traj.points[0].first;
             traj.cur_state.y =  traj.points[0].second;
             if(traj.points[1].first != traj.points[0].first || traj.points[1].second != traj.points[0].second) {
                traj.cur_state.theta =  atan2(traj.points[1].second - traj.points[0].second,  traj.points[1].first - traj.points[0].first);
                last_theta = traj.cur_state.theta;
             }
             else
                traj.cur_state.theta =  last_theta;
             traj.cur_state.v =  hypot(traj.points[1].first - traj.points[0].first, traj.points[1].second - traj.points[0].second) / 0.1;
             cout << "cur_state: " <<   traj.cur_state.x << ", " <<  traj.cur_state.y << ", " <<traj.cur_state.theta <<  endl;
            data.push_back(traj);
        }
        file.close();
        return data;
    }

};