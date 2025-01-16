#include <vector>
#include <iostream>
#include "VehicleParam.hpp"
#include "Obstacles.hpp"
#include </usr/local/include/eigen3/Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
# include <cstddef>
using namespace Eigen;
using namespace std;

class SCCFS {
public:
	SCCFS() {
		nstep = 30;
		dim = 8; 
		dt = 0.4; 
		margin = 1.6; 
		start  = VectorXd(4);
		start  << -12, 10,0,0;
		ending  = VectorXd(4);
		ending  << 2.5, 4,0,0;
		obs = Obstacles();
	}
	Obstacles obs;
	void SetStartAndEnd(const VectorXd& s0, const VectorXd& sf) { start = s0; ending = sf; }
	void InitPath() {
		path = MatrixXd(2, nstep);
		// 初始化路径
		for (int i = 1; i <= nstep; ++i) {
			path.col(i - 1) = ((nstep - i) / double(nstep - 1)) * start.segment(0,2) + ((i - 1) / double(nstep - 1)) * ending.segment(0,2);
		}
		// 将路径转换为一维列向量
		refpath.resize(nstep * dim);
		int index = 0;
		for (int i = 0; i < nstep; ++i) {
			refpath.segment(index, 2) = path.col(i);
			index += 2;
			refpath.segment(index, dim - 2).setZero();  // 其他维度设置为0
			index += dim - 2;
		}
		oripath = refpath;
	}
	void InitPath(const MatrixXd& coarsepath) {
		oripath = refpath = coarsepath;
		nstep = coarsepath.size() / dim;
	}
	void ComputeCostFunction();
	bool solveCFS();
	VectorXd FrontPos(const VectorXd &x) {
		VectorXd z_new(nstep * dim);
		double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		double dis = 0, cos_the = 0, sin_the = 0;
		for (int i = 0; i < nstep; ++i) {
			x1 = x(i * dim);
			y1 = x(i * dim + 1);
			if (i < nstep - 1) {
				x2 = x((i + 1) * dim);
				y2 = x((i + 1) * dim + 1);
				dis = hypot(x1 - x2, y1 - y2);
				cos_the = (x2 - x1) / dis;
				sin_the = (y2 - y1) / dis;
			}
			double x_new = x1 + WheelBase * cos_the;
			double y_new = y1 + WheelBase * sin_the;
			z_new(i * dim) = x_new;
			z_new(i * dim + 1) = y_new;
			// for (int j = 0; j < dim - 2; ++j) {
			// 	z_new(i * dim + 2 + j) = x(i * dim + 2 + j);
			// }
		}
		return z_new;
	}
	VectorXd FrontPos2(const VectorXd &x) {
		VectorXd z_new(nstep * dim);
		double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
		double dis = 0, cos_the = 0, sin_the = 0;
		for (int i = 0; i < nstep; ++i) {
			x1 = x(i * dim);
			y1 = x(i * dim + 1);
			cos_the = cos(x(i * dim + 5));
			sin_the = sin(x(i * dim + 5));
			double x_new = x1 + WheelBase * cos_the;
			double y_new = y1 + WheelBase * sin_the;
			z_new(i * dim) = x_new;
			z_new(i * dim + 1) = y_new;
			// for (int j = 0; j < dim - 2; ++j) {
			// 	z_new(i * dim + 2 + j) = x(i * dim + 2 + j);
			// }
		}
		return z_new;
	}
	void d2poly(const Vector2d& point, const MatrixXd& poly, VectorXd& L, double& S, double& d) ;
	VectorXd getRefpath() {return refpath;}
	VectorXd getInitpath() {return path;}
private:
	int nstep; // number of sampling points on the path
	int dim; // dimention of the problem
	double dt; // sampling time(s)
	double margin; //safety margin
	bool isEndingConstrained = true;
	VectorXd start, ending, refpath, oripath;
	MatrixXd path;
	MatrixXd Qref, Qabs;

	MatrixXd Aeq;
	VectorXd beq;
	int  AeqRows,  AeqCols;

	MatrixXd processQ(const MatrixXd& Q,  const vector<int>& dims) {
		// 提取x和y相关的矩阵行，处理Q矩阵
		MatrixXd Q_new;
		for (int i = 0; i < nstep * dim; ++i) {
			for(int d : dims) {
				if (i % dim == d ) {
					Q_new.conservativeResize(Q_new.rows() + 1, Q.cols());
					Q_new.row(Q_new.rows() - 1) = Q.row(i);
				}
			}
		}
		return Q_new;
	}
	MatrixXd calculatePos(const int& i) {
		//xk, yk
		MatrixXd A = MatrixXd::Zero(2, dim * nstep);
		A.block(0, i * dim, 2, 2) = MatrixXd::Identity(2, 2);
		return A;
	}
	double triArea(const Vector2d& p1, const Vector2d& p2, const Vector2d& p3) {
		return 0.5 * fabs(p1(0) * (p2(1) - p3(1)) + p2(0) * (p3(1) - p1(1)) + p3(0) * (p1(1) - p2(1)));
	}
	Eigen::MatrixXd createDiagonalMatrix(int n, int k) {  
		// 创建一个n阶方阵并初始化为0  
		Eigen::MatrixXd mat = Eigen::MatrixXd::Zero(n, n);  
		// 遍历矩阵元素  
		for (int i = 0; i < n; ++i) {  
			if(i + k < n){
				mat(i, i + k) = 1;
			}
		}  
		return mat;  
	}  

	void printMatrix(MatrixXd m) {
		cout << "Start to print Matrix: " << endl;
		for (int i = 0; i < m.rows(); ++i) {
			for (int j = 0; j < m.cols(); ++j) {
				cout << m(i, j) << " ";
			}
			cout << endl;
		}
	}

	void printSize(MatrixXd m) {
		cout << "Start to print Matrix Size: " << endl;
		cout << "rows: " << m.rows() << ", cols: " << m.cols() << endl;
	}
	
	// tuple<double, VectorXd, MatrixXd> cost_quadratic(const VectorXd& x, const VectorXd& ref, const MatrixXd& Qref, const MatrixXd& Qabs) {
	// 	// 计算目标函数值
	// 	double fun = (x.transpose() - ref.transpose()) * Qref * (x - ref) + x.transpose() * Qabs * x;

	// 	// 计算梯度
	// 	VectorXd grad = 2 * Qref * (x - ref) + 2 * Qabs * x;

	// 	// 计算 Hessian 矩阵
	// 	MatrixXd hess = 2 * Qref + 2 * Qabs;

	// 	return make_tuple(fun, grad, hess);
	// }
};