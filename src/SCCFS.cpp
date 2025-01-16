#include "SCCFS.hpp"
#include <chrono>

void SCCFS::ComputeCostFunction() {
	// 计算 Q1
	MatrixXd Q1 = MatrixXd::Identity(nstep * dim, nstep * dim);
	MatrixXd Q1_new = processQ(Q1, vector <int>{0, 1});
	 Q1 = Q1_new.transpose() * Q1_new;

	// 计算 Q2 (速度差异)
	MatrixXd Vdiff = MatrixXd::Identity(nstep * dim, nstep * dim) - createDiagonalMatrix(nstep * dim, dim);
	MatrixXd Vdiff_new = processQ(Vdiff, vector <int>{0, 1});
	MatrixXd Q2 = Vdiff_new.topRows((nstep - 1) * 2).transpose() * Vdiff_new.topRows((nstep - 1) * 2);

	// 计算 Q3 (加速度差异)
	MatrixXd Adiff = MatrixXd::Identity(nstep * dim, nstep * dim) - 2 * createDiagonalMatrix(nstep * dim, dim) + createDiagonalMatrix(nstep * dim, 2 * dim);;
	MatrixXd Adiff_new = processQ(Adiff, vector <int>{0, 1});
	MatrixXd Q3 = Adiff_new.topRows((nstep - 2) * 2).transpose() * Adiff_new.topRows((nstep - 2) * 2);

	// 计算 Q4 (松弛变量)
	MatrixXd Q4 = MatrixXd::Zero(nstep * 2, nstep * dim);
	for (int i = 0; i < nstep; ++i) {
		for (int j = 0; j < 2; ++j) {
			Q4(i * 2 + j, i * dim + 2 + j) = 1;
		}
	}
	Q4 = Q4.transpose() * Q4;

	// 计算 Q5 (转角变化)
	MatrixXd Ddiff = MatrixXd::Identity(nstep * dim, nstep * dim) - createDiagonalMatrix(nstep * dim, dim);
	MatrixXd Ddiff_new = processQ(Vdiff, vector <int>{7});
	MatrixXd Q5 = Ddiff_new.topRows(nstep - 1).transpose() * Ddiff_new.topRows(nstep - 1);

	// 计算 Q6 (acc变化)
	MatrixXd Jdiff = MatrixXd::Identity(nstep * dim, nstep * dim) - createDiagonalMatrix(nstep * dim, dim);
	MatrixXd Jdiff_new = processQ(Vdiff, vector <int>{6});
	MatrixXd Q6 = Jdiff_new.topRows(nstep - 1).transpose() * Jdiff_new.topRows(nstep - 1);

	// 最终代价函数结合权重
	VectorXd cref(6);  // cref: 参考权重
	cref << 1, 0, 0, 0, 0, 0;
	VectorXd cabs(6);  // cabs: 松弛变量权重
	cabs << 0, 0, 0.01, 10000, 1, 1;

	// 总代价函数
	Qref = Q1 * cref(0) / nstep + Q2 * cref(1) * (nstep - 1) + Q3 * cref(2) * pow(nstep - 1, 4) / (nstep - 2) + Q4 * cref(3) / nstep  + Q5 * cref(4) * (nstep - 1)  + Q6 * cref(5) * (nstep - 1);
	Qabs = Q1 * cabs(0) / nstep + Q2 * cabs(1) * (nstep - 1) + Q3 * cabs(2) * pow(nstep - 1, 4) / (nstep - 2) + Q4 * cabs(3) / nstep  + Q5 * cabs(4) * (nstep - 1)  + Q6 * cabs(5) * (nstep - 1);

	//等式约束
	// 计算矩阵的大小  
		AeqRows = 8;  
		if (!isEndingConstrained) AeqRows = 4;  
		AeqCols = nstep * dim;  
	
		// 使用Eigen定义矩阵和向量  
		Aeq = Eigen::MatrixXd::Zero(AeqRows, AeqCols);  
		// 初始化起点和终点位姿约束  
		Aeq.block(0, 0, 2, 2) = Eigen::MatrixXd::Identity(2, 2);  
		Aeq.block(2, 4, 2, 2) = Eigen::MatrixXd::Identity(2, 2);  
		Aeq.block(4, (nstep - 1) * dim, 2, 2) = Eigen::MatrixXd::Identity(2, 2);
		Aeq.block(6, (nstep - 1) * dim + 4, 2, 2) = Eigen::MatrixXd::Identity(2, 2);  
	
		// // 初始化beq  
		beq = start;
		beq.conservativeResize(8); // 将vec1的大小改为8
    	beq.tail<4>() = ending;
}

bool SCCFS::solveCFS() {
	auto start_time = std::chrono::high_resolution_clock::now();
	ComputeCostFunction();
	int max_iter = 100;
	for (int k = 0; k < max_iter; ++k) {
		cout << "Step " <<  k + 1  << endl;
		MatrixXd Lstack(0, nstep * dim );
		VectorXd Sstack ;
		VectorXd Lowstack;
		VectorXd frontpos = FrontPos(refpath);
		/*********************不等式约束：前后轴的位置避碰约束、上下界**********************/
		for (int i = 0; i < nstep; ++i) {
			// 添加障碍约束，位置计算
			for (int j = 0; j < obs.getNum(); ++j) {
				auto cur_obs = obs.getObs()[j];
				MatrixXd poly(2, cur_obs.vertex_x.size());
				for (int i = 0; i < cur_obs.vertex_x.size(); ++i) {
					poly(0, i) = cur_obs.vertex_x[i];
					poly(1, i) = cur_obs.vertex_y[i];
				}
				poly += cur_obs.speed * MatrixXd::Ones(1, 4) * dt * i; //这里与原代码不一致，差了dt，4表示4个顶点
				// 后轴位置
				Vector2d ref_pos = refpath.segment(i * dim, 2);
				VectorXd L;
				double S, d;
				d2poly(ref_pos, poly, L, S, d);  // 计算到多边形的距离
				L.conservativeResize(L.size() + 1);
				L(L.size() - 1) = -1; //后轴位置
				// printMatrix(L);

				// 加入到约束栈
				MatrixXd L_temp = MatrixXd::Zero(1,  nstep * dim);
				L_temp.block(0, i * dim, 1, L.size()) = L.transpose();
				Lstack.conservativeResize(Lstack.rows() + 1, Lstack.cols());
				Lstack.row(Lstack.rows() - 1) = L_temp;
				Sstack.conservativeResize(Sstack.size() + 1);
				Sstack(Sstack.size() - 1) = S - margin;

				// 处理前轮位置
				if (i < nstep - 1) {
					ref_pos = (frontpos).segment(i * 2, 2);
					d2poly(ref_pos, poly, L, S, d);  // 计算到多边形的距离
					MatrixXd A = calculatePos(i);
					MatrixXd B = calculatePos(i + 1);
					MatrixXd As =  MatrixXd::Zero(1, dim * nstep);
					As(0, i * dim + 3) = -1;
					// 更新约束
					MatrixXd C = B - A;
					VectorXd X0 = C * refpath;
					VectorXd fX0 = X0 / X0.norm();
					MatrixXd dfX0 = C / X0.norm() - X0 * X0.transpose() * C / pow(X0.norm(), 3);

					MatrixXd L_f = L.transpose() * (A + WheelBase * dfX0) + As;
					double S_f = S - margin + L.transpose() * WheelBase * (dfX0 * refpath - fX0);

					// 更新约束栈
					Lstack.conservativeResize(Lstack.rows() + 1, Lstack.cols());
					Lstack.row(Lstack.rows() - 1) = L_f;
					Sstack.conservativeResize(Sstack.size() + 1);
					Sstack(Sstack.size() - 1) = S_f;
				}
			}
			//sk >= 0
			MatrixXd Ls = MatrixXd::Zero(2,  nstep * dim);
			Ls(0, i * dim + 2) = -1;
			Ls(1, i * dim + 3) = -1;
			Lstack.conservativeResize(Lstack.rows() + 2, Lstack.cols());
			Lstack.block(Lstack.rows() - 2, 0, 2, dim * nstep) = Ls; ////////////Lstack.block(Lstack.rows() - 3, 0, 2, dim * nstep) = Ls;
			Sstack.conservativeResize(Sstack.size() + 2);
			Sstack(Sstack.size() - 2) = 0;
			Sstack(Sstack.size() - 1) = 0;
			
			//速度、加速度、前轮转角上下界约束 
			Ls = MatrixXd::Zero(6,  nstep * dim);
			Ls(0, i * dim + 4) = -1; Ls(1, i * dim + 4) =  1;
			Ls(2, i * dim + 6) = -1; Ls(3, i * dim + 6) =  1;
			Ls(4, i * dim + 7) = -1; Ls(5, i * dim + 7) =  1;
			Lstack.conservativeResize(Lstack.rows() + 6, Lstack.cols());
			Lstack.block(Lstack.rows() - 6, 0, 6, dim * nstep) = Ls; ////////////Lstack.block(Lstack.rows() - 3, 0, 2, dim * nstep) = Ls;
			Sstack.conservativeResize(Sstack.size() + 6);   
			VectorXd newValues(6); //
   			newValues << 2, 3.6, 0.4, 0.4, 0.6, 0.6;
    		Sstack.tail<6>() = newValues; 
		}

		/*********************等式约束：osqp没有等式约束定义选项，将不等式约束的上下限设为等值，包括起点和终点的位置和速度约束**********************/ 
		// The car model constraint
		MatrixXd Aeq1 = MatrixXd::Zero(4*(nstep-1), dim*nstep);
		MatrixXd Aeq2 = MatrixXd::Zero(4*(nstep-1), dim*nstep);
		MatrixXd Aeq3 = MatrixXd::Zero(4*(nstep-1), dim*nstep);
		VectorXd beq1 = VectorXd::Zero(4*(nstep-1));
		for (int i = 0; i < nstep; ++i) {
				VectorXd xr(6);
				xr << oripath(dim*i ),  oripath(dim*i + 1), oripath(dim*i + 4), oripath(dim * i + 5),oripath(dim*i + 6),oripath(dim*i + 7);
				MatrixXd Acon(4, 6), Acon2(4, dim);
				Acon <<  0, 0, cos(xr(3)), -xr(2) * sin(xr(3)), 0, 0,
								0, 0, sin(xr(3)), xr(2) * cos(xr(3)), 0, 0,
								0, 0, 0, 0, 1, 0,
								0, 0, tan(xr(5)) / WheelBase, 0, 0, xr(2) / (WheelBase * pow(cos(xr(5)), 2));
				Acon2 <<  0, 0, 0,0,cos(xr(3)), -xr(2) * sin(xr(3)), 0, 0,
								0, 0, 0,0, sin(xr(3)), xr(2) * cos(xr(3)), 0, 0,
								0, 0, 0,0, 0, 0, 1, 0,
								0, 0, 0,0, tan(xr(5)) / WheelBase, 0, 0, xr(2) / (WheelBase * pow(cos(xr(5)), 2));
				if (i != 0) {
					int c = 0;
					for(int j = 0; j < dim; ++j) {
						if(j != 2 && j != 3 && j <=5) {
							Aeq1(4 * (i - 1) + c, dim * i + j) = 1;
							++c;
						}
					}
				}
				if (i != nstep - 1) {
					int c = 0;
					for(int j = 0; j < dim; ++j) {
						if(j != 2 && j != 3 && j <=5) {
							Aeq1(4 * i  + c, dim * i + j) = 1;
							++c;
						}
					}
				}
				if (i < nstep - 1) {
						Aeq3.block(4 * i,  dim * i, 4, dim) = Acon2;
						VectorXd dxr(4);
						dxr << xr(2) * cos(xr(3)),  xr(2) * sin(xr(3)),  xr(4),  xr(2) * tan(xr(5)) / WheelBase;
						beq1.segment(4 * i, 4) = dt * dxr - Acon * xr;
				}
		}
		MatrixXd Aeq_final = Aeq;
		VectorXd beq_final = beq;
		Aeq_final.conservativeResize(AeqRows + 4 * (nstep - 1), Aeq_final.cols());
		Aeq_final.block(AeqRows, 0,4 * (nstep - 1), Aeq_final.cols()) =  Aeq1 - Aeq2 - dt * Aeq3;
		beq_final.conservativeResize(AeqRows + 4 * (nstep - 1));
		beq_final.segment(AeqRows,  4 * (nstep - 1)) = beq1;
		AeqRows  += 4 * (nstep - 1);
		Lstack.conservativeResize(Lstack.rows() + AeqRows, Lstack.cols());
		Lstack.block(Lstack.rows() - AeqRows, 0, AeqRows, dim * nstep) = Aeq_final;
		Sstack.conservativeResize(Sstack.size() + AeqRows);
		Sstack.segment(Sstack.size() -  AeqRows, AeqRows) = beq_final;
		Lowstack = VectorXd(Sstack.size()); // 使用已有的数据数组初始化矩阵
		Lowstack.setConstant(numeric_limits<double>::lowest()); // 使用已有的数据数组初始化矩阵
		Lowstack.segment(Sstack.size() -  AeqRows, AeqRows) = beq_final;

		/********************************二次归划*******************************/
		Eigen::SparseMatrix<double> hessian;      //H: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
		Eigen::VectorXd gradient;      //H: n*n正定矩阵,必须为稀疏矩阵SparseMatrix
		Eigen::SparseMatrix<double> linearMatrix; //A: m*n矩阵,必须为稀疏矩阵SparseMatrix
		MatrixXd Qh = Qref + Qabs;
		hessian = Qh.sparseView();
		gradient = -Qref * oripath;
		linearMatrix = Lstack.sparseView();
		// instantiate the solver
		OsqpEigen::Solver solver;

		// settings
		// solver.settings()->setVerbosity(false);
		// solver.settings()->setWarmStart(true);

		// // set the initial data of the QP solver
		// solver.data()->setNumberOfVariables(nstep * dim);   //变量数n
		// solver.data()->setNumberOfConstraints(Sstack.size()); //约束数m
		// if (!solver.data()->setHessianMatrix(hessian)) {
		// 	// hessian必须为sparse，否则类型不匹配
		// 	cout << "Hessian Error" << endl;
		// 	return false;
		// 	}
		// if (!solver.data()->setGradient(gradient)){
		// 	cout << "Gradient Error" << endl;
		// 	return false;
		// }
		// if (!solver.data()->setLinearConstraintsMatrix(linearMatrix)){
		// 	cout << "LinearConstraints Error" << endl;
		// 	return false;
		// }
		// if (!solver.data()->setUpperBound(Sstack)){
		// 	cout << "UpperBound Error" << endl;
		// 	return false;
		// }	
		// if (!solver.data()->setLowerBound(Lowstack)){
		// 	cout << "LowerBound Error" << endl;
		// 	return false;
		// }	

		// // instantiate the solver
		// if (!solver.initSolver()){
		// 	cout << "InitSolve Failed" << endl;
		// 	return false;
		// }
			

		// Eigen::VectorXd soln;

		// // solve the QP problem
		// if (!solver.solve())
		// {
		// 	cout << "Solve Failed" << endl;
		// 	return false;
		// }

		// soln = solver.getSolution();
		// VectorXd pathnew = soln;
		// cout << "diff is : " << (refpath - pathnew).norm() << endl;

		// if ((refpath - pathnew).norm() < 0.01) {
		// 	auto finish_time = std::chrono::high_resolution_clock::now();
		// 	// 计算运行时间
		// 	std::chrono::duration<double> elapsed = finish_time - start_time;
		// 	cout << "Converged at step " << k + 1 <<  ", Elapsed time: " << elapsed.count() << endl;
		// 	return true;
		// }
		// refpath = pathnew;  // 更新路径
	}
	cout << "Solve Failed: reach max_iter" << endl;
	return false;
}

void SCCFS::d2poly(const Vector2d& point, const MatrixXd& poly, VectorXd& L, double& S, double& d) {
	d = numeric_limits<double>::infinity(); // 初始化距离为无穷大
	S = 0;
	L = VectorXd(2);
	int nside = poly.cols();
	int ii = -1;

	for (int i = 0; i < nside; ++i) {
		VectorXd p1 = poly.col(i);
		VectorXd p2 = poly.col((i + 1) % nside);

		if (i == 1 && poly(0, 1) - poly(0, 0) == -2 && poly(1, 1) - poly(1, 0)== 0) {
			// cout << "#######################################决策凸化测试" << endl;
			continue; //用于测试，对动态障碍决策凸化
		}

		// 计算三边的距离
		double trid[3];
		trid[0] = (p1 - p2).norm();
		trid[1] = (p1 - point).norm();
		trid[2] = (p2 - point).norm();

		Vector2d Lr(p1(1) - p2(1), p2(0) - p1(0));
		double Sr = -p1(0) * p2(1) + p2(0) * p1(1);
		double vd = fabs(Lr(0) * point(0) + Lr(1) * point(1) - Sr) / trid[0];

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