#include "nmpc.hpp"

NMPC1::NMPC1(int predict_step, float sample_time, int num, int f_count)
{
    obs_num = num;
    frame_count = f_count;

    m_predict_step = predict_step;
    m_sample_time = sample_time;
    m_predict_stage = -1;

    n_states = 4;
    n_controls = 2;

    //设置求解器
    set_my_nmpc_solver();

}

NMPC1::~NMPC1()
{

}

//创建求解器
void NMPC1::set_my_nmpc_solver()
{
    //模型建立
    casadi::SX x = casadi::SX::sym("x");
    casadi::SX y = casadi::SX::sym("y");
    casadi::SX v = casadi::SX::sym("v");
    casadi::SX theta = casadi::SX::sym("theta");

    casadi::SX acc = casadi::SX::sym("acc");//加速度
    casadi::SX delta = casadi::SX::sym("delta"); //前轮转角

    //松弛因子
    casadi::SX s1 = casadi::SX::sym("s1"); //后轴松弛因子
    casadi::SX s2 = casadi::SX::sym("s2"); //前轴松弛因子

    casadi::SX states = casadi::SX::vertcat({x, y, v, theta});
    casadi::SX controls = casadi::SX::vertcat({acc, delta});

    // std::cout<<"n_states: "<< n_states<< endl;
    // std::cout<<"n_controls: "<< n_controls<< endl;

    //运动学模型参数

    //运动学模型
    casadi::SX rhs = casadi::SX::vertcat({
        v*casadi::SX::cos(theta),
        v*casadi::SX::sin(theta),
        acc,
        v *casadi::SX::tan(delta) / 2.7
    });

    //定义模型函数
    casadi::Function m_f = casadi::Function("f", {states, controls}, {rhs});

    // 求解问题符号表示
    casadi::SX U = casadi::SX::sym("U", n_controls, m_predict_step); //待求解的控制变量
    casadi::SX X = casadi::SX::sym("X", n_states, m_predict_step+1); //系统状态
    casadi::SX S = casadi::SX::sym("S", 2, m_predict_step+1); //松弛变量
    
    // //当前运动状态
    casadi::SX current_states = casadi::SX::sym("current_states",n_states);

    //优化参数（用于设置目标函数和避障约束）
    casadi::SX opt_para = casadi::SX::sym("opt_para",  n_states*(m_predict_step + 1) + obs_num * 3 * (m_predict_step + 1)  * 2); //每个位置的前后轴位置对于每个障碍物的约束

    //优化变量（需要求解的变量：X、S、U）
    casadi::SX opt_var = casadi::SX::vertcat({casadi::SX::reshape(X.T(),-1,1), casadi::SX::reshape(S.T(),-1,1), casadi::SX::reshape(U.T(),-1,1)});

    //根据上述模型函数向前预测运动状态
    // X(casadi::Slice(),0) = opt_para(casadi::Slice(0,4,1)); //状态初始值，索引左开右闭

    //不等式约束
    casadi::SX g = casadi::SX::zeros(n_states * m_predict_step + obs_num * (m_predict_step + 1) * 2, 1);

    int count = 0;
    for(int i = 0; i < m_predict_step + 1; ++i) {
        casadi::SX X_current = X(casadi::Slice(),i);
        casadi::SX S_current = S(casadi::Slice(),i);
        if(i < m_predict_step) {
            std::vector<casadi::SX> input_X;
            casadi::SX U_current = U(casadi::Slice(),i);
            input_X.emplace_back(X_current);
            input_X.emplace_back(U_current);
            casadi::SX error = X(casadi::Slice(),i+1) - m_f(input_X).at(0)*m_sample_time+X_current; //模型约束
            g(4 * i) = error(0);
            g(4 * i + 1) = error(1);
            g(4 * i + 2) = error(2);
            g(4 * i + 3) = error(3);
            // lbg(4 * i) = 0;
            // lbg(4 * i + 1) = 0;
            // lbg(4 * i + 2) = 0;
            // lbg(4 * i + 3) = 0;
        }
        for (int j = 0; j < obs_num; ++j) {
            casadi::SX Ab1 = opt_para(casadi::Slice(n_states*(m_predict_step + 1) + 3 * 2 * count, n_states*(m_predict_step + 1) + 3 * 2 * count + 3, 1));
            casadi::SX Ab2 = opt_para(casadi::Slice(n_states*(m_predict_step + 1) + 3 * 2 * count + 3, n_states*(m_predict_step + 1) + 3 * 2 * count + 6, 1));
            casadi::SX X_current_front = X_current(casadi::Slice(0,2,1)) + casadi::SX::vertcat({2.7*casadi::SX::cos(X_current(3)), 2.7*casadi::SX::sin(X_current(3))});
            g(n_states * m_predict_step + 2 * count) = Ab1(0) * X_current(0) + Ab1(1) * X_current(1) - Ab1(2) + S_current(0);
            g(n_states * m_predict_step + 2 * count + 1) = Ab2(0) * X_current_front(0) + Ab2(1) * X_current_front(1) - Ab2(2) + S_current(1);
            ++count;
        }
    }

    
    // for(int i = 0; i < m_predict_step + 1; ++i) {
    //     for (int j = 0; j < obs_num; ++j) {
    //         casadi::SX Ab1 = opt_para(casadi::Slice(n_states*(m_predict_step + 1) + 3 * 2 * count, n_states*(m_predict_step + 1) + 3 * 2 * count + 3, 1));
    //         casadi::SX Ab2 = opt_para(casadi::Slice(n_states*(m_predict_step + 1) + 3 * 2 * count + 3, n_states*(m_predict_step + 1) + 3 * 2 * count + 6, 1));
    //         casadi::SX X_current_front = X_current(casadi::Slice(0,2,1)) + casadi::SX::vertcat({2.7*casadi::SX::cos(X_current(3)), 2.7*casadi::SX::sin(X_current(3))});
    //         g(n_states * m_predict_step + 2 * count) = Ab1(0) * X_current(0) + Ab1(1) * X_current(1) - Ab1(2) + S_current(0);
    //         g(n_states * m_predict_step + 2 * count + 1) = Ab2(0) * X_current_front(0) + Ab2(1) * X_current_front(1) - Ab2(2) + S_current(1);
    //         ++count;
    //     }
    // }

    //控制序列与输出的关系函数（预测函数）
    // m_predict_fun = casadi::Function("m_predict_fun",{casadi::SX::reshape(U,-1,1),opt_para},{X});

    //惩罚矩阵
    casadi::SX m_Q = casadi::SX::zeros(4,4);
    casadi::SX m_R = casadi::SX::zeros(2,2);
    casadi::SX m_S = casadi::SX::zeros(2,2);
    m_Q(0,0) = 0.5;  //x和ref
    m_Q(1,1) = 0.5; //y和ref
    m_Q(2,2) = 0.1; //v和v_ref  10/3.6
    m_R(0,0) = 0.5;  //acc更小
    m_R(1,1) = 0.05;  //更少的转向
    m_S(0,0) = 5;  //s1更接近0
    m_S(1,1) = 5;  //s2更接近0
    
    //计算代价函数
    casadi::SX cost_fun = casadi::SX::sym("cost_fun");
    cost_fun = 0;

    int trajectory_points_index = 0;

    for(int k=0; k <= m_predict_step; ++k)
    {
        casadi::SX states_err = X(casadi::Slice(),k)-opt_para(casadi::Slice(4 * k, 4 * k + 4, 1));
        casadi::SX s_err = S(casadi::Slice(),k) - casadi::SX::zeros(2,1);
        cost_fun = cost_fun+casadi::SX::mtimes({states_err.T(),m_Q,states_err})+ casadi::SX::mtimes({s_err.T(),m_S,s_err});
        if(k <  m_predict_step) {
            casadi::SX controls_err = U(casadi::Slice(),k) - casadi::SX::zeros(2,1);
            cost_fun = cost_fun+casadi::SX::mtimes({controls_err.T(),m_R,controls_err});
        }        
    }

    //构建求解器(暂时不考虑约束)
    casadi::SXDict nlp_prob = {
        {"f", cost_fun},
        {"x", opt_var},
        {"p",opt_para},
        {"g", g}
    };

    std::string solver_name = "ipopt";
    casadi::Dict nlp_opts;
    nlp_opts["expand"] = true;
    nlp_opts["ipopt.max_iter"] = 5000;
    nlp_opts["ipopt.print_level"] = 0;
    nlp_opts["print_time"] = 0;
    nlp_opts["ipopt.acceptable_tol"] =  1e-6;
    nlp_opts["ipopt.acceptable_obj_change_tol"] = 1e-4;

    m_solver = nlpsol("nlpsol", solver_name, nlp_prob, nlp_opts);

    cout<<"solver is set"<<endl;
}

void NMPC1::opti_solution(CartesianState& current_state,
                                                        std::vector<GlobalPathPoint>& ori_states,
                                                        std::vector<CartesianState>& ref_states,
                                                        Obstacles& obs)
{
    //存入ref_states
    std::vector<float> ref_states_para(8*m_predict_step + 6, 0);
    std::vector<float> lbg(n_states * m_predict_step + obs_num * (m_predict_step + 1) * 2, numeric_limits<double>::lowest()); // 下界为负无穷大（表示没有下界，即g(x) <= ubg）
    std::vector<float> ubg(n_states * m_predict_step + obs_num * (m_predict_step + 1) * 2, 0.0); // 上界为0（表示g(x) <= 0）

    //设置ref_states_para
    for (int i = 0; i < m_predict_step + 1; ++i)
    {
        ref_states_para[4*i] = ref_states[i].x;
        ref_states_para[4*i+1] = ref_states[i].y;
        ref_states_para[4*i+2] = ref_states[i].speed;
        ref_states_para[4*i+3] = ref_states[i].theta;
        // ref_states_para[4 * (m_predict_step + 1) + 2*i] = 0; //s1
        // ref_states_para[4 * (m_predict_step + 1) + 2*i + 1] = 0; //s2
        if(i < m_predict_step)
        {
            ref_states_para[6 * (m_predict_step + 1) + 2*i] = ref_states[i].acc;
            ref_states_para[6 * (m_predict_step + 1) + 2*i + 1] = atan2(ref_states[i].kappa * 2.7, 1);
        }
        // cout << "ref_states_para: " << ref_states_para[4*i] << "," << ref_states_para[4*i+1] << "," << ref_states_para[4*i+2] << "," << ref_states_para[4*i+3] << endl;
    }

    //设置控制约束
    std::vector<float> lbx;
    std::vector<float> ubx;
    std::vector<float> parameters;
    //上下界设置
    lbx.emplace_back(current_state.x); //x
    ubx.emplace_back(current_state.x);
    lbx.emplace_back(current_state.y);//y
    ubx.emplace_back(current_state.y);
    lbx.emplace_back(current_state.speed);//v
    ubx.emplace_back(current_state.speed);
    lbx.emplace_back(current_state.theta);//theta
    ubx.emplace_back(current_state.theta);
    for (int k = 1; k < m_predict_step + 1; ++k)
    {
        lbx.emplace_back(0.0); //x
        ubx.emplace_back(140.0);
        lbx.emplace_back(0.0);//y
        ubx.emplace_back(80.0);
        lbx.emplace_back(0);//v
        ubx.emplace_back(15/3.6);
        lbx.emplace_back(std::numeric_limits<double>::lowest());//theta
        ubx.emplace_back(std::numeric_limits<double>::max());
    }
        for (int k = 0; k < m_predict_step + 1; ++k)
    {
        lbx.emplace_back(std::numeric_limits<double>::lowest()); //s1
        ubx.emplace_back(0.0);
        lbx.emplace_back(std::numeric_limits<double>::lowest());//s2
        ubx.emplace_back(0.0);
    }
        for (int k = 0; k < m_predict_step; ++k)
    {
        lbx.emplace_back(-5); //acc
        ubx.emplace_back(5);
        lbx.emplace_back(-0.52);//delta
        ubx.emplace_back(0.52);
    }
    
    //给opt_para赋值
    cout << "start to set parameters" << endl;
    ROS_WARN("CUR_STATE: %f, %f, %f, %f", current_state.x, current_state.y, current_state.speed, current_state.theta);
    parameters.emplace_back(current_state.x);
    parameters.emplace_back(current_state.y);
    parameters.emplace_back(current_state.speed);
    parameters.emplace_back(current_state.theta); //起点等式约束
    for(int j=1; j<m_predict_step+1; ++j)
    {
        parameters.emplace_back(ori_states[j].x);
        parameters.emplace_back(ori_states[j].y);
        parameters.emplace_back(10 / 3.6); //期望速度，后续用于构造目标函数
        parameters.emplace_back(ori_states[j].theta);
        // cout << "ori_states: " << ori_states[j].x << "," << ori_states[j].y << "," << ori_states[j].theta << endl;
    }
    
    cout << "ref_state_set_finished" << endl;
    //循环
    int loop_count = 0;
    while(loop_count < 100) {
        parameters.resize(n_states * (m_predict_step + 1));
        std::vector<float> ref_states_para_front = FrontPos(ref_states_para);
        for (int i = 0; i < m_predict_step + 1; ++i)
        {
            // for j = 0; j < obs_num; ++j
            for(auto j : obs.getMoveInds()) { 
                //获取障碍物位置
                //测试用，只有一个动态障碍
                auto cur_obs = obs.getObs()[j];
                MatrixXd poly(2, cur_obs.vertex_x.size());
                for (int k = 0; k < cur_obs.vertex_x.size(); ++k) {
                    poly(0, k) = cur_obs.vertex_x[k];
                    poly(1, k) = cur_obs.vertex_y[k];
                }
                poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.5 *  frame_count + m_sample_time * i); 
                
                double margin =sqrt(2); //安全距离
                //后轴约束
                VectorXd L;
				double S, d;
                Vector2d ref_pos(ref_states_para[4 * i], ref_states_para[4 * i + 1]);
				d2poly(ref_pos, poly, L, S, d);  // 计算到多边形的距离
                parameters.emplace_back(L(0));
                parameters.emplace_back(L(1));
                parameters.emplace_back(S - margin);
                //前轴约束
                 ref_pos  = Vector2d(ref_states_para_front[2 * i], ref_states_para_front[2 * i + 1]);
                d2poly(ref_pos, poly, L, S, d);  // 计算到多边形的距离
                parameters.emplace_back(L(0));
                parameters.emplace_back(L(1));
                parameters.emplace_back(S - margin);
            }
        }
        // cout << "paramstart: " << parameters[0] << "," << parameters[1] << "," << parameters[2] << "," << parameters[3] << endl;
        //求解参数设置
        m_args["lbx"] = lbx;
        m_args["ubx"] = ubx;
        m_args["x0"] = ref_states_para;
    // m_args["lbg"] = lbg;
        m_args["ubg"] = ubg;
        m_args["p"] = parameters;
        //求解
        m_res = m_solver(m_args);

        //获取优化变量
        std::vector<float> opt_states(m_res.at("x"));
        std::vector<float> diff;
        for (int i = 0; i < ref_states_para.size(); ++i) {
            diff.emplace_back(opt_states[i] - ref_states_para[i]);
        }
        // cout << "diff: " << calculateEuclideanNorm(diff) << endl;
        if(calculateEuclideanNorm(diff) < 0.1) {
            break;
        }
        ref_states_para = opt_states;
        // cout << "opt_start_state:" << ref_states_para[0] << "," << ref_states_para[1] << "," << ref_states_para[2] << "," << ref_states_para[3] << endl;
        // cout << "loop_count: " << loop_count << endl;
        ++loop_count;
    }
    best_result = ref_states_para;
}

Eigen::Matrix<float,2,1> NMPC1::get_controls()
{

    return m_control_command;
}

std::vector<Eigen::Matrix<float,3,1>> NMPC1::get_predict_trajectory()
{
    return m_predict_trajectory;
}

void NMPC1::d2poly(const Vector2d& point, const MatrixXd& poly, VectorXd& L, double& S, double& d) {
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