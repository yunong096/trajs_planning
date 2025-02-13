#include "nmpc2.hpp"

Mpc::Mpc(int count) {
    
    frame_count = count;
    N_ = 60;
    dt_ = 0.1;
    acc_max_ = 1.5;
    v_max_ = 15 / 3.6;
    delta_max_ = 0.52;
    vector<double> weights = {1,1,1,0,2,2,10,10}; //Q,R,S
    acc_min_ = - acc_max_;
    delta_min_ = - delta_max_;
    
    Q_ = DM::zeros(4,4); //索引之前初始化size
    R_ = DM::zeros(2,2);
    S_ = DM::zeros(2,2);
    

    setWeights(weights);
    kinematic_equation_ = setKinematicEquation();
}

void Mpc::setWeights(vector<double> weights) {
    //cout << "setweights" << endl;
    Q_(0, 0) = weights[0];
    Q_(1, 1) = weights[1];
    Q_(2, 2) = weights[2];
    Q_(3, 3) = weights[3];

    R_(0, 0) = weights[4];
    R_(1, 1) = weights[5];

    S_(0, 0) = weights[6];
    S_(1, 1) = weights[7];
}

Function Mpc::setKinematicEquation() {
    //cout << "set kinematic" << endl;
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX v = MX::sym("v");
    MX theta = MX::sym("theta");
    MX state_vars = MX::vertcat({x, y, v, theta});  //将状态变量组合成一个垂直向量（列向量）。
    MX acc = MX::sym("acc");
    MX delta = MX::sym("delta");
    MX control_vars = MX::vertcat({acc,  delta});  //控制变量control_vars是一个包含两个元素的垂直向量，分别是v（线速度）和w（角速度）
    
    //rhs means right hand side
    MX rhs = MX::vertcat({v * MX::cos(theta), v * MX::sin(theta), acc, v * tan(delta)/2.7}); // 动力学方程
    return Function("kinematic_equation", {state_vars, control_vars}, {rhs});
}


bool Mpc::solve(CartesianState& current_state,
                                                        std::vector<GlobalPathPoint>& ori_states,
                                                        std::vector<CartesianState>& ref_states_,
                                                        Obstacles& obs) {
                                                            //设置ref_states_para
    int max_iter = 10;
    std::vector<CartesianState> ref_states = ref_states_;
    
    while(max_iter--) {
        Opti opti = Opti();

        Slice all;

        DM  initX= DM::zeros(4, N_ + 1);
        DM  initU= DM::zeros(2, N_);
        DM  initS= DM::zeros(2, N_ + 1);
        for(int i = 0; i < N_ + 1; ++i) {
            initX(0, i) = ref_states[i].x;
            initX(1, i) = ref_states[i].y;
            initX(2, i) = ref_states[i].speed;
            initX(3, i) = ref_states[i].theta;
            // cout << "init_x: " <<  ref_states[i].x << ", " << ref_states[i].y << ", " << ref_states[i].speed << ", " << ref_states[i].theta << endl;
            if(i < N_) {
                initU(0, i) = ref_states[i].acc;
                initU(1, i) = std::atan2(2.7 * ref_states[i].kappa, 1);
                // cout << "init_control: " <<  ref_states[i].acc << ", " << std::atan2(2.7 * ref_states[i].kappa, 1) << endl;
            }
        }

        MX cost = 0;
        X = opti.variable(4, N_ + 1);// 定义一个3行N_+1列的矩阵变量X 注意：是变量。
        S = opti.variable(2, N_ + 1);
        //第一个时间步（索引为0）表示当前时刻的状态 后面N_个时间步表示未来N_个时刻的状态和U对应
        U = opti.variable(2, N_);

        // 设置初始值
        opti.set_initial(X, initX);
        opti.set_initial(U, initU);
        opti.set_initial(S, initS);

        MX x = X(0, all);
        MX y = X(1, all);
        MX v = X(2, all);
        MX theta = X(3, all);
        MX s1 = S(0, all);
        MX s2 = S(1, all);
        MX acc = U(0, all);
        MX delta = U(1, all);

        /*考虑在问题构建中加入带有最小化运动时间的变步长系数，因为DF的求解中考虑了这一点*/

        
        DM X_cur = DM::zeros(4);
        X_cur(0) = current_state.x;
        X_cur(1) = current_state.y;
        X_cur(2) = current_state.speed;
        X_cur(3) = current_state.theta;
        // cout << "set current state success" << endl;

        DM X_ref =DM::zeros(4, N_ + 1);   // 定义一个4行N_+1列的参数矩阵X_ref 注意：是已知的常量
        X_ref(0, 0) = current_state.x;
        X_ref(1, 0) = current_state.y;
        X_ref(2, 0) = current_state.speed;
        X_ref(3, 0) = current_state.theta;
        for (int i = 1; i < N_ + 1; ++i) {
            X_ref(0, i) = ori_states[i].x;
            X_ref(1, i) = ori_states[i].y;
            X_ref(2, i) = 10 / 3.6;
            X_ref(3, i) = ori_states[i].theta;
        }
        // cout << "set ref state success" << endl;


        //set costfunction
        for (int i = 0; i < N_; ++i) {
            MX X_err = X(all, i) - X_ref(all, i); 
            MX U_0 = U(all, i);
            MX S_0 = S(all, i);
            cost += MX::mtimes({X_err.T(), Q_, X_err}); //目标函数是状态误差和控制输入的成本之和
            cost += MX::mtimes({S_0.T(), S_, S_0});
            cost += MX::mtimes({U_0.T(), R_, U_0});
        }
        //MX::mtimes 用于执行矩阵乘法，用于计算两个矩阵的乘积
        cost += MX::mtimes({(X(all, N_) - X_ref(all, N_)).T(), Q_,
                            X(all, N_) - X_ref(all, N_)})   +   MX::mtimes({S(all, N_).T(), S_, S(all, N_)});  //终端项
        opti.minimize(cost); //opti.minimize 用于定义优化问题的目标函数
        // cout << "set cost success" << endl;

        //kinematic constrains opti.subject_to 用于添加约束条件到优化问题中
        for (int i = 0; i < N_; ++i) {
            vector<MX> input(2);
            input[0] = X(all, i);
            input[1] = U(all, i);
            MX X_next = kinematic_equation_(input)[0] * dt_ + X(all, i); 
            //这里体现了模型预测控制模型。  下一步的状态=通过模型计算的速度* dt_ +当前状态
            opti.subject_to(X_next == X(all, i + 1));  // 动力学约束
        }
        // cout << "set kinematic constrains success" << endl;

        //init value 初始化，初始状态赋值第一列所有行
        opti.subject_to(X(all, 0) == X_cur);//第一个时间步（索引为0）表示当前时刻的状态
        //speed angle_speed limit 控制输入的约束
        opti.subject_to(0 <= x <= 140.0);
        opti.subject_to(0 <= y <= 80.0);
        opti.subject_to(0 <= v <= v_max_);
        opti.subject_to( -1 < s1 <= 0);
        opti.subject_to( -1 < s2 <= 0);
        opti.subject_to(acc_min_ <= acc <= acc_max_);
        opti.subject_to(delta_min_ <= delta <= delta_max_);
        // cout << "set lu constrains success" << endl;

        //set obstacle constraints
        int margin = sqrt(2) + 0.5;
        std::vector<float> ref_states_para_front = FrontPos(ref_states);
        for (int i = 0; i < N_ + 1; ++i)
        {
            // for j = 0; j < obs_num; ++j
            for(auto j : obs.getMoveInds()) { 
                //获取障碍物位置
                //测试用，只有一个动态障碍
                if(current_state.x > 60 && j == 9) continue;
                if(current_state.y < 65 && j == 10) continue;
                auto cur_obs = obs.getObs()[j];
                MatrixXd poly(2, cur_obs.vertex_x.size());
                for (int k = 0; k < cur_obs.vertex_x.size(); ++k) {
                    poly(0, k) = cur_obs.vertex_x[k];
                    poly(1, k) = cur_obs.vertex_y[k];
                }
                poly += cur_obs.speed * MatrixXd::Ones(1, 4) * (0.5 *  frame_count + dt_ * i); 
                
                double margin =sqrt(2); //安全距离
                //后轴约束
                VectorXd alpha;
                double beta, d;
                Vector2d ref_pos(ref_states[i].x, ref_states[i].y);
                d2poly(ref_pos, poly,alpha, beta, d);  // 计算到多边形的距离
                opti.subject_to(alpha(0) * X(0, i) + alpha(1) * X(1, i) + S(0, i) <= beta - margin);
                // opti.subject_to(alpha(0) * X(0, i) + alpha(1) * X(1, i)  <= beta - margin);
                // cout << "alpha1:" << alpha(0) << "," << alpha(1) << "beta:" << beta << "d:" << d << endl;

                // //前轴约束
                ref_pos  = Vector2d(ref_states_para_front[2 * i], ref_states_para_front[2 * i + 1]);
                // cout << "ref_pos:" << ref_pos(0) << "," << ref_pos(1) << endl;
                d2poly(ref_pos, poly, alpha, beta, d); // 计算到多边形的距离

                //非线性的前轴约束
                // opti.subject_to(alpha(0) *( X(0, i) + 2.7 * cos(X(3, i))) + alpha(1) * (X(1, i) + 2.7 * sin(X(3, i)))+ S(1, i) <= beta - margin);
                // opti.subject_to(alpha(0) *( X(0, i) + 2.7 * cos(X(3, i))) + alpha(1) * (X(1, i) + 2.7 * sin(X(3, i))) <= beta - margin);
                // cout << "alpha2:" << alpha(0) << ", " << alpha(1) << ", beta:" << beta << ", d:" << d << endl;
                
                //线性前轴约束
                if(i < N_) {
                    // MX NEW_X = {X(0, i), X(1, i), X(0, i + 1), X(1, i + 1)};
                    MatrixXd A(2, 4);
                    A << 1, 0, 0, 0,
                             0, 1, 0, 0; 
                    MatrixXd B(2, 4);
                    B << 0, 0, 1, 0,
                             0, 0, 0, 1; 
                    Vector4d ref_X (ref_states[i].x, ref_states[i].y, 
                                                     ref_states[i+1].x, ref_states[i+1].y);
                    if(ref_states[i].x == ref_states[i+1].x && ref_states[i].y == ref_states[i+1].y) {
                        // ref_X(2) = ref_states[i].x + std::numeric_limits<double>::epsilon() * cos(ref_states[i].theta);
                        // ref_X(3) = ref_states[i].y + std::numeric_limits<double>::epsilon() * sin(ref_states[i].theta);
                        ref_X =  Vector4d(ref_states[i].x, ref_states[i].y, 
                                                     ref_states[i].x + 0.1 * cos(ref_states[i].theta), 
                                                     ref_states[i].y + 0.1 * sin(ref_states[i].theta));
                    }

                    // cout << "ref_states:" << ref_states[i].x << "," << ref_states[i].y  << ", " << ref_states[i+1].x << ", " << ref_states[i+1].y<< endl;
					// 更新约束
					MatrixXd C = B - A;
					VectorXd X0 = C *  ref_X;
					VectorXd fX0 = X0 / X0.norm();
					MatrixXd dfX0 = C / X0.norm() - X0 * X0.transpose() * C / pow(X0.norm(), 3);
					MatrixXd L_f = alpha.transpose() * (A + 2.7 * dfX0);
					double S_f = beta - margin + alpha.transpose() * 2.7 * (dfX0 * ref_X - fX0);
                    // cout << "sf: " << S_f << endl;
                    // cout << "lf0: " << L_f(0) << ", lf1: " << L_f(1) << ", lf2: " << L_f(2) << ", lf3: " << L_f(3) << endl;
                    opti.subject_to(L_f(0) * X(0, i) + L_f(1) * X(1, i) + L_f(2) * X(0, i + 1) + L_f(3) * X(1, i + 1)  + S(1, i) <= S_f);
                }

            }
        }
        // cout << "set obstacle constrains success" << endl;

        //set solver
        casadi::Dict solver_opts; // 设置求解器选项
        solver_opts["expand"] = true; //MX change to SX for speed up
        solver_opts["ipopt.max_iter"] = 1000;
        solver_opts["ipopt.print_level"] = 0;
        solver_opts["print_time"] = 0;
        solver_opts["ipopt.acceptable_tol"] = 1e-6;
        solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6;

        opti.solver("ipopt", solver_opts);

        auto start_time = std::chrono::high_resolution_clock::now();

        solution_ = std::make_unique<casadi::OptiSol>(opti.solve());

        auto end_time = std::chrono::high_resolution_clock::now();
        // 计算时间间隔 
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        // 输出时间间隔
        ROS_WARN( "a NMPC problem solve time: %f s" ,  elapsed_seconds.count() );

        opt_path = getSoluPath();
        double diff = 0;
        for (int i = 0; i < N_ + 1; ++i) {
            diff += pow(opt_path[i].x - ref_states[i].x, 2) + pow(opt_path[i].y - ref_states[i].y, 2);
        }
        cout << "diff: " << sqrt(diff) << endl;
        if(sqrt(diff) < 1.0) {
            return true;
        }
        ref_states = opt_path;
    }
    return false;
}

// vector<double> Mpc::getFirstU() {
//     vector<double> res;
//     auto first_v =  solution_->value(U)(0, 0);
//     auto first_w = solution_->value(U)(1, 0);
    
//     //cout << "first_u" << first_u << " " << "first_v" << first_v << endl;

//     res.push_back(static_cast<double>(first_v));
//     res.push_back(static_cast<double>(first_w));
//     return res;
// }

// vector<double> Mpc::getPredictX() {
//     vector<double> res;
//     auto predict_x = solution_->value(X);
//     cout << "nomal" << endl;
//     //cout << "predict_x size :" << predict_x.size() << endl;
//     for (int i = 0; i <= N_; ++i) {
//         res.push_back(static_cast<double>(predict_x(0, i)));
//         res.push_back(static_cast<double>(predict_x(1, i)));
//     }
//     return res;
// }



