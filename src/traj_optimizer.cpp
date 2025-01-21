#include "df_planner/traj_optimizer.h"

#include "df_planner/constants.h"
#include "df_planner/lbfgs.h"

namespace plan_manage {
/* main planning API */
bool TrajOptimizer::OptimizeTrajectory(const traj_utils::FlatTrajData &traj,
      GlobalPath& refline,
      const vector<GlobalPathPoint>& ori_path, 
      const vector<CartesianState>& last_path, 
      const std::vector<std::vector<Eigen::Vector3d>> &rb_hPoly_container,
      const std::vector<std::vector<Eigen::Vector3d>> &fb_hPoly_container) {
  // 初始化
  refline_ = refline; 
  last_path_ = last_path;
  ori_path_ = ori_path;//用于计算参考线距离
  iniState_container = traj.start_state;
  finState_container = traj.final_state;
  rb_sfc_container = rb_hPoly_container;
  fb_sfc_container = fb_hPoly_container;
  // refline_container = reference_line;
  double final_cost;  // final cost
  const auto &init_inner_pts = traj.inner_pts;
  jerkOpt.reset(piece_);

  // ROS_WARN("DF planner Start Put Init X");
  Eigen::VectorXd x(2 * (piece_ - 1) + 1);  // 2 * x,y + time
  for (int i = 0; i < piece_ - 1; ++i) {
    x(2 * i) = init_inner_pts[i][0]; 
    x(2 * i + 1) = init_inner_pts[i][1];
  }
  x(2 * (piece_ - 1)) = RealT2VirtualT(traj.duration);

  bool flag_success = false;              // 返回值
  cost_vec.resize(piece_, 7);             // todo，变量赋值
  lbfgs::lbfgs_parameter_t lbfgs_params;  // 需要设置lbfgs参数，用config文件
  lbfgs_params.mem_size = 384;
  lbfgs_params.past = 3;
  lbfgs_params.g_epsilon = 1.0e-4;
  lbfgs_params.delta = 1e-3;  // 改了下
  lbfgs_params.min_step = 1.0e-32;
  lbfgs_params.max_iterations =100;
  // wei_time_ = df_param.time_scale();
  // wei_feas_ = df_param.feas_scale();
  // wei_obs_ = df_param.obs_scale();
  // wei_lb_ = df_param.lb_scale();
  // wei_cur_ = df_param.cur_scale();
  // wei_jerk_ = df_param.jerk_scale();
  // wei_acc_ = df_param.acc_scale();
  // wei_ref_ = df_param.ref_scale();
  int result =
      lbfgs::lbfgs_optimize(x, final_cost, TrajOptimizer::costFunctionCallback,
                            this, lbfgs_params, is_debug_mode_);

  /* ---------- get result and check collision ---------- */
  if (result == lbfgs::LBFGS_CONVERGENCE || result == lbfgs::LBFGS_CANCELED ||
      result == lbfgs::LBFGS_STOP ||
      result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
      result == lbfgs::LBFGSERR_MAXIMUMLINESEARCH) {
    flag_success = true;
  }
  if (final_cost >= 50000.0) {
    flag_success = false;
  }
  final_cost_ = final_cost;
  return flag_success;
}

void TrajOptimizer::costFunctionCallback(void *func_data,
                                         const Eigen::VectorXd &x,
                                         Eigen::VectorXd &grad,
                                         Eigen::MatrixXd *cost) {
  TrajOptimizer *opt = reinterpret_cast<TrajOptimizer *>(func_data);
  // opt是PolyTrajOptimizer类型
  cost->resize(opt->piece_, 7);
  cost->setZero();
  // P_container resize 索引赋值
  // piece_num_container是每段seg的piece数

  // 找到了，这个是grad的映射，给他赋值就是给grad赋值
  const int t_index = 2 * (opt->piece_ - 1);
  const double t = x(t_index);
  const double T = opt->VirtualT2RealT(t);  // T是每一段seg段的时间长度，

  const auto &IniS = opt->iniState_container;
  const auto &FinS = opt->finState_container;  // 直接干，不用管前进倒车的状态
  Eigen::Map<const Eigen::MatrixXd> P(x.data(), 2, opt->piece_ - 1);
  opt->jerkOpt.generate(P, T / opt->piece_, IniS, FinS);

  // ROS_WARN("DF planner Start Calc Jerk  Cost");
  const auto &jerk_vec = opt->jerkOpt.getTrajJerkCost();  // jerk_cost
  for (int j = 0; j < opt->piece_; j++) {
    (*cost)(j, 0) += opt->wei_jerk_ * jerk_vec[j];
  }
  opt->jerkOpt.initSmGradCost(opt->wei_jerk_, opt->wei_acc_);
  // 静态障碍物，动态障碍物，可行性约束，参考线约束, 曲率约束
  static Eigen::MatrixXd sampling_costs(opt->piece_, 5);
  sampling_costs.setZero();                // todo
  opt->addPVAGradCost2CT(sampling_costs);  // Time int
  // cost，这里求解了静态障碍物和动态障碍物约束的cost，以及速度加速度曲率的约束的cost
  for (int j = 0; j < sampling_costs.rows(); j++) {
    (*cost)(j, 3) += sampling_costs(j, 0);
    (*cost)(j, 4) += sampling_costs(j, 2);
    (*cost)(j, 5) += sampling_costs(j, 3);
    (*cost)(j, 6) += sampling_costs(j, 4);
  }
  // ROS_WARN("DF planner Start Calc Time  Cost");
  double time_cost = 0.0;
  opt->jerkOpt.calGrads_PT();  // gdt gdp gdhead gdtail
  // waypoint
  Eigen::Map<Eigen::MatrixXd> gradP(grad.data(), 2, opt->piece_ - 1);
  gradP = opt->jerkOpt.get_gdP();
  double *grad_t = grad.data() + t_index;
  opt->VirtualTGradCost(T, t, opt->jerkOpt.get_gdT() / opt->piece_, grad_t,
                        &time_cost);
  for (int j = 0; j < opt->piece_; j++) {
    (*cost)(j, 2) += time_cost / opt->piece_;
  }
  // ROS_WARN("DF planner Finish Calc g-matrix");
  opt->iter_num_ += 1;
}

void TrajOptimizer::VirtualTGradCost(const double RT, const double VT,
                                     const double gdRT, double *gdVT,
                                     double *costT) {
  double gdVT2Rt;
  if (VT > 0) {
    gdVT2Rt = VT + 1.0;
  } else {
    double denSqrt = (0.5 * VT - 1.0) * VT + 1.0;
    gdVT2Rt = (1.0 - VT) / (denSqrt * denSqrt);
  }

  *gdVT = (gdRT + wei_time_) * gdVT2Rt;
  *costT = RT * wei_time_;  // 就是time的权重乘以总时间
}

/* mappings between real world time and unconstrained virtual time */
double TrajOptimizer::RealT2VirtualT(const double RT) {
  return RT > 1.0 + mini_T ? (sqrt(2.0 * RT - 1.0 - 2 * mini_T) - 1.0)
                           : (1.0 - sqrt(2.0 / (RT - mini_T) - 1.0));
}

double TrajOptimizer::VirtualT2RealT(const double VT)  // VT相对时间，RT绝对时间
{
  return VT > 0.0 ? ((0.5 * VT + 1.0) * VT + 1.0) + mini_T
                  : 1.0 / ((0.5 * VT - 1.0) * VT + 1.0) + mini_T;
}

void TrajOptimizer::addPVAGradCost2CT(Eigen::MatrixXd &costs) {
  // 静态障碍物，动态障碍物，可行性约束，参考线约束,曲率约束
  // output gradT gradC
  // 没用变量都删掉
  Eigen::Vector2d sigma;
  Eigen::Vector2d dsigma;
  Eigen::Vector2d ddsigma;
  Eigen::Vector2d dddsigma;
  Eigen::Vector2d ddddsigma;
  Eigen::Matrix<double, 6, 1> beta0 = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> beta1 = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> beta2 = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> beta3 = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 1> beta4 = Eigen::Matrix<double, 6, 1>::Zero();
  Eigen::Matrix<double, 6, 2> gradViolaPc;
  Eigen::Matrix<double, 6, 2> d_vel_d_c;
  Eigen::Matrix<double, 6, 2> gradViolaAc;
  Eigen::Matrix<double, 6, 2> gradViolaKc;
  Eigen::Matrix<double, 6, 2> gradViolaRefc;  // 约束对c导数
  Eigen::Matrix<double, 6, 2> gradViolaConsistc;  // 约束对c导数

  const auto &rb_sfc = rb_sfc_container;
  const auto &fb_sfc = fb_sfc_container;
  Eigen::Matrix2d ego_R;
  Eigen::Matrix2d help_R;
  int pointid = -1;

  for (int i = 0; i < piece_; ++i) {
    const int K = corridor_per_piece;
    const auto &c = jerkOpt.getCoeffs().block<6, 2>(i * 6, 0);
	//提取一个6行2列的子矩阵，起始行索引为6*i，起始列索引为0，即每段的五次多项式的六个系数，x和y各对应一列
    const double step = jerkOpt.getDt() / K;  // T_i /k
    double s1 = 0.0;
    Eigen::VectorXd piece_cost = Eigen::VectorXd::Zero(5);
    for (int j = 0; j < K; ++j) {
      const double s2 = s1 * s1;
      const double s3 = s2 * s1;
      const double s4 = s2 * s2;
      const double s5 = s4 * s1;

	  //x和y是t的五次多项式，c_trans*beta代表各阶导数
      beta0 << 1.0, s1, s2, s3, s4, s5;
      beta1(1, 0) = 1.0;
      beta1(2, 0) = 2.0 * s1;
      beta1(3, 0) = 3.0 * s2;
      beta1(4, 0) = 4.0 * s3;
      beta1(5, 0) = 5.0 * s4;
      beta2(2, 0) = 2.0;
      beta2(3, 0) = 6.0 * s1;
      beta2(4, 0) = 12.0 * s2;
      beta2(5, 0) = 20.0 * s3;
      beta3(3, 0) = 6.0;
      beta3(4, 0) = 24 * s1;
      beta3(5, 0) = 60 * s2;
      beta4(4, 0) = 24.0;
      beta4(5, 0) = 120 * s1;
      const double alpha = 1.0 / K * j;
      s1 += step;  // 时间按分辨率向前移动
      pointid++;
      // 初始都为0
      // update s1 for the next iteration
      const auto &c_trans = c.transpose();
      sigma = c_trans * beta0;
      dsigma = c_trans * beta1;
      ddsigma = c_trans * beta2;
      dddsigma = c_trans * beta3;
      ddddsigma = c_trans * beta4;

      double z_h0 = dsigma.norm();  // 速度
      const double z_h1 = ddsigma.transpose() * dsigma;
      const double z_h2 = dddsigma.transpose() * dsigma;
      const double z_h3 = ddsigma.transpose() * B_h * dsigma;
      if ((z_h0 < 1e-4) || (j == 0 && i == 0) || (i == piece_ - 1 && j == K)) {
        continue;
      }

      const double vel2_reci = 1.0 / (z_h0 * z_h0);  // 1除以sigma的平方
      const double vel2_reci_e = 1.0 / (z_h0 * z_h0 + epis);
      const double vel3_2_reci_e = vel2_reci_e * sqrt(vel2_reci_e);
      // 1除以sigma的六次方
      z_h0 = 1.0 / z_h0;

      const double z_h4 = z_h1 * vel2_reci;
      const double acc2 = z_h1 * z_h1 * vel2_reci;
      const double cur = z_h3 * vel3_2_reci_e;

      ego_R << dsigma(0), -dsigma(1), dsigma(1), dsigma(0);
      ego_R = ego_R * z_h0;

      Eigen::Matrix2d temp_a;
      Eigen::Matrix2d temp_v;
      temp_a << ddsigma(0), -ddsigma(1), ddsigma(1), ddsigma(0);
      temp_v << dsigma(0), -dsigma(1), dsigma(1), dsigma(0);
      Eigen::Matrix2d R_dot =
          (temp_a * z_h0 - temp_v * vel2_reci * z_h0 * z_h1);
      // 再外面一层是离散的自车轨迹点K个
      // vec_le在初始化的时候已经更新过了，就是自车的四个顶点
      //这里换成cfs，不再使用自车的四个顶点，而使用前后轴中心
      // for (const auto &le : vec_le_) {

        // pointid是约束点的index
        // 这里的cost（0）是做了与静态障碍物走廊位置上的cost
        // ROS_WARN("DF planner Start Calc Obs Cost");
        Eigen::Vector2d le;
        le << 0.0, 0.0;
        Eigen::Vector2d bpt = sigma + ego_R * le ;  //
        Eigen::Matrix2d temp_l_Bl;
        temp_l_Bl << le(0), -le(1), le(1), le(0);

        Eigen::Vector2d le_f = vec_front_;
        Eigen::Vector2d bpt_f = sigma + ego_R * le_f ;  //
        Eigen::Matrix2d temp_l_Bl_f;
        temp_l_Bl_f << le_f(0), -le_f(1), le_f(1), le_f(0);
        for (int k = 0; k < rb_sfc[pointid].size(); k++) {
          //后轴约束
          const auto &rb_outerNormal = rb_sfc[pointid][k].head<2>();
          double d_pos = rb_outerNormal.dot(bpt) - rb_sfc[pointid][k](2);

          // 位置上避障的函数
          if (d_pos > 0.0) {
            double violaPosPena = 0.0;
            double violaPosPenaD = 0.0;
            positiveSmoothedL1(d_pos, &violaPosPena, &violaPosPenaD);
            // 根据volaPos计算两个penalty

            gradViolaPc = beta0 * rb_outerNormal.transpose() +
                          beta1 * rb_outerNormal.transpose() *
                              (temp_l_Bl * z_h0 -
                               ego_R * le * dsigma.transpose() * vel2_reci);

            const double gradViolaPt =
                alpha * rb_outerNormal.transpose() * (dsigma + R_dot * le);

            jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
                step * wei_obs_ * violaPosPenaD * gradViolaPc;
            jerkOpt.get_gdT() +=
                wei_obs_ *
                (violaPosPenaD * gradViolaPt * step + violaPosPena / K);

            piece_cost(0) += step * wei_obs_ * violaPosPena;
          }

          //前轴约束
          const auto &fb_outerNormal = fb_sfc[pointid][k].head<2>();
          d_pos = fb_outerNormal.dot(bpt_f) - fb_sfc[pointid][k](2);

          // 位置上避障的函数
          if (d_pos > 0.0) {
            double violaPosPena = 0.0;
            double violaPosPenaD = 0.0;
            positiveSmoothedL1(d_pos, &violaPosPena, &violaPosPenaD);
            // 根据volaPos计算两个penalty

            gradViolaPc = beta0 * fb_outerNormal.transpose() +
                          beta1 * fb_outerNormal.transpose() *
                              (temp_l_Bl_f * z_h0 -
                               ego_R * le * dsigma.transpose() * vel2_reci);

            const double gradViolaPt =
                alpha * fb_outerNormal.transpose() * (dsigma + R_dot * le);

            jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
                step * wei_obs_ * violaPosPenaD * gradViolaPc;
            jerkOpt.get_gdT() +=wei_obs_ * (violaPosPenaD * gradViolaPt * step +
                                            violaPosPena / K);

            piece_cost(0) += step * wei_obs_ * violaPosPena;
            // piece_cost(3) += step * wei_lb_ * violaPosPena;

          }
        }
      // }
      double violaRefPena = 0.0;
      double violaRefPenaD = 0.0;
      // 加入参考线中心代价
      // ROS_WARN("DF planner Start Calc Refline Cost");
      Eigen::Vector2d dist_to_ref = UpdateRefIndex(sigma, pointid);
      // Eigen::Vector2d dist_to_ref = UpdateRefIndex(sigma);
      const double violaRef = dist_to_ref(0, 0) * dist_to_ref(0, 0) +
                              dist_to_ref(1, 0) * dist_to_ref(1, 0);
      if (violaRef > 0.0) {
        positiveSmoothedL1(violaRef, &violaRefPena, &violaRefPenaD);
        gradViolaRefc = 2.0 * beta0 * dist_to_ref.transpose();
        const double gradViolaReft =
            2.0 * alpha * dsigma.transpose() * dist_to_ref;
        jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
            step * wei_ref_ * violaRefPenaD * gradViolaRefc;
        jerkOpt.get_gdT() += wei_ref_ * (violaRefPenaD * gradViolaReft * step +
                                         violaRefPena / K);
        piece_cost(3) += step * wei_ref_ * violaRefPena;
      }

      double violaConsistPena = 0.0;
      double violaConsistPenaD = 0.0;
      // 加入连续代价
      Eigen::Vector2d dist_to_Consist = UpdateConsistIndex(sigma, pointid);
      const double violaConsist = dist_to_Consist(0, 0) * dist_to_Consist(0, 0) +
                              dist_to_Consist(1, 0) * dist_to_Consist(1, 0);
      if (violaConsist > 0.0) {
        positiveSmoothedL1(violaConsist, &violaConsistPena, &violaConsistPenaD);
        gradViolaConsistc = 2.0 * beta0 * dist_to_Consist.transpose();
        const double gradViolaConsistt =
            2.0 * alpha * dsigma.transpose() * dist_to_Consist;
        jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
            step * wei_consist_ * violaConsistPenaD * gradViolaConsistc;
        jerkOpt.get_gdT() += wei_consist_ * (violaConsistPenaD * gradViolaConsistt * step +
                                         violaConsistPena / K);
        piece_cost(3) += step * wei_consist_ * violaConsistPena;
      }

      // ROS_WARN("DF planner Start Calc Speed Cost");
      const double d_vel =
          1.0 / vel2_reci - max_forward_vel_ * max_forward_vel_;
      if (d_vel > 0.0) {
        double violaVelPena = 0.0;
        double violaVelPenaD = 0.0;
        positiveSmoothedL1(d_vel, &violaVelPena, &violaVelPenaD);
        // 速度的代价

        d_vel_d_c = 2.0 * beta1 * dsigma.transpose();  // 6*2
        const double d_vel_d_t = 2.0 * alpha * z_h1;  // 1*1,这里为什么加alpha？
        jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
            step * wei_feas_ * violaVelPenaD * d_vel_d_c;
        jerkOpt.get_gdT() +=
            wei_feas_ * (violaVelPenaD * d_vel_d_t * step + violaVelPena / K);
        piece_cost(2) += step * wei_feas_ * violaVelPena;  // 速度
      }
      
      // 加速度的代价
      // ROS_WARN("DF planner Start Calc Acc Cost");
      const double violaAcc = acc2 - max_forward_acc * max_forward_acc;
      if (violaAcc > 0.0) {
        double violaAccPena = 0.0;
        double violaAccPenaD = 0.0;
        positiveSmoothedL1(violaAcc, &violaAccPena, &violaAccPenaD);
        gradViolaAc = 2.0 * beta1 * (z_h4 * ddsigma.transpose() -
                                     z_h4 * z_h4 * dsigma.transpose()) +
                      2.0 * beta2 * z_h4 * dsigma.transpose();  // 6*2
        const double gradViolaAt =
            2.0 * alpha *
            (z_h4 * (ddsigma.squaredNorm() + z_h2) - z_h4 * z_h4 * z_h1);
        jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
            step * wei_feas_ * violaAccPenaD * gradViolaAc;
        jerkOpt.get_gdT() +=
            wei_feas_ * (violaAccPenaD * gradViolaAt * step + violaAccPena / K);
        piece_cost(2) += step * wei_feas_ * violaAccPena;  // 加速度
      }

      const double curv_dir = cur > 0.0 ? 1.0 : -1.0;
      double violaCurvPena = 0.0;
      double violaCurvPenaD = 0.0;
      positiveSmoothedL1(std::fabs(cur), &violaCurvPena, &violaCurvPenaD);
      gradViolaKc =
          curv_dir *
          (beta1 *
               (vel3_2_reci_e * ddsigma.transpose() * B_h -
                3 * vel3_2_reci_e * vel2_reci_e * z_h3 * dsigma.transpose()) +
           beta2 * vel3_2_reci_e * dsigma.transpose() * B_h.transpose());
      const double gradViolaKt =
          curv_dir * alpha * vel3_2_reci_e *
          (dddsigma.transpose() * B_h * dsigma - 3 * vel2_reci_e * z_h3 * z_h1);
      jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
          step * wei_cur_ * 10.0 * violaCurvPenaD * gradViolaKc;
      jerkOpt.get_gdT() +=
          wei_cur_ * 10.0 *
          (violaCurvPenaD * gradViolaKt * step + violaCurvPena / K);
      piece_cost(4) += step * wei_cur_ * 10.0 * violaCurvPena;
      const double d_curv = std::fabs(cur) - max_forward_cur;
      if (d_curv > 0.0) {
        double violaDCurvPena = 0.0;
        double violaDCurvPenaD = 0.0;
        positiveSmoothedL1(d_curv, &violaDCurvPena, &violaDCurvPenaD);
        jerkOpt.get_gdC().block<6, 2>(i * 6, 0) +=
            step * wei_cur_ * 10.0 * violaDCurvPenaD * gradViolaKc * 20.0;
        jerkOpt.get_gdT() +=
            wei_cur_ * 10.0 *
            (violaDCurvPenaD * gradViolaKt * step + violaDCurvPena / K) * 20.0;
        piece_cost(4) += step * wei_cur_ * 10.0 * violaDCurvPena * 20.0;
      }
    }
    // 在这里把结果放入cost
    for (int j = 0; j < 5; j++) {
      costs(i, j) += piece_cost(j);
    }
  }
}

void TrajOptimizer::positiveSmoothedL1(const double x, double *f, double *df) {
  const double pe = 1.0e-4;
  const double half = 0.5 * pe;
  const double f3c = 1.0 / (pe * pe);
  const double f4c = -0.5 * f3c / pe;
  const double d2c = 3.0 * f3c;
  const double d3c = 4.0 * f4c;

  if (x < pe) {
    *f = (f4c * x + f3c) * x * x * x;
    *df = (d3c * x + d2c) * x * x;
  } else {
    *f = x - half;
    *df = 1.0;
  }
}

/* helper functions */
void TrajOptimizer::setParam() {
  B_h << 0, -1, 1, 0;
  // Eigen::Vector2d le_1;
  // Eigen::Vector2d le_2;
  // Eigen::Vector2d le_3;
  // Eigen::Vector2d le_4;
  // // vertexs of the ego car in the body frame
  // vec_le_.clear();
  // const double half_width = vehicle_param_.width() / 2.0;
  // const double half_length = vehicle_param_.length() / 2.0;
  // le_1 << vehicle_param_.d_cr() + half_length + ego_buffer,
  //     half_width + ego_buffer;
  // le_2 << vehicle_param_.d_cr() + half_length + ego_buffer,
  //     -half_width - ego_buffer;
  // le_3 << vehicle_param_.d_cr() - half_length - ego_buffer,
  //     -half_width - ego_buffer;
  // le_4 << vehicle_param_.d_cr() - half_length - ego_buffer,
  //     half_width + ego_buffer;

    vec_front_  << 2.7 , 0.0; //wheelbase

  // attention here! These vectors store one more of the vertexs! The vertexs
  // are stored Clockwise！
  // vec_le_.push_back(le_1);
  // vec_le_.push_back(le_2);
  // vec_le_.push_back(le_3);
  // vec_le_.push_back(le_4);
  // vec_le_.push_back(le_1);
}

vector<CartesianState> TrajOptimizer::GetResult(double gap) {
  // 优化结果密集采样输出
  vector<CartesianState> final_path;
  const auto &final_traj = jerkOpt.getTraj(1);
  // const double gap = 0.1;
  const double total_t = final_traj.getTotalDuration();
  const int duration_size = std::ceil(total_t / gap);
  int count = 0;
  for (double t = 0.0; t < total_t; t += gap, ++count) {
    CartesianState state;
    const auto pieceIdx = final_traj.locatePieceIdx(t);
    // cout << "relative_t: " << pieceIdx.first << ", piece_idx" << pieceIdx.second << endl;
    const auto &piece = final_traj[pieceIdx.first];
    const double relative_t = pieceIdx.second;
    piece.getState(relative_t, state);
    // state->set_t(t);
    final_path.emplace_back(state);
    // if(count < 5)
      // cout << "df_state: " << state.x << ", " << state.y << ", " << state.speed << ", " << state.theta << ", " << state.acc << ", " << state.kappa << endl;
  }
  // cout << "final_path_size: " <<  final_path.size() << endl;
  const auto &piece_positions = final_traj.getPositions();
  for (int j = 0; j < piece_positions.cols(); j++) {
    if (j == 0) {
      cout << "起点：" << piece_positions(0, j) << ", " << piece_positions(1, j) << endl;
    } else if (j == piece_positions.cols() - 1) {
      cout << "终点：" << piece_positions(0, j) << ", " << piece_positions(1, j) << endl;
    } else {
      cout << "中间点：" << piece_positions(0, j) << ", " << piece_positions(1, j) << endl;
    }
  }
  return final_path;
}

vector<CartesianState> TrajOptimizer::GetResult() {
  // 优化结果密集采样输出
  vector<CartesianState> final_path;
  const auto &final_traj = jerkOpt.getTraj(1);
  const double total_t = final_traj.getTotalDuration();
  double gap = total_t / (corridor_per_piece * piece_);
  final_path.emplace_back(CartesianState(0,0,0,0,0,0));
  for (double t = gap; t <= total_t - gap + 0.01; t += gap) {
    CartesianState state;
    const auto pieceIdx = final_traj.locatePieceIdx(t);
    const auto &piece = final_traj[pieceIdx.first];
    const double relative_t = pieceIdx.second;
    piece.getState(relative_t, state);
    // state->set_t(t);
    final_path.emplace_back(state);
  }
  final_path.emplace_back(CartesianState(0,0,0,0,0,0));
  cout << "cur_gap_t: " << gap << endl;
  cout << "final_path_size: " <<  final_path.size() << endl;
  return final_path;
}

double TrajOptimizer::getResultDuration() {
  const auto &final_traj = jerkOpt.getTraj(1);
  return final_traj.getTotalDuration();
}

Eigen::Vector2d TrajOptimizer::UpdateRefIndex(
  //用于计算与参考线的距离代价
    const Eigen::Vector2d &sigma) {  // 引用 + 返回pair
  //找最近点
    GlobalPathPoint ref_state = GlobalPathPoint{0, 0, 0, 0, 0, 0};
    double min_dist = std::numeric_limits<double>::infinity();
    int min_ind = 0;
    for(int i = 0; i < refline_.x.size(); ++i) {
        double dx = sigma(0) - refline_.x[i];
        double dy = sigma(1) - refline_.y[i];
        double dist = hypot(dx, dy);
        if (dist < min_dist) {
            min_dist = dist;
            min_ind = i;
        }
    }
    // cout << "min_ind: " << min_ind << " min_dist: " << min_dist << endl;
    if(min_ind > 0 && min_ind < refline_.x.size() - 1)
        ref_state = CoarsePathGenerator::FindNearestPt(refline_.x[min_ind-1], refline_.y[min_ind-1], refline_.all_s[min_ind-1], 
                                                                                                            refline_.theta[min_ind-1], refline_.kappa[min_ind-1], refline_.dkappa[min_ind-1], 
                                                                                                            refline_.x[min_ind], refline_.y[min_ind], refline_.all_s[min_ind], 
                                                                                                            refline_.theta[min_ind], refline_.kappa[min_ind], refline_.dkappa[min_ind], 
                                                                                                            refline_.theta[min_ind + 1], refline_.kappa[min_ind + 1], refline_.dkappa[min_ind + 1], 
                                                                                                            refline_.s[min_ind-1] ,refline_.s[min_ind], sigma(0), sigma(1));
    else if (min_ind == 0)
        ref_state = CoarsePathGenerator::FindNearestPt(refline_.x[min_ind], refline_.y[min_ind], refline_.all_s[min_ind], 
                                                                                                            refline_.theta[min_ind], refline_.kappa[min_ind], refline_.dkappa[min_ind], 
                                                                                                            refline_.theta[min_ind + 1], refline_.kappa[min_ind + 1], refline_.dkappa[min_ind + 1], 
                                                                                                            refline_.s[min_ind] , sigma(0), sigma(1));
    else if (min_ind == refline_.x.size() - 1)
        ref_state = CoarsePathGenerator::FindNearestPt(refline_.x[min_ind-1], refline_.y[min_ind-1], refline_.all_s[min_ind-1], 
                                                                                                            refline_.theta[min_ind-1], refline_.kappa[min_ind-1], refline_.dkappa[min_ind-1], 
                                                                                                            refline_.theta[min_ind], refline_.kappa[min_ind], refline_.dkappa[min_ind], 
                                                                                                            refline_.s[min_ind - 1] , sigma(0), sigma(1));
  return Eigen::Vector2d(ref_state.x - sigma(0), ref_state.y - sigma(1));
}

Eigen::Vector2d TrajOptimizer::UpdateRefIndex(const Eigen::Vector2d &sigma, int index) {  
  return Eigen::Vector2d(ori_path_[index].x - sigma(0), ori_path_[index].y - sigma(1));
}

Eigen::Vector2d TrajOptimizer::UpdateConsistIndex(const Eigen::Vector2d &sigma, int index) {  
  if (index + 5 < last_path_.size())
    return Eigen::Vector2d(last_path_[index + 5].x - sigma(0), last_path_[index+5].y - sigma(1));
  else
    return Eigen::Vector2d(0.0, 0.0);
}

}  // namespace plan_manage
