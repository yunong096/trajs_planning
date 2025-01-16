#pragma once

#include <eigen3/Eigen/Eigen>
#include <algorithm>
#include <cmath>

// #include "core/proto/hybrid_ara.pb.h"

namespace lbfgs {
// ----------------------- Data Type Part -----------------------

// cost容器
// vector<vector<double>> cost_container;

/**
 * L-BFGS optimization parameters.
 */
struct lbfgs_parameter_t {
  /**
   * The number of corrections to approximate the inverse hessian matrix.
   *  The L-BFGS routine stores the computation results of previous m
   *  iterations to approximate the inverse hessian matrix of the current
   *  iteration. This parameter controls the size of the limited memories
   *  (corrections). The default value is 8. Values less than 3 are
   *  not recommended. Large values will result in excessive computing time.
   */
  int mem_size = 8;

  /**
   * Epsilon for grad convergence test. DO NOT USE IT in nonsmooth cases!
   *  Set it to 0.0 and use past-delta-based test for nonsmooth functions.
   *  This parameter determines the accuracy with which the solution is to
   *  be found. A minimization terminates when
   *      ||g(x)||_inf / max(1, ||x||_inf) < g_epsilon,
   *  where ||.||_inf is the infinity norm. The default value is 1.0e-5.
   *  This should be greater than 1.0e-6 in practice because L-BFGS does
   *  not directly reduce first-order residual. It still needs the function
   *  value which can be corrupted by machine_prec when ||g|| is small.
   */
  double g_epsilon = 1.0e-5;

  /**
   * Distance for delta-based convergence test.
   *  This parameter determines the distance, in iterations, to compute
   *  the rate of decrease of the cost function. If the value of this
   *  parameter is zero, the library does not perform the delta-based
   *  convergence test. The default value is 3.
   */
  int past = 3;

  /**
   * Delta for convergence test.
   *  This parameter determines the minimum rate of decrease of the
   *  cost function. The library stops iterations when the following
   *  condition is met:
   *      |f' - f| / max(1, |f|) < delta,
   *  where f' is the cost value of past iterations ago, and f is
   *  the cost value of the current iteration.
   *  The default value is 1.0e-6.
   */
  double delta = 1.0e-6;

  /**
   * The maximum number of iterations.
   *  The lbfgs_optimize() function terminates an minimization process with
   *  ::LBFGSERR_MAXIMUMITERATION status code when the iteration count
   *  exceedes this parameter. Setting this parameter to zero continues an
   *  minimization process until a convergence or error. The default value
   *  is 0.
   */
  int max_iterations = 0;

  /**
   * The maximum number of trials for the line search.
   *  This parameter controls the number of function and gradients evaluations
   *  per iteration for the line search routine. The default value is 64.
   */
  int max_linesearch = 64;

  /**
   * The minimum step of the line search routine.
   *  The default value is 1.0e-20. This value need not be modified unless
   *  the exponents are too large for the machine being used, or unless the
   *  problem is extremely badly scaled (in which case the exponents should
   *  be increased).
   */
  double min_step = 1.0e-20;

  /**
   * The maximum step of the line search.
   *  The default value is 1.0e+20. This value need not be modified unless
   *  the exponents are too large for the machine being used, or unless the
   *  problem is extremely badly scaled (in which case the exponents should
   *  be increased).
   */
  double max_step = 1.0e+20;

  /**
   * A parameter to control the accuracy of the line search routine.
   *  The default value is 1.0e-4. This parameter should be greater
   *  than zero and smaller than 1.0.
   */
  double f_dec_coeff = 1.0e-4;

  /**
   * A parameter to control the accuracy of the line search routine.
   *  The default value is 0.9. If the function and gradient
   *  evaluations are inexpensive with respect to the cost of the
   *  iteration (which is sometimes the case when solving very large
   *  problems) it may be advantageous to set this parameter to a small
   *  value. A typical small value is 0.1. This parameter should be
   *  greater than the f_dec_coeff parameter and smaller than 1.0.
   */
  double s_curv_coeff = 0.9;

  /**
   * A parameter to ensure the global convergence for nonconvex functions.
   *  The default value is 1.0e-6. The parameter performs the so called
   *  cautious update for L-BFGS, especially when the convergence is
   *  not sufficient. The parameter must be positive but might as well
   *  be less than 1.0e-3 in practice.
   */
  double cautious_factor = 1.0e-6;

  /**
   * The machine precision for floating-point values. The default is 1.0e-16.
   *  This parameter must be a positive value set by a client program to
   *  estimate the machine precision.
   */
  double machine_prec = 1.0e-16;
};

/**
 * Return values of lbfgs_optimize().
 *  Roughly speaking, a negative value indicates an error.
 */
enum {
  /** L-BFGS reaches convergence. */
  LBFGS_CONVERGENCE = 0,
  /** L-BFGS satisfies stopping criteria. */
  LBFGS_STOP,
  /** The iteration has been canceled by the monitor callback. */
  LBFGS_CANCELED,

  /** Unknown error. */
  LBFGSERR_UNKNOWNERROR = -1024,
  /** Invalid number of variables specified. */
  LBFGSERR_INVALID_N,
  /** Invalid parameter lbfgs_parameter_t::mem_size specified. */
  LBFGSERR_INVALID_MEMSIZE,
  /** Invalid parameter lbfgs_parameter_t::g_epsilon specified. */
  LBFGSERR_INVALID_GEPSILON,
  /** Invalid parameter lbfgs_parameter_t::past specified. */
  LBFGSERR_INVALID_TESTPERIOD,
  /** Invalid parameter lbfgs_parameter_t::delta specified. */
  LBFGSERR_INVALID_DELTA,
  /** Invalid parameter lbfgs_parameter_t::min_step specified. */
  LBFGSERR_INVALID_MINSTEP,
  /** Invalid parameter lbfgs_parameter_t::max_step specified. */
  LBFGSERR_INVALID_MAXSTEP,
  /** Invalid parameter lbfgs_parameter_t::f_dec_coeff specified. */
  LBFGSERR_INVALID_FDECCOEFF,
  /** Invalid parameter lbfgs_parameter_t::s_curv_coeff specified. */
  LBFGSERR_INVALID_SCURVCOEFF,
  /** Invalid parameter lbfgs_parameter_t::machine_prec specified. */
  LBFGSERR_INVALID_MACHINEPREC,
  /** Invalid parameter lbfgs_parameter_t::max_linesearch specified. */
  LBFGSERR_INVALID_MAXLINESEARCH,
  /** The function value became NaN or Inf. */
  LBFGSERR_INVALID_FUNCVAL,
  /** The line-search step became smaller than lbfgs_parameter_t::min_step. */
  LBFGSERR_MINIMUMSTEP,
  /** The line-search step became larger than lbfgs_parameter_t::max_step. */
  LBFGSERR_MAXIMUMSTEP,
  /** Line search reaches the maximum, assumptions not satisfied or precision
     not achievable.*/
  LBFGSERR_MAXIMUMLINESEARCH,
  /** The algorithm routine reaches the maximum number of iterations. */
  LBFGSERR_MAXIMUMITERATION,
  /** Relative search interval width is at least
     lbfgs_parameter_t::machine_prec. */
  LBFGSERR_WIDTHTOOSMALL,
  /** A logic error (negative line-search step) occurred. */
  LBFGSERR_INVALIDPARAMETERS,
  /** The current search direction increases the cost function value. */
  LBFGSERR_INCREASEGRADIENT,
};

/**
 * Callback interface to provide cost function and gradient evaluations.
 *
 *  The lbfgs_optimize() function call this function to obtain the values of
 * cost function and its gradients when needed. A client program must implement
 *  this function to evaluate the values of the cost function and its
 *  gradients, given current values of variables.
 *
 *  @param  instance    The user data sent for lbfgs_optimize() function by the
 * client.
 *  @param  x           The current values of variables.
 *  @param  g           The gradient vector. The callback function must compute
 *                      the gradient values for the current variables.
 *  @retval double      The value of the cost function for the current
 * variables.
 */
typedef void (*lbfgs_evaluate_t)(void *instance, const Eigen::VectorXd &x,
                                 Eigen::VectorXd &g,
                                 Eigen::MatrixXd *cost_matrix);

/**
 * Callback interface to provide an upper bound at the beginning of the current
 * line search.
 *
 *  The lbfgs_optimize() function call this function to obtain the values of the
 *  upperbound of the stepsize to search in, provided with the beginning values
 * of variables before the line search, and the current step vector (can be
 * descent direction). A client program can implement this function for more
 * efficient linesearch. Any step larger than this bound should not be
 * considered. For example, it has a very large or even inf function value. Note
 * that the function value at the provided bound should be FINITE! If it is not
 * used, just set it nullptr.
 *
 *  @param  instance    The user data sent for lbfgs_optimize() function by the
 * client.
 *  @param  xp          The values of variables before current line search.
 *  @param  d           The step vector. It can be the descent direction.
 *  @retval double      The upperboud of the step in current line search
 * routine, such that (stpbound * d) is the maximum reasonable step.
 */
typedef double (*lbfgs_stepbound_t)(void *instance, const Eigen::VectorXd &xp,
                                    const Eigen::VectorXd &d);

/**
 * Callback interface to monitor the progress of the minimization process.
 *
 *  The lbfgs_optimize() function call this function for each iteration.
 * Implementing this function, a client program can store or display the current
 * progress of the minimization process. If it is not used, just set it nullptr.
 *
 *  @param  instance    The user data sent for lbfgs_optimize() function by the
 * client.
 *  @param  x           The current values of variables.
 *  @param  g           The current gradient values of variables.
 *  @param  fx          The current value of the cost function.
 *  @param  step        The line-search step used for this iteration.
 *  @param  k           The iteration count.
 *  @param  ls          The number of evaluations called for this iteration.
 *  @retval int         Zero to continue the minimization process. Returning a
 *                      non-zero value will cancel the minimization process.
 */
typedef int (*lbfgs_progress_t)(void *instance, const Eigen::VectorXd &x,
                                const Eigen::VectorXd &g, const double fx,
                                const double step, const int k, const int ls);

/**
 * Callback data struct
 */
struct callback_data_t {
  void *instance = nullptr;
  lbfgs_evaluate_t proc_evaluate = nullptr;
};

// ----------------------- L-BFGS Part -----------------------

/**
 * Line search method for smooth or nonsmooth functions.
 *  This function performs line search to find a point that satisfy
 *  both the Armijo condition and the weak Wolfe condition. It is
 *  as robust as the backtracking line search but further applies
 *  to continuous and piecewise smooth functions where the strong
 *  Wolfe condition usually does not hold.
 *
 *  @see
 *      Adrian S. Lewis and Michael L. Overton. Nonsmooth optimization
 *      via quasi-Newton methods. Mathematical Programming, Vol 141,
 *      No 1, pp. 135-163, 2013.
 */
inline int line_search_lewisoverton(
    Eigen::VectorXd &x, double &f, Eigen::VectorXd &g, double &stp,
    const Eigen::VectorXd &s, const Eigen::VectorXd &xp,
    const Eigen::VectorXd &gp, const double stpmin, const double stpmax,
    const callback_data_t &cd, const lbfgs_parameter_t &param,
    const bool is_debug_mode) {
  int count = 0;
  bool brackt = false;
  bool touched = false;
  double finit;
  double dginit;
  double dgtest;
  double dstest;
  double mu = 0.0;
  double nu = stpmax;
  int res;
  Eigen::MatrixXd cost_matrix;

  /* Check the input parameters for errors. */
  if (!(stp > 0.0)) {
    return LBFGSERR_INVALIDPARAMETERS;
  }

  /* Compute the initial gradient in the search direction. */
  dginit = gp.dot(s);  // s是上一次的梯度的反方向，gp是上一次的梯度
  /* Make sure that s points to a descent direction. */
  if (0.0 < dginit) {
    return LBFGSERR_INCREASEGRADIENT;
  }

  /* The initial value of the cost function. */
  finit = f;
  dgtest = param.f_dec_coeff * dginit;
  dstest = param.s_curv_coeff * dginit;

  while (true) {
    x = xp + stp * s;

    /* Evaluate the function and gradient values. */
    cd.proc_evaluate(cd.instance, x, g, &cost_matrix);
    f = cost_matrix.sum();  // 这里又更新cost和grad
    ++count;

    /* Test for errors. */
    if (std::isinf(f) || std::isnan(f)) {
      res = LBFGSERR_INVALID_FUNCVAL;
      break;
    }

    if (param.past > 0 &&
        fabs(finit - f) / (fabs(finit) + 1.0) < param.delta / param.past) {
      res = count;
      break;
    }
    /* Check the Armijo condition. 是拟牛顿法选择步长的方法*/
    if (f > finit + stp * dgtest) {
      nu = stp;
      brackt = true;
    } else {
      /* Check the weak Wolfe condition. 线搜索选择步长的标准*/
      if (g.dot(s) < dstest) {
        mu = stp;
      } else {
        res = count;
        break;
      }
    }
    if (param.max_linesearch <=
        count)  // max_linesearch是每次迭代最大的尝试搜索次数
    {
      // std::cout<< fabs(dginit)/finit<<std::endl;
      // std::cout << fabs(finit-f)/(fabs(finit)+1.0) << std::endl;
      /* Maximum number of iteration. */
      res = LBFGSERR_MAXIMUMLINESEARCH;
      break;
    }
    if (brackt && (nu - mu) < param.machine_prec * nu) {
      /* Relative interval width is at least machine_prec. */
      res = LBFGSERR_WIDTHTOOSMALL;
      break;
    }

    if (brackt) {
      stp = 0.5 * (mu + nu);
    } else {
      stp *= 2.0;
    }

    if (stp < stpmin) {
      /* The step is the minimum value. */
      res = LBFGSERR_MINIMUMSTEP;
      break;
    }
    if (stp > stpmax) {
      if (touched) {
        /* The step is the maximum value. */
        res = LBFGSERR_MAXIMUMSTEP;
        break;
      } else {
        /* The maximum value should be tried once. */
        touched = true;  // 多给一次机会
        stp = stpmax;
      }
    }
  }
  if (is_debug_mode) {
    // auto iter_data = ret->add_data_iteration();
    // iter_data->set_cost(f);
    // for (int j = 0; j < cost_matrix.cols(); j++) {  // 每一段的cost
    //   auto *cost_map = iter_data->add_detail_cost();
    //   for (int k = 0; k < cost_matrix.rows(); k++) {  // cost类型
    //     cost_map->add_value(cost_matrix(k, j));
    //   }
    // }
  }
  // std::cout << "jerk cost:" << cost_matrix.col(0).sum() << ";   "
  //           << "acc cost:" << cost_matrix.col(1).sum() << ";   "
  //           << "time cost: " << cost_matrix.col(2).sum() << ";   "
  //           << "static obs cost: " << cost_matrix.col(3).sum() << ";   "
  //           << "feas cost: " << cost_matrix.col(4).sum() << ";   "
  //           << "ref cost: " << cost_matrix.col(5).sum() << "; "
  //           << "cur cost: " << cost_matrix.col(6).sum() << ";" << std::endl;
  // std::cout << "cost change: " << f << std::endl;
  return res;
}

/**
 * Start a L-BFGS optimization.
 * Assumptions: 1. f(x) is either C2 or C0 but piecewise C2;
 *              2. f(x) is lower bounded;
 *              3. f(x) has bounded level sets;
 *              4. g(x) is either the gradient or subgradient;
 *              5. The gradient exists at the initial guess x0.
 * A user must implement a function compatible with ::lbfgs_evaluate_t
 * (evaluation callback) and pass the pointer to the callback function to
 * lbfgs_optimize() arguments. Similarly, a user can implement a function
 * compatible with
 * ::lbfgs_stepbound_t to provide an external upper bound for stepsize, and
 * ::lbfgs_progress_t (progress callback) to obtain the current progress
 * (e.g., variables, function, and gradient, etc) and to cancel the iteration
 * process if necessary. Implementation of the stepbound and the progress
 * callback is optional: a user can pass nullptr if progress notification is not
 * necessary.
 *
 *
 *  @param  x               The vector of decision variables.
 *                          THE INITIAL GUESS x0 SHOULD BE SET BEFORE THE CALL!
 *                          A client program can receive decision variables
 *                          through this vector, at which the cost and its
 *                          gradient are queried during minimization.
 *  @param  f               The ref to the variable that receives the final
 *                          value of the cost function for the variables.
 *  @param  proc_evaluate   The callback function to provide function f(x) and
 *                          gradient g(x) evaluations given a current values of
 *                          variables x. A client program must implement a
 *                          callback function compatible with lbfgs_evaluate_t
 *                          and pass the pointer to the callback function.
 *  @param  proc_stepbound  The callback function to provide values of the
 *                          upperbound of the stepsize to search in, provided
 *                          with the beginning values of variables before the
 *                          line search, and the current step vector (can be
 *                          negative gradient). A client program can implement
 *                          this function for more efficient linesearch. If it
 * is not used, just set it nullptr.
 *  @param  proc_progress   The callback function to receive the progress
 *                          (the number of iterations, the current value of
 *                          the cost function) of the minimization
 *                          process. This argument can be set to nullptr if
 *                          a progress report is unnecessary.
 *  @param  instance        A user data pointer for client programs. The
 * callback functions will receive the value of this argument.
 *  @param  param           The parameters for L-BFGS optimization.
 *  @retval int             The status code. This function returns a nonnegative
 *                          integer if the minimization process terminates
 * without an error. A negative integer indicates an error.
 */
inline int lbfgs_optimize(Eigen::VectorXd &x, double &f,
                          lbfgs_evaluate_t proc_evaluate, void *instance,
                          const lbfgs_parameter_t &param,
                          const bool is_debug_mode) {
  // 调整函数的输入输出，保证输入在前，输出在后
  int res;
  int i;
  int j;
  int k;
  int ls;
  int end;
  int bound;
  double step = 1.0;
  double step_min;
  double step_max;
  double fx;
  double ys;
  double yy;
  double gnorm_inf;
  double xnorm_inf;
  double beta;
  double rate;
  double cau;

  const int n = x.size();
  const int m = param.mem_size;

  /* Check the input parameters for errors. */
  if (n <= 0) {
    return LBFGSERR_INVALID_N;
  }
  if (m <= 0) {
    return LBFGSERR_INVALID_MEMSIZE;
  }
  if (param.g_epsilon < 0.0) {
    return LBFGSERR_INVALID_GEPSILON;
  }
  if (param.past < 0) {
    return LBFGSERR_INVALID_TESTPERIOD;
  }
  if (param.delta < 0.0) {
    return LBFGSERR_INVALID_DELTA;
  }
  if (param.min_step < 0.0) {
    return LBFGSERR_INVALID_MINSTEP;
  }
  if (param.max_step < param.min_step) {
    return LBFGSERR_INVALID_MAXSTEP;
  }
  if (!(param.f_dec_coeff > 0.0 && param.f_dec_coeff < 1.0)) {
    return LBFGSERR_INVALID_FDECCOEFF;
  }
  if (!(param.s_curv_coeff < 1.0 && param.s_curv_coeff > param.f_dec_coeff)) {
    return LBFGSERR_INVALID_SCURVCOEFF;
  }
  if (!(param.machine_prec > 0.0)) {
    return LBFGSERR_INVALID_MACHINEPREC;
  }
  if (param.max_linesearch <= 0) {
    return LBFGSERR_INVALID_MAXLINESEARCH;
  }

  /* Prepare intermediate variables. */
  Eigen::VectorXd xp(n);
  Eigen::VectorXd g(n);
  Eigen::VectorXd gp(n);
  Eigen::VectorXd d(n);
  Eigen::VectorXd pf(std::max(1, param.past));

  /* Initialize the limited memory. */
  Eigen::VectorXd lm_alpha = Eigen::VectorXd::Zero(m);
  Eigen::MatrixXd lm_s = Eigen::MatrixXd::Zero(n, m);
  Eigen::MatrixXd lm_y = Eigen::MatrixXd::Zero(n, m);
  Eigen::VectorXd lm_ys = Eigen::VectorXd::Zero(m);

  /* Construct a callback data. */
  callback_data_t cd;
  cd.instance = instance;
  cd.proc_evaluate = proc_evaluate;

  /* Evaluate the function value and its gradient. */
  Eigen::MatrixXd cost;
  cd.proc_evaluate(cd.instance, x, g, &cost);
  fx = cost.sum();  // 这里计算cost和grad
  // std::cout << "jerk cost:" << cost.col(0).sum() << ";   "
  //           << "acc cost:" << cost.col(1).sum() << ";   "
  //           << "time cost: " << cost.col(2).sum() << ";   "
  //           << "static obs cost: " << cost.col(3).sum() << ";   "
  //           << "feas cost: " << cost.col(4).sum() << ";   "
  //           << "ref cost: " << cost.col(5).sum() << "; "
  //           << "cur cost: " << cost.col(6).sum() << ";" << std::endl;
  // std::cout << "cost change: " << fx << std::endl;
  if (is_debug_mode) {
    // auto iter_data = ret->add_data_iteration();
    // iter_data->set_cost(fx);
    // for (int j = 0; j < cost.cols(); j++) {  // 每一段的cost
    //   auto *cost_map = iter_data->add_detail_cost();
    //   for (int k = 0; k < cost.rows(); k++) {  // cost类型
    //     cost_map->add_value(cost(k, j));
    //   }
    // }
  }

  /* Store the initial value of the cost function. */
  pf(0) = fx;

  // 输出cost
  // auto c = ret->add_data_iteration();

  /*
  Compute the direction;
  we assume the initial hessian matrix H_0 as the identity matrix.
  */
  d = -g;  // 梯度的反方向

  /*
  Make sure that the initial variables are not a stationary point.
  */
  gnorm_inf = g.cwiseAbs().maxCoeff();  // 找到g中的绝对值的最大值
  xnorm_inf = x.cwiseAbs().maxCoeff();

  if (gnorm_inf / std::max(1.0, xnorm_inf) < param.g_epsilon) {
    /* The initial guess is already a stationary point. */
    res = LBFGS_CONVERGENCE;
  } else {
    /*
    Compute the initial step:
    */
    step = 1.0 / d.norm();

    k = 1;
    end = 0;
    bound = 0;
    // double while_start = npp::now_ms();
    // double total_ls = 0.0;
    while (true) {
      /* Store the current position and gradient vectors. */
      xp = x;
      gp = g;

      /* If the step bound can be provied dynamically, then apply it. */
      step_min = param.min_step;
      step_max = param.max_step;
      /* Search for an optimal step. */
      // double ls_start = npp::now_ms();
      ls = line_search_lewisoverton(
          x, fx, g, step, d, xp, gp, step_min, step_max, cd, param,
          is_debug_mode);  // 求解迭代步长，里面会再计算cost和grad
      // auto cost = cd.proc_evaluate(cd.instance, x, g);
      // fx = cost.sum();  // 这里计算cost和grad

      if (ls < 0) {
        /* Revert to the previous point. */
        x = xp;
        g = gp;
        res = ls;
        break;
      }
      /*
      Convergence test.
      The criterion is given by the following formula:
      ||g(x)||_inf / max(1, ||x||_inf) < g_epsilon
      */
      gnorm_inf = g.cwiseAbs().maxCoeff();
      xnorm_inf = x.cwiseAbs().maxCoeff();
      if (gnorm_inf / std::max(1.0, xnorm_inf) <
          param.g_epsilon)  // 同样的收敛条件
      {
        /* Convergence. */
        res = LBFGS_CONVERGENCE;
        break;
      }

      /*
      Test for stopping criterion.
      The criterion is given by the following formula:
      |f(past_x) - f(x)| / max(1, |f(x)|) < \delta.
      */
      if (0 < param.past) {
        /* We don't test the stopping criterion while k < past. */
        if (param.past <= k) {
          /* The stopping criterion. */
          rate = std::fabs(pf(k % param.past) - fx) /
                 std::max(1.0,
                          std::fabs(fx));  // pf是previous
          // f，两次差的cost非常小的时候就判断可以stop

          if (rate < param.delta) {
            res = LBFGS_STOP;
            break;
          }
        }

        /* Store the current value of the cost function. */
        pf(k % param.past) = fx;
      }

      if (param.max_iterations != 0 &&
          param.max_iterations <= k)  // k是当前迭代次数
      {
        /* Maximum number of iterations. */
        res = LBFGSERR_MAXIMUMITERATION;
        break;
      }

      /* Count the iteration number. */
      ++k;

      /*
      Update vectors s and y:
      s_{k+1} = x_{k+1} - x_{k} = \step * d_{k}.
      y_{k+1} = g_{k+1} - g_{k}.
      */
      lm_s.col(end) = x - xp;
      lm_y.col(end) = g - gp;

      /*
      Compute scalars ys and yy:
      ys = y^t \cdot s = 1 / \rho.
      yy = y^t \cdot y.
      Notice that yy is used for scaling the hessian matrix H_0 (Cholesky
      factor).
      */
      ys = lm_y.col(end).dot(lm_s.col(end));
      yy = lm_y.col(end).squaredNorm();
      lm_ys(end) = ys;

      /* Compute the negative of gradients. */
      d = -g;

      /*
      Only cautious update is performed here as long as
      (y^t \cdot s) / ||s_{k+1}||^2 > \epsilon * ||g_{k}||^\alpha,
      where \epsilon is the cautious factor and a proposed value
      for \alpha is 1.
      This is not for enforcing the PD of the approxomated Hessian
      since ys > 0 is already ensured by the weak Wolfe condition.
      This is to ensure the global convergence as described in:
      Dong-Hui Li and Masao Fukushima. On the global convergence of
      the BFGS method for nonconvex unconstrained optimization problems.
      SIAM Journal on Optimization, Vol 11, No 4, pp. 1054-1064, 2011.
      */
      cau = lm_s.col(end).squaredNorm() * gp.norm() * param.cautious_factor;

      if (ys > cau) {
        /*
        Recursive formula to compute dir = -(H \cdot g).
        This is described in page 779 of:
        Jorge Nocedal.
        Updating Quasi-Newton Matrices with Limited Storage.
        Mathematics of Computation, Vol. 35, No. 151,
        pp. 773--782, 1980.
        */
        ++bound;
        bound = m < bound ? m : bound;
        end = (end + 1) % m;

        j = end;
        for (i = 0; i < bound; ++i) {
          j = (j + m - 1) % m; /* if (--j == -1) j = m-1; */
          /* \alpha_{j} = \rho_{j} s^{t}_{j} \cdot q_{k+1}. */
          lm_alpha(j) = lm_s.col(j).dot(d) / lm_ys(j);
          /* q_{i} = q_{i+1} - \alpha_{i} y_{i}. */
          d += (-lm_alpha(j)) * lm_y.col(j);
        }

        d *= ys / yy;

        for (i = 0; i < bound; ++i) {
          /* \beta_{j} = \rho_{j} y^t_{j} \cdot \gamm_{i}. */
          beta = lm_y.col(j).dot(d) / lm_ys(j);
          /* \gamm_{i+1} = \gamm_{i} + (\alpha_{j} - \beta_{j}) s_{j}. */
          d += (lm_alpha(j) - beta) * lm_s.col(j);
          j = (j + 1) % m; /* if (++j == m) j = 0; */
        }
      }

      /* The search direction d is ready. We try step = 1 first. */
      step = 1.0;
    }
    // std::cout << "lbfgs while循环耗时: " << npp::now_ms() - while_start <<
    // "ms"
    //           << std::endl;
  }

  /* Return the final value of the cost function. */
  f = fx;
  // std::cout << "iter num: " << k << std::endl;
  // std::cout << "final cost: " << f << std::endl;
  // std::cout << "optimization result: " << res << std::endl;
  return res;
}
}  // namespace lbfgs
