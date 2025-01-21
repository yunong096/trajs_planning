#include "path_planner_ilqr.h"

#include <cmath>

#include "core/modules/common/math/vec2d.h"
#include "core/modules/path_planner/path_planner_preprocessor/path_planner_constants.hpp"
#include "forward_simulator.h"
#include "ilqr_data.pb.h"
#include "ilqr_math_utils.h"
#include "motion_plan_common.pb.h"
#include "path_planner_constraint.h"
#include "path_planner_cost.h"
#include "path_planner_data.h"
#include "thirdparty/fplus/fplus.hpp"

namespace lat_ilqr {

using ilqr_solver::BoundConstraint;
using ilqr_solver::ILqr;
using ilqr_solver::SolverConfig;

using npp::pnc::math_utils::LinearInterp;
using npp::pnc::math_utils::NormalizeAngle;
using npp::pnc::math_utils::ResizeAndResetEigenVec;

static constexpr double kEps = 1e-6;
static constexpr double kDeg2rad = M_PI / 180.0;
static constexpr double kAccelLimitRatio = 40.0;
static constexpr double kJerkLimitRatio = 200.0;
static constexpr double kSnapLimitRatio = 5.0;
static constexpr double kDeltaTolerance = 0.1 * kDeg2rad;
static constexpr double kOmegaTolerance = 0.1 * kDeg2rad;
static constexpr double kAccelTolerance = 0.05;
static constexpr double kJerklTolerance = 0.05;
static constexpr double kConstraintHorizonRatio = 0.25;
static constexpr double kConstraintDefault = 10.0;

void ILQRPathPlannerProblem::Init(const SolverConfig &ilqr_solver_config) {
  data_ = std::make_unique<LatIlqrDataCache>();
  ilqr_solver_config_ptr_ = std::make_unique<SolverConfig>(ilqr_solver_config);
  data_->ResizeStepData(Horizon() + 1);
  ilqr_core_ptr_ = std::make_unique<ILqr>();
  common_term_calculator_ = std::make_unique<LatIlqrCommonTerm>(data_.get());
  lat_model_ = std::make_unique<ILqrLatModel>(*data_);
  ilqr_core_ptr_->Init(lat_model_.get(), ilqr_solver_config_ptr_.get(),
                       common_term_calculator_.get());
  cost_stack_vec_.emplace_back(std::make_unique<PathRefCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<ThetaRefCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<DesireHeadingCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<AccelCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<JerkCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<LaneBoundaryCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<ObstacleCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<SteeringBoundCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<SnapCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<CartRBSoftCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<CartRBHardCostTerm>(*data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<StreamWallSoftCostTerm>(*data_));
  cost_stack_vec_.emplace_back(
      std::make_unique<StreamWallHardCostTerm>(*data_));
  cost_stack_vec_.emplace_back(std::make_unique<CartLBSoftCostTerm>(*data_));

  for (const auto &cost_ptr : cost_stack_vec_) {
    ilqr_core_ptr_->AddCost(cost_ptr.get());
  }

  ResizeAndResetEigenVec(x0_, Horizon() + 1, GetStateSize());
  InitConstrainConfig(ilqr_solver_config_ptr_.get());
}

void ILQRPathPlannerProblem::InitConstrainConfig(
    const SolverConfig *const /*ilqr_solver_config_ptr*/) {
  if (!EnableCilqr()) {
    return;
  }
  ilqr_core_ptr_->SetConstraintSize(ILQRCostraintId::CONSTRAIN_SIZE);

  for (size_t step = 0; step < Horizon() + 1; step++) {
    std::vector<BoundConstraint> constraint_vec;
    constraint_vec.reserve(ILQRCostraintId::CONSTRAIN_SIZE);

    constraint_vec_.emplace_back(std::make_unique<DeltaConstraint>(*data_));
    auto delta_constraint = BoundConstraint();
    delta_constraint.Init(constraint_vec_.back().get(), kDeltaTolerance,
                          ilqr_solver_config_ptr_.get());
    constraint_vec.emplace_back(delta_constraint);

    constraint_vec_.emplace_back(std::make_unique<OmegaConstraint>(*data_));
    auto omega_constraint = BoundConstraint();
    omega_constraint.Init(constraint_vec_.back().get(), kOmegaTolerance,
                          ilqr_solver_config_ptr_.get());
    constraint_vec.emplace_back(omega_constraint);

    constraint_vec_.emplace_back(std::make_unique<AccelConstraint>(*data_));
    auto accel_constraint = BoundConstraint();
    accel_constraint.Init(constraint_vec_.back().get(), kAccelTolerance,
                          ilqr_solver_config_ptr_.get());
    constraint_vec.emplace_back(accel_constraint);

    constraint_vec_.emplace_back(std::make_unique<JerkConstraint>(*data_));
    auto jerk_constraint = BoundConstraint();
    jerk_constraint.Init(constraint_vec_.back().get(), kJerklTolerance,
                         ilqr_solver_config_ptr_.get());
    constraint_vec.emplace_back(jerk_constraint);

    ilqr_core_ptr_->AddConstraint(step, constraint_vec);
  }

  // (optional)update extra info, for example, increase the weight of control
  ilqr_core_ptr_->SetCallback(
      [&](const std::size_t outer_iter, const bool &reset) {
        constexpr std::size_t kUpdateIter = 7;
        if (outer_iter > kUpdateIter) {
          return;
        }
        constexpr double kSnapMax = 1000.0;
        double snap_scale = (*data_)(SNAP_SCALE);
        snap_scale *= 2.0;
        if (reset) {
          snap_scale = (*data_)(INIT_SNAP_SCALE);
        }
        snap_scale = std::fmin(snap_scale, kSnapMax);
        *(*data_).Mutable(SNAP_SCALE) = snap_scale;
      });
}

bool ILQRPathPlannerProblem::RbCollisionChecker(
    const StateVec & /*x_vec*/) const {
  return false;
}

bool ILQRPathPlannerProblem::PathCircleChecker(const StateVec &x_vec) const {
  constexpr double kThetaDiffGradLimit = 120 * kDeg2rad;
  for (std::size_t i = 0; i < x_vec.size(); i++) {
    const double theta = x_vec[i][StateId::THETA];
    const double ref_theta = (*data_)(REF_THETA, i);
    if (std::fabs(NormalizeAngle(theta - ref_theta)) > kThetaDiffGradLimit) {
      return true;
    }
  }
  return false;
}

void ILQRPathPlannerProblem::GenInitSeed(
    const npp::pnc::ilqr::LatMotionPlanInput &input, OutputInfo *output) const {
  constexpr double kPreviewTime = 2.0;
  const int sim_size = Horizon() + 1;
  npp::pnc::ForwardSimAgent ego_agent(
      input.ppp_input().init_state(), kPreviewTime,
      input.ppp_input().agent_params().wheel_base(), GetTimeStep(), sim_size);
  ego_agent.SetAccAndJerkLimit((*data_)(ACCEL_LIMIT), (*data_)(JERK_LIMIT));

  const auto &referenline = input.ppp_output().ref_lines();
  std::vector<npp::planning_math::Vec2d> input_pts;
  input_pts.reserve(referenline.size());
  for (const auto &pt : referenline) {
    input_pts.push_back({pt.x(), pt.y()});
  }

  npp::pnc::TargetPath target_path(input_pts);
  npp::pnc::ilqr::Output traj{};

  bool forward_sim_ok =
      npp::pnc::ForwardSimulation::Simulation(target_path, ego_agent, &traj);
  if (!forward_sim_ok) {
    // TODO: formulate a final backup solution
  }
  ControlVec forward_sim_u{};
  ResizeAndResetEigenVec(forward_sim_u, Horizon(), GetInputSize());

  StateVec forward_sim_x{};
  ResizeAndResetEigenVec(forward_sim_x, Horizon() + 1, GetStateSize());

  Control control(1);
  for (int i = 0; i < static_cast<int>(forward_sim_x.size()); i++) {
    if (i < static_cast<int>(forward_sim_u.size())) {
      control(0) = i < traj.controls_size() ? traj.controls(i) : 0.0;
      forward_sim_u[i] = control;
    }
    auto &cur_state = traj.states(i);
    forward_sim_x[i] << cur_state.x(), cur_state.y(), cur_state.theta(),
        cur_state.delta(), cur_state.gamma();
  }

  *output = {forward_sim_x, forward_sim_u, PathCircleChecker(forward_sim_x),
             RbCollisionChecker(forward_sim_x), true};
}

void ILQRPathPlannerProblem::FillHardBoundScaleAndShiftRefCenteringCost(
    const npp::pnc::ilqr::LatMotionPlanInput &input) const {
  const auto &params = input.ppp_input().ilqr_params();

  auto Data = [this](FixedDataIndex index) -> double & {
    return *data_->Mutable(index);
  };
  Data(JERK_LIMIT_RATIO) = kJerkLimitRatio;
  Data(ACCEL_LIMIT_RATIO) = kAccelLimitRatio;
  Data(SNAP_LIMIT_RATIO) = kSnapLimitRatio;
  Data(CART_RB_HARD_SCALE) = params.cart_rb_hard_scale();
  Data(CART_RB_SOFT_SCALE) = params.cart_rb_soft_scale();
  Data(STREAM_WALL_HARD_SCALE) = params.stream_wall_hard_scale();
  Data(STREAM_WALL_SOFT_SCALE) = params.stream_wall_soft_scale();
  Data(CART_LB_SOFT_SCALE) = params.cart_lb_soft_scale();
  // reset wheel angle bound scale
  Data(WHEEL_ANGLE_BOUND_SCALE) = params.delta_bnd_scale();
  // reset lane boundary scale
  // convert ref_centering cost back to l distance
  Data(CONVERT_REF_CENTERING_COST) = -kEps;
}

void ILQRPathPlannerProblem::Update(
    const npp::pnc::ilqr::LatMotionPlanInput &input,
    npp::pnc::ilqr::LatMotionPlanOutput *output,
    std::vector<npp::pnc::State> *dense_output) {
  ClearCandidatesOuput(&output_candidates_);
  UpdateCostConfig(input);
  UpdateInitState(input.ppp_input().init_state(), &(x0_[0]));
  UpdateWarmStartData(input);
  UpdateConstrainBound(EnableCilqrTest(), input.ppp_input().init_state().v(),
                       input, output->mutable_output_candidates());
  Solve(&output_candidates_[SolutionId::kNormal]);
  BackUpMode(input, &output_candidates_);
  const double wheel_base = input.ppp_input().agent_params().wheel_base();
  const int final_traj_index =
      UpdateOutput(output_candidates_, wheel_base, output);
  GenDenseTraj(wheel_base, output_candidates_[final_traj_index].x_vec,
               output_candidates_[final_traj_index].u_vec, dense_output);
  UpdateOutputSolverInfo(*ilqr_core_ptr_->GetSolverInfoPtr(), output);
  UpdateCilqrSolverInfo(*ilqr_core_ptr_->GetCilqrInfoPtr(), output);
}

void ILQRPathPlannerProblem::UpdateConstrainBound(
    const bool enable_constraint_test, const double v,
    const npp::pnc::ilqr::LatMotionPlanInput &input,
    npp::pnc::ilqr::OutputCandidates *out_candidates_pb) const {
  if (!EnableCilqr()) {
    return;
  }
  auto lat_ilqr_bound_params = input.ppp_input().lat_iqlr_bounds();
  double accel_bound = lat_ilqr_bound_params.accel_bound();
  std::vector<double> jerk_bound_list(
      lat_ilqr_bound_params.jerk_bound().begin(),
      lat_ilqr_bound_params.jerk_bound().end());
  std::vector<double> v_list(lat_ilqr_bound_params.v_vec().begin(),
                             lat_ilqr_bound_params.v_vec().end());
  double jerk_bound = LinearInterp::Interp(v, v_list, jerk_bound_list);
  double delta_bound = ILqrLatModel::FunctionSafetyDeltaLimit(v);
  constexpr double kLimitConstraintRatio = 0.7;
  const double default_delta_bound =
      ILqrLatModel::FunctionSafetyDeltaLimit(0.0);
  double omega_bound =
      ILqrLatModel::FunctionSafetyOmegaLimit(v) * kLimitConstraintRatio;
  const double default_omega_bound =
      ILqrLatModel::FunctionSafetyOmegaLimit(0.0) * kLimitConstraintRatio;

  auto SetConstraintBound = [](const double lower, const double upper,
                               npp::pnc::Bound *constraint) {
    constraint->set_lower(lower);
    constraint->set_upper(upper);
  };

  auto *constrains_info = out_candidates_pb->mutable_constraints();
  npp::pnc::math_utils::ProtoUtil::EnsureSize<npp::pnc::ilqr::ConstrainInfo>(
      constrains_info, Horizon() + 1);
  const size_t constraint_max_index = *data_->Mutable(CONSTRAINT_MAX_INDEX);
  for (size_t i = 0; i < Horizon() + 1; i++) {
    auto DataAtStep = [this, i](StepDataIndex index) -> double & {
      return *data_->Mutable(index, i);
    };

    if (i <= constraint_max_index) {
      DataAtStep(ACCEL_CONSTRAINT_LOWER) = -accel_bound;
      DataAtStep(ACCEL_CONSTRAINT_UPPER) = accel_bound;

      DataAtStep(JERK_CONSTRAINT_LOWER) = -jerk_bound;
      DataAtStep(JERK_CONSTRAINT_UPPER) = jerk_bound;

      DataAtStep(DELTA_CONSTRAINT_LOWER) = -delta_bound;
      DataAtStep(DELTA_CONSTRAINT_UPPER) = delta_bound;

      DataAtStep(OMEGA_CONSTRAINT_LOWER) = -omega_bound;
      DataAtStep(OMEGA_CONSTRAINT_UPPER) = omega_bound;

    } else {
      DataAtStep(ACCEL_CONSTRAINT_LOWER) = -kConstraintDefault;
      DataAtStep(ACCEL_CONSTRAINT_UPPER) = kConstraintDefault;

      DataAtStep(JERK_CONSTRAINT_LOWER) = -kConstraintDefault;
      DataAtStep(JERK_CONSTRAINT_UPPER) = kConstraintDefault;

      const double min_delta_limit =
          std::min(delta_bound * kConstraintExpandScale, default_delta_bound);
      DataAtStep(DELTA_CONSTRAINT_LOWER) = -min_delta_limit;
      DataAtStep(DELTA_CONSTRAINT_UPPER) = min_delta_limit;
      const double min_omega_limit =
          std::min(omega_bound * kConstraintExpandScale, default_omega_bound);
      DataAtStep(OMEGA_CONSTRAINT_LOWER) = -min_omega_limit;
      DataAtStep(OMEGA_CONSTRAINT_UPPER) = min_omega_limit;
    }

    if (enable_constraint_test) {
      PybindTest(ilqr_solver_config_ptr_->cilqr_config(), i);
    }

    auto &constraint_pb = *out_candidates_pb->mutable_constraints(i);
    SetConstraintBound(DataAtStep(ACCEL_CONSTRAINT_LOWER),
                       DataAtStep(ACCEL_CONSTRAINT_UPPER),
                       constraint_pb.mutable_accel_bound());
    SetConstraintBound(DataAtStep(JERK_CONSTRAINT_LOWER),
                       DataAtStep(JERK_CONSTRAINT_UPPER),
                       constraint_pb.mutable_jerk_bound());
    SetConstraintBound(DataAtStep(DELTA_CONSTRAINT_LOWER),
                       DataAtStep(DELTA_CONSTRAINT_UPPER),
                       constraint_pb.mutable_delta_bound());
    SetConstraintBound(DataAtStep(OMEGA_CONSTRAINT_LOWER),
                       DataAtStep(OMEGA_CONSTRAINT_UPPER),
                       constraint_pb.mutable_omega_bound());
  }
}

void ILQRPathPlannerProblem::BackUpMode(
    const npp::pnc::ilqr::LatMotionPlanInput &input,
    std::vector<OutputInfo> *output_list) {
  auto cost_check_result = CheckCost(*data_, Horizon() + 1);
  if (!cost_check_result.first ||
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition ==
          ilqr_solver::ILqr::iLqrSolveCondition::BACKWARD_PASS_FAIL ||
      ilqr_core_ptr_->GetSolverInfoPtr()->solver_condition ==
          ilqr_solver::ILqr::iLqrSolveCondition::INIT_TERMINATE) {
    ilqr_core_ptr_->Reset();
    RestHardBoundScaleAndShiftRefCenteringCost();
    // disable constrain when get init value at backup mode
    const bool enable_cilqr = EnableCilqr();
    ilqr_solver_config_ptr_->set_enable_cilqr(false);
    Solve(&(output_list->at(SolutionId::kBackUpInit)));
    FillHardBoundScaleAndShiftRefCenteringCost(input);
    ilqr_solver_config_ptr_->set_enable_cilqr(enable_cilqr);
    ilqr_core_ptr_->UpdateWarmStart(
        {output_list->at(SolutionId::kBackUpInit).u_vec});
    Solve(&(output_list->at(SolutionId::kBackUpFinal)));
  }
}

void ILQRPathPlannerProblem::FillRefLineInfoAndCommonTerms(
    const npp::pnc::RefPoint &ref_point, const double init_v,
    const double wheel_base, size_t step) const {
  auto DataAtStep = [this, step](StepDataIndex index) -> double & {
    return *data_->Mutable(index, step);
  };
  DataAtStep(REF_X) = ref_point.x();
  DataAtStep(REF_Y) = ref_point.y();
  DataAtStep(REF_THETA) = ref_point.theta();
  (*(*data_).Mutable(INIT_VEL)) = init_v;
  (*(*data_).Mutable(CURV_FACTOR)) = 1.0 / wheel_base;
  (*(*data_).Mutable(WHEEL_BASE)) = wheel_base;
  DataAtStep(REF_COS_THETA) = std::cos(ref_point.theta());
  DataAtStep(REF_COS_THETA_SQUARE) = std::pow(DataAtStep(REF_COS_THETA), 2.0);
  DataAtStep(REF_SIN_THETA) = std::sin(ref_point.theta());
  DataAtStep(REF_SIN_THETA_SQUARE) = std::pow(DataAtStep(REF_SIN_THETA), 2.0);
  DataAtStep(REF_COST_THETA_DOT_REF_SIN_THETA) =
      DataAtStep(REF_SIN_THETA) * DataAtStep(REF_COS_THETA);
  DataAtStep(SAMPLE_S) = ref_point.s();
}

void ILQRPathPlannerProblem::FillBoundInfo(
    const npp::pnc::micro_decider::DecisionInfo &bnd, size_t step) const {
  auto DataAtStep = [this, step](StepDataIndex index) -> double & {
    return *data_->Mutable(index, step);
  };
  DataAtStep(REF_OFFSET) = bnd.ref_offset().value();
  DataAtStep(REF_OFFSET_X) =
      DataAtStep(REF_X) - bnd.ref_offset().value() * DataAtStep(REF_SIN_THETA);
  DataAtStep(REF_OFFSET_Y) =
      DataAtStep(REF_Y) + bnd.ref_offset().value() * DataAtStep(REF_COS_THETA);

  DataAtStep(DESIRE_HEADING) = bnd.desire_heading().value();
  DataAtStep(CART_RB_REAR_SOFT_LOWER) = bnd.cart_rb_soft_rear().lower();
  DataAtStep(CART_RB_REAR_SOFT_UPPER) = bnd.cart_rb_soft_rear().upper();
  DataAtStep(CART_RB_REAR_HARD_LOWER) = bnd.cart_rb_hard_rear().lower();
  DataAtStep(CART_RB_REAR_HARD_UPPER) = bnd.cart_rb_hard_rear().upper();
  DataAtStep(CART_RB_FRONT_SOFT_LOWER) = bnd.cart_rb_soft_front().lower();
  DataAtStep(CART_RB_FRONT_SOFT_UPPER) = bnd.cart_rb_soft_front().upper();
  DataAtStep(CART_RB_FRONT_HARD_LOWER) = bnd.cart_rb_hard_front().lower();
  DataAtStep(CART_RB_FRONT_HARD_UPPER) = bnd.cart_rb_hard_front().upper();
  DataAtStep(CART_RB_CENTER_SOFT_LOWER) = bnd.cart_rb_soft_center().lower();
  DataAtStep(CART_RB_CENTER_SOFT_UPPER) = bnd.cart_rb_soft_center().upper();
  DataAtStep(CART_RB_CENTER_HARD_LOWER) = bnd.cart_rb_hard_center().lower();
  DataAtStep(CART_RB_CENTER_HARD_UPPER) = bnd.cart_rb_hard_center().upper();

  DataAtStep(STREAM_WALL_REAR_SOFT_LOWER) =
      bnd.stream_wall_inflation_rear().lower();
  DataAtStep(STREAM_WALL_REAR_SOFT_UPPER) =
      bnd.stream_wall_inflation_rear().upper();
  DataAtStep(STREAM_WALL_REAR_HARD_LOWER) = bnd.stream_wall_hard_rear().lower();
  DataAtStep(STREAM_WALL_REAR_HARD_UPPER) = bnd.stream_wall_hard_rear().upper();
  DataAtStep(STREAM_WALL_FRONT_SOFT_LOWER) =
      bnd.stream_wall_inflation_front().lower();
  DataAtStep(STREAM_WALL_FRONT_SOFT_UPPER) =
      bnd.stream_wall_inflation_front().upper();
  DataAtStep(STREAM_WALL_FRONT_HARD_LOWER) =
      bnd.stream_wall_hard_front().lower();
  DataAtStep(STREAM_WALL_FRONT_HARD_UPPER) =
      bnd.stream_wall_hard_front().upper();
  DataAtStep(STREAM_WALL_CENTER_SOFT_LOWER) =
      bnd.stream_wall_inflation_center().lower();
  DataAtStep(STREAM_WALL_CENTER_SOFT_UPPER) =
      bnd.stream_wall_inflation_center().upper();
  DataAtStep(STREAM_WALL_CENTER_HARD_LOWER) =
      bnd.stream_wall_hard_center().lower();
  DataAtStep(STREAM_WALL_CENTER_HARD_UPPER) =
      bnd.stream_wall_hard_center().upper();

  DataAtStep(CART_LB_REAR_SOFT_LOWER) = bnd.cart_lb_rear().lower();
  DataAtStep(CART_LB_REAR_SOFT_UPPER) = bnd.cart_lb_rear().upper();
  DataAtStep(CART_LB_CENTER_SOFT_LOWER) = bnd.cart_lb_center().lower();
  DataAtStep(CART_LB_CENTER_SOFT_UPPER) = bnd.cart_lb_center().upper();
  DataAtStep(CART_LB_FRONT_SOFT_LOWER) = bnd.cart_lb_front().lower();
  DataAtStep(CART_LB_FRONT_SOFT_UPPER) = bnd.cart_lb_front().upper();
}

void ILQRPathPlannerProblem::PybindTest(
    const ilqr_solver::CilqrConfig &cilqr_config, size_t i) const {
  auto min_index = std::min(cilqr_config.min_constraint_index(),
                            cilqr_config.max_constraint_index());
  auto max_index = std::max(cilqr_config.min_constraint_index(),
                            cilqr_config.max_constraint_index());
  if (i >= static_cast<size_t>(min_index) &&
      i <= static_cast<size_t>(max_index)) {
    auto DataAtStep = [this, i](StepDataIndex index) -> double & {
      return *data_->Mutable(index, i);
    };
    DataAtStep(ACCEL_CONSTRAINT_LOWER) = cilqr_config.target_accel_value();
    DataAtStep(ACCEL_CONSTRAINT_UPPER) = cilqr_config.target_accel_value();
  }
}

void ILQRPathPlannerProblem::UpdateCostConfig(
    const npp::pnc::ilqr::LatMotionPlanInput &input) {
  const auto &init_state = input.ppp_input().init_state();
  for (size_t i = 0; i < Horizon() + 1; i++) {
    FillRefLineInfoAndCommonTerms(
        input.ppp_output().ref_lines(i), init_state.v(),
        input.ppp_input().agent_params().wheel_base(), i);
    FillBoundInfo(input.micro_decider_output().dec(i), i);
    *(*data_).Mutable(TERMINAL_FLAG, i) = 0;
  }
  FillFunctionSafetyParams(input);
  FillCommonTuningParams(input);
  FillHardBoundScaleAndShiftRefCenteringCost(input);
  FillExtraContext();
  // terminal settings
  *(*data_).Mutable(TERMINAL_FLAG, Horizon()) = 1;
}

void ILQRPathPlannerProblem::FillFunctionSafetyParams(
    const npp::pnc::ilqr::LatMotionPlanInput &input) const {
  const auto &function_safety_params =
      input.ppp_input().functional_safety_params();
  ILqrLatModel::v_function_safety =
      std::vector<double>(function_safety_params.v_function_safety().begin(),
                          function_safety_params.v_function_safety().end());
  ILqrLatModel::delta_bound_function_safety = std::vector<double>(
      function_safety_params.delta_bound_function_safety().begin(),
      function_safety_params.delta_bound_function_safety().end());
  ILqrLatModel::omega_bound_function_safety = std::vector<double>(
      function_safety_params.omega_bound_function_safety().begin(),
      function_safety_params.omega_bound_function_safety().end());
  ILqrLatModel::snap_soft_bound =
      std::vector<double>(function_safety_params.snap_soft_bound().begin(),
                          function_safety_params.snap_soft_bound().end());
}

void ILQRPathPlannerProblem::FillCommonTuningParams(
    const npp::pnc::ilqr::LatMotionPlanInput &input) const {
  double init_v = input.ppp_input().init_state().v();
  const auto &params = input.ppp_input().ilqr_params();
  const auto &function_safety_params =
      input.ppp_input().functional_safety_params();
  for (size_t i = 0; i < Horizon() + 1; i++) {
    auto DataAtStep = [this, i](StepDataIndex index) -> double & {
      return *data_->Mutable(index, i);
    };
    DataAtStep(REF_CENTERING_SCALE) = params.ref_centering_scale();
  }

  auto FixData = [this](FixedDataIndex index) -> double & {
    return *data_->Mutable(index);
  };
  const double kWheelAngleLimit =
      function_safety_params.delta_bound_function_safety().Get(0) * kDeg2rad;
  const double steer_angle_to_front_angle_radians =
      input.ppp_input().agent_params().steer_angle_to_front_angle_radians();
  FixData(DESIRE_HEADING_SCALE) = params.desire_heading_scale();
  FixData(LAT_ACCEL_SCALE) = params.lat_accel_scale();
  FixData(CONSTANT_HEADING_SCALE) = params.constant_heading_scale();
  FixData(LAT_JERK_SCALE) = params.lat_jerk_scale();
  FixData(DELTA_SCALE) = params.delta_scale();
  FixData(OMEGA_SCALE) = params.omega_scale();
  FixData(SNAP_SCALE) = params.snap_scale();
  FixData(INIT_SNAP_SCALE) = params.snap_scale();
  FixData(WHEEL_ANGLE_LOWER) = -kWheelAngleLimit;
  FixData(WHEEL_ANGLE_UPPER) = kWheelAngleLimit;
  // update vel_ratio info
  FixData(ACCEL_LIMIT) = input.ppp_input().lat_iqlr_bounds().accel_limit();
  FixData(JERK_LIMIT) = input.ppp_input().lat_iqlr_bounds().jerk_limit();
  FixData(SNAP_LIMIT) =
      ILqrLatModel::SoftSnapLimit(init_v) * steer_angle_to_front_angle_radians;
  FixData(CONSTRAINT_MAX_INDEX) =
      static_cast<size_t>(Horizon() * kConstraintHorizonRatio);
}

void ILQRPathPlannerProblem::FillExtraContext() const {
  auto fix_data = [this](FixedDataIndex index) -> double & {
    return *data_->Mutable(index);
  };
  fix_data(DT) = GetTimeStep();
}

void ILQRPathPlannerProblem::RestHardBoundScaleAndShiftRefCenteringCost() {
  auto fix_data = [this](FixedDataIndex index) -> double & {
    return *data_->Mutable(index);
  };
  fix_data(JERK_LIMIT_RATIO) = 0.0;
  fix_data(ACCEL_LIMIT_RATIO) = 0.0;
  fix_data(CART_RB_HARD_SCALE) = 0.0;
  fix_data(CART_RB_SOFT_SCALE) = 0.0;
  fix_data(STREAM_WALL_HARD_SCALE) = 0.0;
  fix_data(STREAM_WALL_SOFT_SCALE) = 0.0;
  fix_data(CART_LB_SOFT_SCALE) = 0.0;
  fix_data(WHEEL_ANGLE_BOUND_SCALE) = 0.0;
  fix_data(CONVERT_REF_CENTERING_COST) = 1.0;
}

void ILQRPathPlannerProblem::GenDenseTraj(
    const double wheel_base, const StateVec &xs, const ControlVec &us,
    std::vector<npp::pnc::State> *dense_xs) const {
  if (dense_xs == nullptr) {
    return;
  }
  double delta_bound =
      ILqrLatModel::FunctionSafetyDeltaLimit((*data_)(INIT_VEL));
  double omega_bound =
      ILqrLatModel::FunctionSafetyOmegaLimit((*data_)(INIT_VEL));
  auto Transfer = [=](State state, npp::pnc::State *res) {
    res->set_x(state(StateId::X));
    res->set_y(state(StateId::Y));
    res->set_theta(state(StateId::THETA));
    res->set_delta(
        std::fmin(std::fmax(state(StateId::DELTA), -delta_bound), delta_bound));
    res->set_gamma(
        std::fmin(std::fmax(state(StateId::GAMMA), -omega_bound), omega_bound));
    res->set_curvature(std::tan(state(StateId::DELTA)) / wheel_base);
  };
  // 原本是21个采样点+20段*每段9个加密点共201，加入5个点的buffer防止外插，共206个点
  const size_t T = Horizon();

  dense_xs->resize(T * kDenseCnt + 1 + kDenseBuffer);
  int idx = 0;
  for (size_t i = 0; i < T; ++i) {
    Transfer(xs[i], &(*dense_xs)[idx++]);
    for (size_t j = 1; j < kDenseCnt; ++j) {
      Transfer(
          lat_model_->UpdateDynamicsDenseStep(xs[i], us[i], i, j, kDenseCnt),
          &(*dense_xs)[idx++]);
    }
  }
  Transfer(xs[T], &(*dense_xs)[idx++]);
  for (int i = 1; i < kDenseBuffer + 1; ++i) {
    Transfer(
        lat_model_->UpdateDynamicsDenseStep(xs[T], us[T - 1], T, i, kDenseCnt),
        &(*dense_xs)[idx++]);
  }
}

void ILQRPathPlannerProblem::TransferOutputToProtoData(
    const OutputInfo &out_info, const double wheel_base,
    npp::pnc::ilqr::Output *out_proto) const {
  ClearAndResizeOutput(out_proto);
  const auto &result_xk = out_info.x_vec;
  const auto &result_uk = out_info.u_vec;
  for (size_t j = 0; j < Horizon() + 1; ++j) {
    auto state = out_proto->add_states();
    state->set_t(static_cast<double>(j) * GetTimeStep());
    state->set_x((result_xk)[j][X]);
    state->set_y((result_xk)[j][Y]);
    state->set_theta((result_xk)[j][THETA]);
    state->set_delta((result_xk)[j][DELTA]);
    state->set_gamma((result_xk)[j][GAMMA]);
    state->set_v((*data_)(INIT_VEL));
    state->set_curvature(std::tan((result_xk)[j][DELTA]) / wheel_base);
    if (j == Horizon()) {
      out_proto->add_controls(
          out_proto->controls()[out_proto->controls_size() - 1]);
    } else {
      out_proto->add_controls((result_uk)[j][SNAP]);
    }
    state->mutable_frenet_state()->set_l((*data_)(OFFSET, j));
    state->mutable_frenet_state()->set_s((*data_)(SAMPLE_S, j));
  }
  return;
}

int ILQRPathPlannerProblem::UpdateOutput(
    const std::vector<OutputInfo> &output_candidates, const double wheel_base,
    npp::pnc::ilqr::LatMotionPlanOutput *output) const {
  auto *out_candidates_pb = output->mutable_output_candidates();
  out_candidates_pb->clear_final_output_type();
  out_candidates_pb->clear_candidates();

  for (int i = 0; i < SolutionId::kSolutionSize; i++) {
    auto &cur_output = output_candidates[i];
    if (!cur_output.is_valid_traj) {
      break;
    }
    auto *cur_candidates_proto = out_candidates_pb->add_candidates();
    TransferOutputToProtoData(cur_output, wheel_base, cur_candidates_proto);
  }
  const int init_seed_index = ilqr_core_ptr_->GetInitSeedIndex();
  out_candidates_pb->set_init_seed_type(
      static_cast<npp::pnc::ilqr::CandidateType>(init_seed_index));

  for (int i = SolutionId::kSolutionSize - 1; i >= 0; i--) {
    // never use turn circle solution
    if (output_candidates[i].is_turn_circle ||
        !output_candidates[i].is_valid_traj || (i == SolutionId::kBackUpInit)) {
      continue;
    }
    // never use backup init solution
    out_candidates_pb->set_final_output_type(
        static_cast<npp::pnc::ilqr::CandidateType>(i));
    out_candidates_pb->set_is_final_traj_rb_collision(
        output_candidates[i].is_rb_collision);
    break;
  }
  output->mutable_final_output()->CopyFrom(
      out_candidates_pb->candidates(out_candidates_pb->final_output_type()));
  return static_cast<int>(out_candidates_pb->final_output_type());
}

void ILQRPathPlannerProblem::UpdateInitState(const npp::pnc::State &init_state,
                                             State *x) const {
  *x << init_state.x(), init_state.y(), init_state.theta(), init_state.delta(),
      init_state.gamma();
}

void ILQRPathPlannerProblem::ClearCandidatesOuput(
    std::vector<OutputInfo> *output_candidates) const {
  output_candidates->clear();
  output_candidates->resize(SolutionId::kSolutionSize);
}

void ILQRPathPlannerProblem::GetOutputFromControl(const ControlVec &u_vec,
                                                  const double curv_factor,
                                                  OutputInfo *output) const {
  StateVec x_vec{};
  ResizeAndResetEigenVec(x_vec, Horizon() + 1, GetStateSize());
  x_vec[0] = x0_[0];
  for (size_t i = 0; i < u_vec.size(); i++) {
    x_vec[i + 1] = ILqrLatModel::UpdateDynamicsWithDt(
        x_vec[i], u_vec[i], (*data_)(INIT_VEL), curv_factor, GetTimeStep());
  }

  *output = {x_vec, u_vec, PathCircleChecker(x_vec), RbCollisionChecker(x_vec),
             true};
}

void ILQRPathPlannerProblem::UpdateWarmStartData(
    const npp::pnc::ilqr::LatMotionPlanInput &input) {
  const double curv_factor =
      1.0 / input.ppp_input().agent_params().wheel_base();
  ControlVec warm_start_u{};
  ResizeAndResetEigenVec(warm_start_u, Horizon(), GetInputSize());

  const auto &warm_start_data = input.warm_start_data();
  const double pre_dt_ratio = warm_start_data.pre_dt_ratio();
  for (int i = 0; i < warm_start_data.controls_size() - 1; i++) {
    warm_start_u[i](0) = warm_start_data.controls(i) * pre_dt_ratio;
  }

  GetOutputFromControl(warm_start_u, curv_factor,
                       &output_candidates_[SolutionId::kPrePlan]);
  GenInitSeed(input, &output_candidates_[SolutionId::kForwardSim]);
  ilqr_core_ptr_->UpdateWarmStart(
      {output_candidates_[SolutionId::kForwardSim].u_vec, warm_start_u});
}

void ILQRPathPlannerProblem::ClearAndResizeOutput(
    npp::pnc::ilqr::Output *output) const {
  auto states = output->mutable_states();
  states->Clear();
  states->Reserve(Horizon() + 1);
  auto controls = output->mutable_controls();
  controls->Clear();
  controls->Reserve(Horizon() + 1);
}

void ILQRPathPlannerProblem::UpdateOutputSolverInfo(
    const ilqr_solver::ILqr::ILqrSolverInfo &core_solver_info,
    npp::pnc::ilqr::LatMotionPlanOutput *output) {
  output->clear_ilqr_info();
  output->clear_data_iteration();
  auto *ilqr_info = output->mutable_ilqr_info();
  ilqr_info->set_solver_condition(core_solver_info.solver_condition);
  ilqr_info->set_cost_size(core_solver_info.cost_size);
  ilqr_info->set_iter_count(core_solver_info.iter_count);
  ilqr_info->set_init_cost(core_solver_info.init_cost);
  ilqr_info->set_total_time_ms(core_solver_info.total_time_ms);
  ilqr_info->set_t_compute_deriv_ms(core_solver_info.t_compute_deriv_ms);
  ilqr_info->set_t_backward_pass_ms(core_solver_info.t_backward_pass_ms);
  ilqr_info->set_t_forward_pass_ms(core_solver_info.t_forward_pass_ms);

  if (core_solver_info.iter_count > 0) {
    const size_t start_index = ilqr_solver_config_ptr_->is_debug_mode()
                                   ? 0
                                   : core_solver_info.iter_count - 1;

    for (size_t i = start_index; i < core_solver_info.iter_count; i++) {
      const auto &iteration_info = core_solver_info.iteration_info_vec[i];
      auto *data = output->add_data_iteration();
      data->set_linesearch_success(iteration_info.linesearch_success);
      data->set_cur_lambda(iteration_info.lambda);
      data->set_cost(iteration_info.cost);
      data->set_dcost(iteration_info.dcost);
      data->set_expect(iteration_info.expect);

      for (const auto &cost_map_cpp : iteration_info.cost_map) {
        auto *cost_map = data->add_detail_cost();
        for (const auto &step : cost_map_cpp) {
          cost_map->add_value(step);
        }
      }
    }
  }
}

void ILQRPathPlannerProblem::Solve(OutputInfo *output) const {
  ilqr_core_ptr_->Solve(x0_[0]);

  *output = {*ilqr_core_ptr_->GetStateResultPtr(),
             *ilqr_core_ptr_->GetControlResultPtr(),
             PathCircleChecker(*ilqr_core_ptr_->GetStateResultPtr()),
             RbCollisionChecker(*ilqr_core_ptr_->GetStateResultPtr()), true};
}

void ILQRPathPlannerProblem::UpdateCilqrSolverInfo(
    const ilqr_solver::ILqr::CilqrSolve &core_cilqr_info,
    npp::pnc::ilqr::LatMotionPlanOutput *output) {
  output->clear_cilqr_info();
  auto *cilqr_info = output->mutable_cilqr_info();
  for (const auto &iter : core_cilqr_info.cilqr_solve) {
    auto data = cilqr_info->add_cilqr_solve();
    for (const auto &itm : iter.cilqr_iteration) {
      auto iteration = data->add_cilqr_iteration();
      for (const auto &it : itm.cilqr_info) {
        auto info = iteration->add_cilqr_info();
        info->set_lambdas(it.lambda);
        info->set_penalty(it.penalty);
      }
    }
    for (const auto &itm : iter.max_violation) {
      data->add_max_violation(itm);
    }
  }
}

}  // namespace lat_ilqr
