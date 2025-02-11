#pragma once

#include <vector>

#include "df_planner/traj_container.h"
#include "df_planner/traj_utils.h"
#include "df_planner/constants.h"
#include "df_planner/lbfgs_global.hpp"
#include "df_planner/lbfgs.h"

namespace plan_manage {

 using namespace std;


  class PolyTrajOptimizer
  {

  private:
    int traj_seg_num_;

    int drone_id_;
    int traj_resolution_; // number of distinctive constrain points each piece
    int destraj_resolution_; //number of distinctive constrain points of the first and last piece (should be more dense!
    int variable_num_;     // optimization variables
    int iter_num_;         // iteration of the solver
    double min_ellip_dist2_; // min trajectory distance in surround


    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP,
      STOP_FOR_REBOUND,
      STOP_FOR_ERROR
    } force_stop_type_;

    /*lbfgs param*/
    int memsize;
    int past;
    double delta;
    double mini_T = 0.05;


    // Each col of cfgHs denotes a facet (outter_normal^T,point^T)^T
    Eigen::Matrix<double, 2, 2> B_h;
    double epis;
    std::vector<int> singul_container;
    std::vector<Eigen::MatrixXd> iniState_container;
    std::vector<Eigen::MatrixXd> finState_container;
    std::vector<std::vector<Eigen::MatrixXd>> cfgHs_container;
    std::vector<double> gearVel_container;
    int trajnum;
    /*debug*/
    Eigen::MatrixXd ctrl_points_;
    std::vector<Eigen::Vector2d> cos_points;
    std::vector<Eigen::Vector2d> key_points;
    std::vector<Eigen::MatrixXd> debug_hPolys;
    std::vector<traj_utils::MinJerkOpt> jerkOpt_container;
    std::vector<int> piece_num_container;

  public:
    
    PolyTrajOptimizer() {}
    ~PolyTrajOptimizer() {}

    /* optimization parameters */
    double wei_obs_;                         // obstacle weight
    double wei_surround_;                       // surround weight
    double wei_feas_;                        // feasibility weight
    double wei_sqrvar_;                      // squared variance weight
    double wei_time_;                        // time weight
    double surround_clearance_; // safe distance
    
    double half_margin;                        // safe margin
    traj_utils::VehicleParam veh_param_;          
    double t_now_;
    double L_;
    std::vector<Eigen::Vector2d> lz_set_;
    std::vector<Eigen::Vector2d> vec_le_, vec_lo_;
    int number_of_hyperplanes_of_ego_car_, number_of_hyperplanes_of_surround_car_;


    /*new added param*/
    double max_forward_vel, max_backward_vel, max_forward_cur, max_backward_cur;
    double max_forward_acc, max_backward_acc, max_phidot_, max_latacc_;       // dynamic limits
    double non_sinv = 0.24;
    bool GearOpt = true;

    /* set variables */
    void setParam();
    ros::Publisher debug_pub,debug_pub1,debug_galaxy_poly_pub_,debug_key_points_pub;
    void displayPoints();
    void displayCosPoints();
    void displayBugPoly();
    void displayKeyPoints();
    // void setEnvironment(const GridMap::Ptr &map);
    void setControlPoints(const Eigen::MatrixXd &points);
    // void setSurroundTrajs(plan_utils::SurroundTrajData *surround_trajs_ptr);
    void setDroneId(const int drone_id);

    /* helper functions */
    inline const std::vector<traj_utils::MinJerkOpt> *getMinJerkOptPtr(void) { return &jerkOpt_container; }
    inline int get_traj_resolution_() { return traj_resolution_; };
    inline int get_destraj_resolution_() { return destraj_resolution_; };
    inline double getsurroundClearance(void) { return surround_clearance_; }

    /* main planning API */
    bool OptimizeTrajectory(const std::vector<Eigen::MatrixXd> &iniStates, const std::vector<Eigen::MatrixXd> &finStates,
                            std::vector<Eigen::MatrixXd> &initInnerPts, const Eigen::VectorXd &initTs,
                            std::vector<std::vector<Eigen::MatrixXd>> &hPoly_container,std::vector<int> singuls,double now = ros::Time::now().toSec(),double help_eps = 1.0e-4);


    double log_sum_exp(double alpha, Eigen::VectorXd &all_dists, double &exp_sum);


  private:
    /* callbacks by the L-BFGS optimizer */
    static double costFunctionCallback(void *func_data, const Eigen::VectorXd &x, Eigen::VectorXd &grad);

    static int earlyExitCallback(void *func_data, const double *x, const double *g,
                                 const double fx, const double xnorm, const double gnorm,
                                 const double step, int n, int k, int ls);

    /* mappings between real world time and unconstrained virtual time */
    template <typename EIGENVEC>
    void RealT2VirtualT(const Eigen::VectorXd &RT, EIGENVEC &VT);

    template <typename EIGENVEC>
    void VirtualT2RealT(const EIGENVEC &VT, Eigen::VectorXd &RT);

    template <typename EIGENVEC, typename EIGENVECGD>
    void VirtualTGradCost(const Eigen::VectorXd &RT, const EIGENVEC &VT,
                          const Eigen::VectorXd &gdRT, EIGENVECGD &gdVT,
                          double &costT);
    void VirtualTGradCost(const double &RT, const double & VT, const double & gdRT, double & gdVT, double & costT);

  
    void addPVAGradCost2CT(Eigen::VectorXd &costs, const int trajid, const double trajtime);

    bool obstacleGradCostP(const int i_dp,
                           const Eigen::Vector2d &p,
                           Eigen::Vector2d &gradp,
                           double &costp);

    bool surroundGradCostP(const int i_dp,
                        const double t,
                        const Eigen::Vector2d &p,
                        const Eigen::Vector2d &v,
                        Eigen::Vector2d &gradp,
                        double &gradt,
                        double &grad_prev_t,
                        double &costp);

    double dynamicObsGradCostP( const double &omg,
                                const double &step,
                                const double &t,               // current absolute time
                                const Eigen::Matrix<double, 6, 1> &beta0,
                                const Eigen::Matrix<double, 6, 1> &beta1,
                                const double & gama, 
                                const int& pieceid,
                                const int& trajres,
                                const Eigen::Vector2d &sigma, // the rear model
                                const Eigen::Vector2d &dsigma,
                                const Eigen::Vector2d &ddsigma,
                                const Eigen::Matrix2d &ego_R,
                                const Eigen::Matrix2d &help_R,
                                const int &trajid,const double& trajtime);
    double debugGradCheck(const int i_dp, // index of constraint point
                                         double t, // current absolute time
                                        Eigen::Vector2d sigma, // the rear model 
                                        Eigen::Vector2d dsigma,
                                        Eigen::Vector2d ddsigma,                                                                             
                                        const int trajid, const int sur_id, double res_t,Eigen::Matrix<double, 6, 2> c
                                        ,int i ,int j,double omg,double step,double wei_surround_,int K);
    bool dynamicObsCosCheck(double t_now, const Eigen::MatrixXd iniStates,  int trajid, int sur_id);

    void distanceSqrVarianceWithGradCost2p(const Eigen::MatrixXd &ps,
                                           Eigen::MatrixXd &gdp,
                                           double &var);

    inline bool extractVs(const std::vector<Eigen::MatrixXd> &hPs,
                          std::vector<Eigen::MatrixXd> &vPs) const
    {
        // const int M = hPs.size() - 1;

        // vPs.clear();
        // vPs.reserve(2 * M + 1);

        // int nv;
        // Eigen::MatrixXd curIH, curIV, curIOB;
        // for (int i = 0; i < M; i++)
        // {
        //     if (!geoutils::enumerateVs(hPs[i], curIV))
        //     {
        //         return false;
        //     }
        //     nv = curIV.cols();
        //     curIOB.resize(3, nv);
        //     curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        //     vPs.push_back(curIOB);

        //     curIH.resize(6, hPs[i].cols() + hPs[i + 1].cols());
        //     curIH << hPs[i], hPs[i + 1];
        //     if (!geoutils::enumerateVs(curIH, curIV))
        //     {
        //         return false;
        //     }
        //     nv = curIV.cols();
        //     curIOB.resize(3, nv);
        //     curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        //     vPs.push_back(curIOB);
        // }

        // if (!geoutils::enumerateVs(hPs.back(), curIV))
        // {
        //     return false;
        // }
        // nv = curIV.cols();
        // curIOB.resize(3, nv);
        // curIOB << curIV.col(0), curIV.rightCols(nv - 1).colwise() - curIV.col(0);
        // vPs.push_back(curIOB);

        return true;
    }

    std::vector<Eigen::MatrixXd> debug_P;
    std::vector<Eigen::VectorXd> debug_T;
    std::vector<double> debug_cost;

    void updatePT(const Eigen::MatrixXd &inPs,const Eigen::VectorXd &ts, double cost);
    void getBoundPts(Eigen::Vector2d &position, double angle, std::vector<Eigen::Vector2d> &BoundVertices);
    void positiveSmoothedL1(const double &x, double &f, double &df);
    void positiveSmoothedL3(const double &x, double &f, double &df);

  public:
    typedef unique_ptr<PolyTrajOptimizer> Ptr;
    vector<CartesianState> GetResult(double gap) {
      // 优化结果密集采样输出
      vector<CartesianState> final_path;
      for(auto& jerkOpt : jerkOpt_container) {
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
        // const auto &piece_positions = final_traj.getPositions();
        // for (int j = 0; j < piece_positions.cols(); j++) {
        //   if (j == 0) {
        //     cout << "起点：" << piece_positions(0, j) << ", " << piece_positions(1, j) << endl;
        //   } else if (j == piece_positions.cols() - 1) {
        //     cout << "终点：" << piece_positions(0, j) << ", " << piece_positions(1, j) << endl;
        //   } else {
        //     cout << "中间点：" << piece_positions(0, j) << ", " << piece_positions(1, j) << endl;
        //   }
        // }
      }
      return final_path;
    }

    void GetResult(double gap, GlobalPath& refline) {
      refline = GlobalPath();
      // 优化结果密集采样输出
      for(auto& jerkOpt : jerkOpt_container) {
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
          piece.pushState(relative_t, refline);
          // if(count < 5)
            // cout << "df_state: " << state.x << ", " << state.y << ", " << state.speed << ", " << state.theta << ", " << state.acc << ", " << state.kappa << endl;
        }
      }
    }

  };


}  // namespace plan_manage
