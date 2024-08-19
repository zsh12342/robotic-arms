#pragma once

#include <iostream>
#include <string.h>
#include <vector>

#include <casadi/casadi.hpp>
using namespace casadi;

extern "C"
{
    // #include ComPosition_src.c
}

typedef struct {
  double s;
  double xlip_current[4];
  double stance_leg;
  double zH;
  double Ts;
  double Tr;
  double leg_width;
  double Lx_offset;
  double Ly_des;
  double kx;
  double ky;
  double mu;
  double mass;
} planner_to_mpc_data;

typedef struct {
  double time;
  double ufp_wrt_st[2];
  // double ufp_wrt_com[2];
} mpc_to_planner_data;

extern std::string LF_path;
extern std::string RF_path;

class Predict_Fp
{

public:
    // EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // Constructor
    Predict_Fp(std::string & horizon);

    // Destructor
    virtual ~Predict_Fp();

    // Update
    void Update_(const planner_to_mpc_data &to_mpc_update,
                 mpc_to_planner_data &to_planner_update);

private:
    // Related to class loop
    int iter_ = 0;

    // custom_cassie_out struct
    double time_cassie_;
    double s_;
    std::vector<double> x_lip_current_;
    double stance_leg_;
    double zH_;
    double Ts_;
    double Tr_;
    double leg_width_;
    double Lx_offset_;
    double Ly_des_;
    // std::vector<double> ufp_max_;
    // std::vector<double> ufp_min_;
    double kx_new_;
    double ky_new_;
    double mu_new_;
    double mass_new_;

    // Terrain
    double kx_init_;
    std::deque<double> kx_traj_;
    double ky_init_;
    std::deque<double> ky_traj_;
    double mu_init_;
    std::deque<double> mux_traj_;
    std::deque<double> muy_traj_;

    // Cassie parameters
    const double ufp_x_max = 0.8;  // max is 0.8 with ratio = 1
    const double ufp_y_max_ = 0.6;
    const double ufp_y_min_ = 0.1;
    int leg_identity_ = -1;             // leg swap parameter

    // Casadi fp solver info
    const int n_xlip_ = 4;
    const int n_ufp_ = 2;

    int N_steps_;
    int N_xsol_;
    int N_ufpsol_;

    std::string solver_LS_;      
    std::string solver_RS_;      

    casadi::Function f_solver_LS_;
    casadi::Function f_solver_RS_;
    
    std::vector<double> xlip_guess_;
    std::vector<double> ufp_guess_;
};