#ifndef __HYBRID_FORCE_POSITION_CONTROLLER_H_
#define __HYBRID_FORCE_POSITION_CONTROLLER_H_

#include <iostream>
#include <stdlib.h>
#include <math.h>

#include "eigen_types.h"
#include "robot_var.h"
#include "hardware_interface.h"
#include "robot_model_gp50.h"
using namespace std;

typedef enum
{
	HYBIRD_POSITION_FORCE_CONTROL = 1,
	Tool_Calibration = 2,
}HybridForcePosCtr_MovementMode;

typedef struct
{
	int joint_enable[JOINT_NUM];
	int movement_mode;
}HybridForcePosCtr_Config;

class HybridForcePositionController{
   public:
	  HybridForcePositionController();
	  ~HybridForcePositionController();

	  bool init(const HybridForcePosCtr_Config* user_joint_pos_cfg, HardwareInterface* user_robot_hw_ptr);
      bool execute();
	  Eigen::VectorXd computeForcePI(const Eigen::VectorXd& F_des,const Eigen::VectorXd& F_fbk);

   private:
		int count;
		int cycle_count;
		int stable_count;
		int traj_num;
		int force_num;
		int contact_frame_num;
		int itp_num;
		int P2P_skip_frame;
		int stable_frame;
		bool new_point;
		double maximum_force;
		double minimum_force;
		double minimum_belt_rotate_force_variance_in_contact;
		double close_xyz_thres;
		double delta_joint_limit;
		double base_force;
		double raw_max_force;
		double valid_frc_fdbk_range;
		double qd_delta_limit;
		double pos_fdb[JOINT_NUM] = {0};
		double spd_fdb[JOINT_NUM] = {0};
		double trq_fdb[JOINT_NUM] = {0};



		double car_traj_buffer[1000][16];
		Eigen::Matrix<double, 6, 6> Damping_inv = Eigen::Matrix<double, 6, 6>::Zero();
		Eigen::Matrix<double, 6, 6> Selection = Eigen::Matrix<double, 6, 6>::Zero();

		Eigen::VectorXd qd_delta_sum = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd Vb_BC_B_des = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd Vb_BC_C_des = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd F_C_des = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd F_C_fbk = Eigen::VectorXd::Zero(6);
		Eigen::VectorXd F_T_des = Eigen::VectorXd::Zero(6);
		Eigen::Matrix<double, 3, 3> R_BC = Eigen::Matrix<double, 3, 3>::Zero();
		Eigen::Matrix<double, 3, 3> R_TC = Eigen::Matrix<double, 3, 3>::Zero();

		HybridForcePosCtr_Config joint_pos_cfg;
		HardwareInterface* robot_hw_ptr;
};

#endif
