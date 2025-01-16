#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cassert>
#include <array>
#include <algorithm>
#include <iterator>
#include <queue>
#include <Eigen/Geometry>

#include "robot_var.h"
#include "hybird_force_position_controller.h"
#include "ft_sensor.h"
#include "file_loader.h"
#include "modern_robotics.h"

extern FILE *pFileData;
extern FILE *pFileData1;
extern RobotModel_GP50 GP50;
extern ati::FTSensor ftsensor_;

double traj_buffer[4000000][6];
double force_buffer[4000000][6];
double contact_frame_buffer[4000000][9];


double calculateVariance(const Eigen::VectorXd& data) {
    double mean = data.mean();
    Eigen::ArrayXd squaredDiff = (data.array() - mean).array().square();
    double variance = squaredDiff.mean();
    return variance;
}

HybridForcePositionController::HybridForcePositionController()
{
	count = 0;
	cycle_count = 0;
	stable_count = 0;
	stable_frame = 0;
	maximum_force = 20.0;
	minimum_force = 5.0;
	minimum_belt_rotate_force_variance_in_contact = 0.0;
	raw_max_force = 150.0;
	itp_num = 200;
	valid_frc_fdbk_range = 10.0;
	delta_joint_limit = 1e-4;
	close_xyz_thres = 1e-3;
	P2P_skip_frame = 1;
	base_force = 7.0;
	qd_delta_limit = 1.0;
	new_point = true;
	cout << "HybridForcePositionController is being created" << endl;
}

HybridForcePositionController::~HybridForcePositionController()
{

	cout << "HybridForcePositionController is being deleted" << endl;
	robot_hw_ptr->setRobotPowerOff();
}

bool HybridForcePositionController::init(const HybridForcePosCtr_Config *user_joint_pos_cfg, HardwareInterface *user_robot_hw_ptr)
{

	cout << "control check 1" << endl;
	if (NULL == user_joint_pos_cfg && NULL == user_robot_hw_ptr)
	{
		cout << "HybridForcePositionController init Error: invalid JointPosCfg" << endl;
		return false;
	}

	memcpy(&joint_pos_cfg, user_joint_pos_cfg, sizeof(HybridForcePosCtr_Config));
	cout << "control check 2" << endl;
	robot_hw_ptr = user_robot_hw_ptr;
	robot_hw_ptr->resetAxisState();
	robot_hw_ptr->setAxisEnable(joint_pos_cfg.joint_enable);
	robot_hw_ptr->setAxisMode(AXIS_POSITION_MODE);
	robot_hw_ptr->getJointPos(pos_fdb);
	cout << "control check 3" << endl;

	if (joint_pos_cfg.movement_mode == HYBIRD_POSITION_FORCE_CONTROL)
	{
		
		traj_num = loadPoint(traj_buffer);
		force_num = loadForceTraj(force_buffer);
		contact_frame_num = loadContactFrame(contact_frame_buffer);
		cout<<"contact_frame_num is: "<<contact_frame_num<<endl;
		assert(traj_num == force_num &&  "traj num is equal to force num");
		assert(traj_num == contact_frame_num &&  "traj num is equal to contact_frame num");


		double* minPtr = min_element(&force_buffer[0][0], &force_buffer[traj_num - 1][6 - 1] + 1);
    	double minValue = *minPtr;
		assert(minValue >= 0. &&  "All reference force should be non-negative");
		cout << "trajectory and force waypoint number is" << traj_num << endl;
	}
	else if (joint_pos_cfg.movement_mode == Tool_Calibration)
	{
		traj_num = loadPoint(traj_buffer);
		cout << "trajectory number is " << traj_num << endl;
	}


	if (ftsensor_.init("192.168.1.1"))
	{
		ftsensor_.getRDTRate();
	}


	F_C_des << 0,0,10,0,0,0;
	Selection(2, 2) = 1.0;
	for (size_t i = 0; i < 6; ++i)
	{

		Damping_inv(i, i) = 1.0/200.0;
	}		 

	return true;
}

bool HybridForcePositionController::execute()
{
	double pos_des[JOINT_NUM] = {0};
	static double pos_des_last[JOINT_NUM] = {0};
	double trq_des[JOINT_NUM] = {0};
	int pulse[DOF] = {0};
	int i = 0;
	static int count_samepose = 0;
	static double delta_joint_pos_sum[7] = {0};
	Mat67 J_BC;
	Vec6 V_b;
	double delta_joint_force_num[7] = {0};
	double delta_joint_pos[7] = {0};
	double contact_frame_current[16] = {0};
	double force_map = -0.30;
	cycle_count++;
	static int F2P_count = 0;
	static int interpolate_num =0;
	float measurements[6];
	double dt = 0.004;
	static bool singularity_itp = false;

	static Eigen::VectorXd force_fbk_data(10);

	int state = 0;
	Mat6 J_BT;
	Eigen::VectorXd solution;

	Eigen::VectorXd pos_fdb_vec(6);
	Eigen::VectorXd pos_dsr_vec(6);
	Eigen::VectorXd pos_last_des_vec(6);
	robot_hw_ptr->getJointPos(pos_fdb);


	pos_fdb_vec << pos_fdb[0],pos_fdb[1],pos_fdb[2],pos_fdb[3],pos_fdb[4],pos_fdb[5];
	
	Eigen::MatrixXd X_BT = GP50.calFK(pos_fdb_vec,'T');

	Eigen::VectorXd F_S_fbk_raw = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd F_S_fbk = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd F_T_fbk = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd F_C_out = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd qd_delta;
	Eigen::VectorXd q_diff(6);
	static Eigen::MatrixXd X_BT_record;
	static bool first_time = false;

	if (X_BT(2,3) >= 3.0)
	{
		printf("the z position of tool larger than threshold");
		return false;
	}

	if (joint_pos_cfg.movement_mode == HYBIRD_POSITION_FORCE_CONTROL)
	{
		bool bSuccess = false;
		bSuccess = ftsensor_.getMeasurements(measurements);
		if (bSuccess == false)
		{
			printf("Torce torque sensor read failed" );
			return false;
		}
		F_S_fbk_raw << (double)measurements[0],(double)measurements[1],(double)measurements[2],(double)measurements[3],(double)measurements[4],(double)measurements[5];
		for (int i = 0; i < 6; i++)
		{
			if (measurements[i] >= raw_max_force || measurements[i] <= -raw_max_force)
			{
				printf("Torce torque sensor %d achieve the maximum range %f",i, measurements[i] );
				return false;
			}
		}
		for (int i = 0; i < 9; i++)
		{
			contact_frame_current[i] = contact_frame_buffer[min(count, traj_num)][i];
		}
		R_BC << contact_frame_current[0],contact_frame_current[1],contact_frame_current[2],
				contact_frame_current[3],contact_frame_current[4],contact_frame_current[5],
				contact_frame_current[6],contact_frame_current[7],contact_frame_current[8];
		F_S_fbk = GP50.calF_ext_S(pos_fdb_vec,F_S_fbk_raw);
		Eigen::MatrixXd AD = Adjoint_PR(GP50.X_ST);
		F_T_fbk = Adjoint_PR(GP50.X_ST).transpose() * F_S_fbk;
		Eigen::Matrix3d R_BT = X_BT.block<3, 3>(0, 0);
		Eigen::Matrix3d R_CT = R_BC.transpose() * R_BT;
		F_C_fbk = Rot66(R_CT) * F_T_fbk;
		double variance = minimum_belt_rotate_force_variance_in_contact + 0.1;
		if(cycle_count <= 9)
		{
			force_fbk_data(cycle_count) = F_C_fbk[2];
		}
		else
		{
			for (size_t i = 0; i < 9; ++i)
			{
				force_fbk_data(i) = force_fbk_data(i+1);
			}
			force_fbk_data(9) = F_C_fbk[2];
			variance = calculateVariance(force_fbk_data);

			if (cycle_count % 100 == 0) {
				cout << "variance" << variance << endl;
			}
		}
		/* 	set the desired force to base force */
		for (int i = 0; i < 6; i++)
		{
			if (force_buffer[count][i] > 0.0) {
				F_C_des(i) = base_force;
			} else {
				F_C_des(i) = 0;
			}
			
		}
		/* 	following section computes the 
			joint increments wrt. the desired force */

		F_C_out = computeForcePI(F_C_des,F_C_fbk);


		if (F_C_fbk(2)<=F_C_des(2) && first_time == false)
		{
			for (size_t i = 0; i < 6; ++i)
			{

				Damping_inv(i, i) = 1.0/200.0;
			}		
		}
		else
		{
			for (size_t i = 0; i < 6; ++i)
			{

				Damping_inv(i, i) = 1.0/200.0;
			}	
			first_time = true;	
		}

		Vb_BC_C_des = -Selection * Damping_inv * F_C_out;
		Vb_BC_B_des = Rot66(R_BC) * Vb_BC_C_des;
		Eigen::MatrixXd Jg_BT = GP50.calGeomJaco(pos_fdb_vec,'T');
		Eigen::MatrixXd Jg_BT_turn = GP50.calGeomJaco(pos_fdb_vec,'T');
		Eigen::MatrixXd tempRows = Jg_BT.topRows(3);
		Eigen::MatrixXd tempRows1 = Jg_BT.bottomRows(3);
		Jg_BT_turn.topRows(3) = tempRows1;
		Jg_BT_turn.bottomRows(3) = tempRows;
		qd_delta = Jg_BT_turn.inverse() * Vb_BC_B_des;
		qd_delta_sum = qd_delta_sum + qd_delta * dt;
		for (int i = 0; i < 6; i++) {
			if (qd_delta[i] > qd_delta_limit) {
				force_buffer[count][2] = 0.;
				break;
			}
		}

		for (int i = 0; i < 6; i++) {
					pos_des[i] = traj_buffer[min(count, traj_num)][i] + qd_delta_sum(i);
				}
		
		// next step
		count++;
	}
	
	else if (joint_pos_cfg.movement_mode == Tool_Calibration)
	{

		if (count <= traj_num)
		{
			for (int i = 0; i < 6; i++)
			{
				pos_des[i] = traj_buffer[count][i];
				pos_des[0] = 1.22;
			}
		}
		else
		{
			for (int i = 0; i < 6; i++)
			{
				pos_des[i] = traj_buffer[traj_num][i];
				pos_des[0] = 1.22;
			}
		}
		

		if (pos_des_last[3] == pos_des[3] && pos_des_last[4] == pos_des[4] && pos_des_last[5] == pos_des[5])
		{
			count_samepose++;
			if (count_samepose >= 600)
			{
				printf("record the FTS!!!!!!!!!!!!");
				ftsensor_.getMeasurements(measurements);
				for(int i=0;i<=50;i++ )
				{
					fprintf(pFileData1, "%d %f %f %f %f %f %f %f %f %f %f %f %f\n", count, measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5],
					pos_fdb[0], pos_fdb[1], pos_fdb[2], pos_fdb[3], pos_fdb[4], pos_fdb[5]);
				}

				count_samepose = 0;
			}
		}
		else
		{
			count_samepose = 0;
		}	
		for (int i = 0; i < 6; i++)
		{
			pos_des_last[i] = pos_des[i];
		}
	}

	for (int i = 0; i < 6; i++)
	{
		pos_des_last[i] = pos_des[i];
	}
	robot_hw_ptr->setJointPosCmd(pos_des);
	fprintf(pFileData, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %d\n", count, F_C_fbk[0], F_C_fbk[1], F_C_fbk[2], F_C_fbk[3], F_C_fbk[4], F_C_fbk[5],
 		pos_fdb[0], pos_fdb[1], pos_fdb[2], pos_fdb[3], pos_fdb[4], pos_fdb[5], 
		pos_des[0], pos_des[1], pos_des[2], pos_des[3], pos_des[4], pos_des[5],
		qd_delta_sum(0), qd_delta_sum(1), qd_delta_sum(2), qd_delta_sum(3), qd_delta_sum(4), qd_delta_sum(5),
		state);


	if (cycle_count%100 == 0)
	{
		cout << "F_S_fbk_raw" << F_S_fbk_raw << endl;
		cout << "F_S_fbk" << F_S_fbk << endl;
		cout << "Damping_inv" << Damping_inv << endl;
		cout << "F_C_fbk" << F_C_fbk << endl;
		cout << "Vb_BC_B_des" << Vb_BC_B_des << endl;
		cout << "F_C_des" << F_C_des << endl;
		cout << "qd_delta_sum" << qd_delta_sum << endl;
		printf("%d",count_samepose);
	}
	return true;
}

Eigen::VectorXd HybridForcePositionController::computeForcePI(const Eigen::VectorXd& F_des,const Eigen::VectorXd& F_fbk)
{
	Eigen::Matrix<double, 6, 6> Kp_force = Eigen::MatrixXd::Zero(6, 6);
	Eigen::Matrix<double, 6, 6> Ki_force = Eigen::MatrixXd::Zero(6, 6);
	Eigen::Matrix<double, 6, 6> Kd_force = Eigen::MatrixXd::Zero(6, 6);
	Eigen::VectorXd F_err = Eigen::VectorXd::Zero(6);
	Eigen::VectorXd F_output = Eigen::VectorXd::Zero(6);
	static Eigen::VectorXd F_err_sum = Eigen::VectorXd::Zero(6);
	double dt = 0.004;
	F_err = F_des - F_fbk;
	for (size_t i = 0; i < 6; ++i)
	{
		Kp_force(i, i) = 0.005;
		Ki_force(i, i) = 0.002;
	}
	F_err_sum = F_err_sum + F_err * dt;
	for (size_t i = 0; i < 6; i++)
	{
		if (F_err_sum(i) >= 30)
		{
			F_err_sum(i) = 30;
		}
		else if (F_err_sum(i) <= -30)
		{
			F_err_sum(i) = -30;
		}
	}
	F_output = Kp_force * F_err + Ki_force * F_err_sum;

	return F_output;
}