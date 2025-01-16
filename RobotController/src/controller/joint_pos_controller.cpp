#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "robot_var.h"
#include "joint_pos_controller.h"
#include "ft_sensor.h"
#include "file_loader.h"

extern FILE *pFileData;
extern RobotModel_GP50 GP50;
extern ati::FTSensor ftsensor_;

double traj_buffer_pos[4000000][6];
double car_traj_buffer[4000000][16];

JointPositionController::JointPositionController()
{
	count = 0;
	cout << "JointPositionController is being created" << endl;
}

JointPositionController::~JointPositionController()
{
	cout << "JointPositionController is being deleted" << endl;
	robot_hw_ptr->setRobotPowerOff();
}

bool JointPositionController::init(const JointPosCtr_Config *user_joint_pos_cfg, HardwareInterface *user_robot_hw_ptr)
{

	if (NULL == user_joint_pos_cfg && NULL == user_robot_hw_ptr)
	{
		cout << "JointPositionController init Error: invalid JointPosCfg" << endl;
		return false;
	}
	memcpy(&joint_pos_cfg, user_joint_pos_cfg, sizeof(JointPosCtr_Config));
	robot_hw_ptr = user_robot_hw_ptr;
	robot_hw_ptr->resetAxisState();
	robot_hw_ptr->setAxisEnable(joint_pos_cfg.joint_enable);
	robot_hw_ptr->setAxisMode(AXIS_POSITION_MODE);

	if (joint_pos_cfg.movement_mode == JOINT_POSITION_CONTROL)
	{
		traj_num = loadPoint(traj_buffer_pos);
		cout << traj_num << endl;
	}
	else if(joint_pos_cfg.movement_mode == Car_POSITION_CONTROL)
	{
		traj_num = loadCarPoint(car_traj_buffer);
		cout << traj_num << endl;
	}

	return true;
}

bool JointPositionController::execute()
{
	double pos_des[JOINT_NUM] = {0};
	double trq_des[JOINT_NUM] = {0};
	int pulse[DOF] = {0};
	static int writecount = 0;
	int i = 0;
	static double delta_joint_pos_sum[7] = {0};
	Mat67 J_BC;
	Vec6 V_b;
	double delta_joint_force_num[7] = {0};
	double delta_joint_pos[7] = {0};
	double force_map = -0.30;
	count++;
	Mat6 J_BT;
	Eigen::VectorXd solution;
	
	robot_hw_ptr->getJointPos(pos_fdb);

	if (joint_pos_cfg.movement_mode == JOINT_POSITION_CONTROL)
	{

		if(count <= traj_num)
		{
			for (int i = 0; i < 6; i++)
			{
				pos_des[i] = traj_buffer_pos[count][i];
			}
		}
		else
		{
			for (int i = 0; i < 6; i++)
			{
				pos_des[i] = traj_buffer_pos[traj_num][i];
			}
		}
	}


	if (joint_pos_cfg.movement_mode == Car_POSITION_CONTROL)
	{
		double car_pos_des[16] = {0};
		Eigen::VectorXd q_guess(6);
		Eigen::Matrix<double, 4, 4> T_BF;
		Eigen::VectorXd q_des(6);
		if(count <= traj_num)
		{
			for (int i = 0; i < 16; i++)
			{
				car_pos_des[i] = car_traj_buffer[count][i];
			}
		}
		else
		{
			for (int i = 0; i < 16; i++)
			{
				car_pos_des[i] = car_traj_buffer[traj_num][i];
			}
		}
		T_BF << car_pos_des[0],car_pos_des[1],car_pos_des[2],car_pos_des[3],car_pos_des[4],
		car_pos_des[5],car_pos_des[6],car_pos_des[7],car_pos_des[8],car_pos_des[9],
		car_pos_des[10],car_pos_des[11],car_pos_des[12],car_pos_des[13],car_pos_des[14],car_pos_des[15];
		q_guess << pos_fdb[0],pos_fdb[1],pos_fdb[2],pos_fdb[3],pos_fdb[4],pos_fdb[5];
		q_des = GP50.calIK(q_guess,T_BF);
		for (int i = 0; i < 6; i++)
		{
			pos_des[i] = q_des(i);
		}
	}

	robot_hw_ptr->setJointPosCmd(pos_des);
	return true;
}
