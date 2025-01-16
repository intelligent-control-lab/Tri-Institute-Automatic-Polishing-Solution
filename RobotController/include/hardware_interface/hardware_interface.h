#ifndef __HARDWARE_INTERFACE_H_
#define __HARDWARE_INTERFACE_H_

#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <string>
#include <vector>

#include "udp_Interface.h"
#include "robot_var.h"

using namespace std;

typedef enum
{
	AXIS_POSITION_MODE  = 8,
	AXIS_VELOCITY_MODE = 9,
	AXIS_TORQUE_MODE  = 10,
}HardwareInterface_AxisMode;

typedef struct
{
	bool joint_pos_fbk_supervision_enable;
	bool joint_spd_fbk_supervision_enable;
	bool joint_trq_fbk_supervision_enable;
	bool joint_pos_cmd_supervision_enable;
	bool joint_trq_cmd_supervision_enable;
}HardwareInterface_SupervisionEnable;

class HardwareInterface{
	public:
		HardwareInterface();
		~HardwareInterface();


		double real_trq[JOINT_NUM];
		double joint_trq_fbk[JOINT_NUM];

		bool getRobotStates(double* joint_pos,double* joint_spd,double* joint_trq);
		bool getJointPos(double* joint_pos);
		
		bool getJointSpd(double* joint_spd);
		bool getJointTrq(double* joint_trq);
		bool getSkinTrq(unsigned char* skin_trq);
		bool getPositionerPos(double *positioner_pos);

		bool setPositionerPosCmd(const double *positioner_pos);
		bool setAxisEnable(const int* user_axis_enable);
		bool setJointPosCmd(const double* joint_pos_des);
		bool setJointPosInc(const double *joint_pos_des,const double *joint_pos_fbk);
		bool setJointTrqCmd(const double* joint_trq_des);
		bool resetAxisState();
		bool setAxisMode(const int axis_mode);
		bool setRobotPowerOff();
		stmotion_controller::udp::UDP_Interface::Ptr robot_connection;
		stmotion_controller::udp::UDP_Interface::Ptr robot_connection_send;


	private:
		unsigned int actuator_count;
		const char* robot_name;
		double motor_pos_precision;
		double motor_spd_precision;
		double motor_trq_fbk_precision;
		double motor_trq_cmd_precision;
		double motor_abs_zero[JOINT_NUM];
		double trq_static_offset[JOINT_NUM];
		double trq_dym_offset[JOINT_NUM];
		double joint_range[JOINT_NUM];
		double joint_max_spd[JOINT_NUM];
		double joint_max_trq[JOINT_NUM];
		
		int motor_dir[JOINT_NUM];
		int axis_enable[JOINT_NUM];
		int first_enter[JOINT_NUM];

		HardwareInterface_SupervisionEnable hw_supervision;
		
		bool JointPosFbkSupervision(const double* joint_pos);
		bool JointSpdFbkSupervision(const double* joint_spd);
		bool JointTrqFbkSupervision(const double* joint_trq);
		bool JointPosCmdSupervision(const double* joint_pos_des);
		bool JointTrqCmdSupervision(const double* joint_trq_des);
};

#endif
