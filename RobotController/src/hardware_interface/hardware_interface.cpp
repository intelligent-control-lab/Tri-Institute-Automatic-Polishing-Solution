#include <iostream>
#include <stdlib.h>

#include "robot_var.h"
#include "hardware_interface.h"

#define PORT 10000
#define PORT_REAL_TIME 10001

float TIME_DURATION = 100.0f;

HardwareInterface::HardwareInterface()
{
	robot_connection_send = std::make_shared<stmotion_controller::udp::UDP_Interface>();
	robot_connection = std::make_shared<stmotion_controller::udp::UDP_Interface>();

	robot_connection->ServerSetup(11001);
	robot_connection_send->Setup(11000);
}

HardwareInterface::~HardwareInterface()
{

	cout << "HardwareInterface is being deleted" << endl;
}

bool HardwareInterface::setAxisEnable(const int *user_axis_enable)
{
	int i = 0;
	for (i = 0; i < JOINT_NUM; i++)
	{
		axis_enable[i] = user_axis_enable[i];
	}

	return true;
}

bool HardwareInterface::getJointPos(double *joint_pos)
{
	float joint_pos_raw[6] = {0};
	robot_connection->GetJointPos(joint_pos_raw);
	for (int i = 0; i < 6; i++)
	{
		joint_pos[i] = (double)joint_pos_raw[i];
	}

	return true;
}

bool HardwareInterface::getPositionerPos(double *positioner_pos)
{

	return true;
}


bool HardwareInterface::getJointSpd(double *joint_spd)
{


	return true;
}

bool HardwareInterface::getJointTrq(double *joint_trq)
{

	return true;
}

bool HardwareInterface::getSkinTrq(unsigned char *skin_trq)
{

	return true;
}

bool HardwareInterface::getRobotStates(double *joint_pos, double *joint_spd, double *joint_trq)
{
	static int bSuccess = true;
	bSuccess = getJointPos(joint_pos);
	if (bSuccess == false)
	{
		return false;
	}
	bSuccess = getJointSpd(joint_spd);
	if (bSuccess == false)
	{
		return false;
	}
	bSuccess = getJointTrq(joint_trq);
	if (bSuccess == false)
	{
		return false;
	}

	return true;
}

bool HardwareInterface::setJointPosCmd(const double *joint_pos_des)
{
	float joint_pos[6] = {0};

	for (int i = 0; i < JOINT_NUM; i++)
	{
		joint_pos[i] = (float)joint_pos_des[i];
	}
	
	robot_connection_send->SendJointPos(joint_pos);

	return true;
}

bool HardwareInterface::setPositionerPosCmd(const double *positioner_pos)
{
	float joint_pos[8] = {0};
	double joint_pos_fbk[6] = {0};
	getJointPos(joint_pos_fbk);
	for (int i = 0; i < JOINT_NUM; i++)
	{
		joint_pos[i] = (float)joint_pos_fbk[i];
	}

	joint_pos[6] = (float)positioner_pos[0];
	joint_pos[7] = (float)positioner_pos[1];


	robot_connection_send->SendJointPos(joint_pos);

	return true;
}
bool HardwareInterface::setJointPosInc(const double *joint_pos_des, const double *joint_pos_fbk)
{
	float joint_pos_inc[6] = {0};
	for (int i = 0; i < JOINT_NUM; i++)
	{
		joint_pos_inc[i] = (float)joint_pos_des[i] - joint_pos_fbk[i];








	}


	robot_connection_send->SendJointPos(joint_pos_inc);

	return true;
}

bool HardwareInterface::setJointTrqCmd(const double *joint_trq_des)
{

	return true;
}

bool HardwareInterface::setAxisMode(const int axis_mode)
{

	return true;
}

bool HardwareInterface::resetAxisState()
{

	return true;
}

bool HardwareInterface::setRobotPowerOff()
{

	return true;
}

bool HardwareInterface::JointPosFbkSupervision(const double *joint_pos)
{

	return true;
}

bool HardwareInterface::JointSpdFbkSupervision(const double *joint_spd)
{

	return true;
}

bool HardwareInterface::JointTrqFbkSupervision(const double *joint_trq)
{
	return true;
}

bool HardwareInterface::JointPosCmdSupervision(const double *joint_pos_des)
{

	return true;
}

bool HardwareInterface::JointTrqCmdSupervision(const double *joint_trq_des)
{

	return true;
}
