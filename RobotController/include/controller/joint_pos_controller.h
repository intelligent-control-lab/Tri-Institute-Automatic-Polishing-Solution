#ifndef __JOINT_POSITION_CONTROLLER_H_
#define __JOINT_POSITION_CONTROLLER_H_

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
	JOINT_POSITION_CONTROL  = 1,
	JOINT_POSITION_INCENTIVE_TRJ = 2,
	JOINT_ADMITTANCE_CONTROL  = 3,
	Car_POSITION_CONTROL  = 4,
}JointPosCtr_MovementMode;

typedef struct
{
	int joint_enable[JOINT_NUM];
	int movement_mode;
}JointPosCtr_Config;

class JointPositionController{
   public:
	  JointPositionController();
	  ~JointPositionController();

	  bool init(const JointPosCtr_Config* user_joint_pos_cfg, HardwareInterface* user_robot_hw_ptr);
      bool execute();

   private:
		int count;
		int traj_num;
		double pos_fdb[JOINT_NUM] = {0};
		double spd_fdb[JOINT_NUM] = {0};
		double trq_fdb[JOINT_NUM] = {0};


		JointPosCtr_Config joint_pos_cfg;
		HardwareInterface* robot_hw_ptr;
		double last_posval[3];
};

#endif
