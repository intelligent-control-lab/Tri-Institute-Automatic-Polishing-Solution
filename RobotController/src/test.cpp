#include <iostream>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <string.h>
#include <sys/time.h>
#include <math.h>
#include <vector>
#include <Eigen/Dense>
#include <chrono>

#include "hardware_interface.h"
#include "robot_model_gp50.h"
#include "ft_sensor.h"
using namespace std;
RobotModel_GP50 GP50;
FILE *pFileData;
FILE *pFileData1;
ati::FTSensor ftsensor_;
int main(int argc, char *argv[])
{
    char *path1 = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/tool_gravity_data.txt";
    pFileData1 = fopen(path1, "a");
    if (NULL == pFileData1)
    {
        printf("File construct failure\n");
        return -1;
    }


    HardwareInterface robot_hw;
    double joint_pos_fbk[6] = {0};
	float measurements[6];
    robot_hw.getJointPos(joint_pos_fbk);

	if (ftsensor_.init("192.168.1.1"))
	{
		ftsensor_.getRDTRate();
	}

    bool bSuccess = false;
    bSuccess = ftsensor_.getMeasurements(measurements);
    if(bSuccess == false)
    {
        printf("Torce torque sensor read failed" );
        return false;
    }

    usleep(500000);
    cout << "joint_pos_fbk:" << joint_pos_fbk[0] << " " << joint_pos_fbk[1] << " " << joint_pos_fbk[2] << " " << joint_pos_fbk[3] << " " << joint_pos_fbk[4] << " " << joint_pos_fbk[5] << endl;
    cout << "measurements:" << measurements[0] << " " << measurements[1] << " " << measurements[2] << " " << measurements[3] << " " << measurements[4] << " " << measurements[5] << endl;
    int count = 1;
    fprintf(pFileData1, "%d %f %f %f %f %f %f %f %f %f %f %f %f\n", count, measurements[0], measurements[1], measurements[2], measurements[3], measurements[4], measurements[5],
    joint_pos_fbk[0], joint_pos_fbk[1], joint_pos_fbk[2], joint_pos_fbk[3], joint_pos_fbk[4], joint_pos_fbk[5]);
    fclose(pFileData1);

    return true;
}