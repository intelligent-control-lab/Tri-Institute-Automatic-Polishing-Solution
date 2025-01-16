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
#include "joint_pos_controller.h"
#include "hybird_force_position_controller.h"
#include "robot_model_gp50.h"
#include "ft_sensor.h"

using namespace std;
FILE *pFileData;
FILE *pFileData1;
RobotModel_GP50 GP50;
ati::FTSensor ftsensor_;

int main(int argc, char *argv[])
{

    char *path = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/datalog.txt";
    pFileData = fopen(path, "w");
    if (NULL == pFileData)
    {
        printf("File construct failure\n");
        return -1;
    }

    char *path1 = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/tool_gravity_data.txt";
    pFileData1 = fopen(path1, "w");
    if (NULL == pFileData1)
    {
        printf("File construct failure\n");
        return -1;
    }


    HardwareInterface robot_hw;
    JointPositionController JointPosController;
    JointPosCtr_Config user_joint_pos_cfg = {{1, 1, 1, 1, 1, 1}, JOINT_POSITION_CONTROL};
    JointPosController.init(&user_joint_pos_cfg, &robot_hw);
    double joint_pos_fbk[6] = {0};
    double p_pos_fbk[2] = {0};
    static int brobot_ok = true;
    while(brobot_ok)
    {
        auto start1 = std::chrono::high_resolution_clock::now();
        brobot_ok = JointPosController.execute();
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start1);  
        usleep(4000);
    }
    
    printf("the code is finished");
    return true;
}