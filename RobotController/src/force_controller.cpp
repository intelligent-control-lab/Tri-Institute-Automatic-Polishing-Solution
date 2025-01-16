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

    Eigen::VectorXd q(6);
    Eigen::VectorXd q1(6);
    Eigen::VectorXd q_guess(6);
    Eigen::VectorXd q_guess1(6);
    q <<  -0.024316334973315,	0.455177661097599,	-0.146541737377914,	0.041140136548949,	0.744742261320493	,-0.075374814182320;
    q_guess << 0.08182, -0.06807, -0.60672,  2.4037 , -0.49859, -3.3126;
    q_guess1 << 0.08182, -0.06807, -0.60672,  2.4037 , -1.49859, -3.3126;
    Eigen::MatrixXd T_BT = GP50.calFK(q_guess,'T');
    cout << T_BT << endl;
    q1 = GP50.calIK(q_guess1,T_BT);
    Eigen::MatrixXd Jg_BF = GP50.calGeomJaco(q_guess,'T');
    Eigen::MatrixXd Jg_BF_turn = GP50.calGeomJaco(q_guess,'T');
    Eigen::MatrixXd tempRows = Jg_BF.topRows(3);
    Eigen::MatrixXd tempRows1 = Jg_BF.bottomRows(3);
    Jg_BF_turn.topRows(3) = tempRows1;
    Jg_BF_turn.bottomRows(3) = tempRows;
    cout << q1 << endl;
    cout << Jg_BF << endl;
    HardwareInterface robot_hw;
    HybridForcePositionController HybridForcePosController;
    HybridForcePosCtr_Config hybird_force_position_cfg = {{1, 1, 1, 1, 1, 1}, HYBIRD_POSITION_FORCE_CONTROL};   
    HybridForcePosController.init(&hybird_force_position_cfg, &robot_hw);
    
    double joint_pos_fbk[6] = {0};
    static int brobot_ok = true;
    int64_t now = 0;
    int64_t last = 0;
    int64_t start, end = 0;
    auto stop = std::chrono::high_resolution_clock::now();;
    while(brobot_ok)
    {
        auto start1 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(start1 - stop);  
        if (duration.count() >= 4000)
        {
            brobot_ok =  HybridForcePosController.execute();
            stop = std::chrono::high_resolution_clock::now();
        }
    }
    printf("the code is finished");
    return true;
}