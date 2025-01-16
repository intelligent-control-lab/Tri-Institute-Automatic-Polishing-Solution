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

using namespace std;
RobotModel_GP50 GP50;
FILE *pFileData1;
int main(int argc, char *argv[])
{
    double car_traj_buffer[40000][16];
	FILE *jointFile;
	char buffer[1024];
	char *path = (char *)"../data/traj_car_generated.txt";
	jointFile = fopen(path, "r");
	if (jointFile == NULL)
	{
		std::cerr << "Error: Unable to open the file." << std::endl;
		return 0;
	}

	std::string line;
	int i = 0;
	while (fgets(buffer, sizeof(buffer), jointFile) != NULL)
	{
		if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &car_traj_buffer[i][0], &car_traj_buffer[i][1], &car_traj_buffer[i][2],
				   &car_traj_buffer[i][3], &car_traj_buffer[i][4], &car_traj_buffer[i][5], &car_traj_buffer[i][6], &car_traj_buffer[i][7], 
				   &car_traj_buffer[i][8], &car_traj_buffer[i][9], &car_traj_buffer[i][10], &car_traj_buffer[i][11],&car_traj_buffer[i][12], 
				   &car_traj_buffer[i][13], &car_traj_buffer[i][14], &car_traj_buffer[i][15]) == 16)
		{}
		else
		{
			std::cerr << "Error: Failed to read 16 data values from the line." << std::endl;
		}
		i++;
	}
	fclose(jointFile);

    char *path1 = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/ik.txt";
    pFileData1 = fopen(path1, "w");
    if (NULL == pFileData1)
    {
        printf("File construct failure\n");
        return -1;
    }

    std::cout << i << std::endl;
    Eigen::VectorXd q_guess(6);
    Eigen::VectorXd q_des(6);
    Eigen::Matrix<double, 4, 4> T_BF;
    HardwareInterface robot_hw;
    double pos_fdb[6] = {0};
    robot_hw.getJointPos(pos_fdb);

    for(int j=0;j<i;j++)
    {
        T_BF << car_traj_buffer[j][0],car_traj_buffer[j][1],car_traj_buffer[j][2],car_traj_buffer[j][3],car_traj_buffer[j][4],
		car_traj_buffer[j][5],car_traj_buffer[j][6],car_traj_buffer[j][7],car_traj_buffer[j][8],car_traj_buffer[j][9],
		car_traj_buffer[j][10],car_traj_buffer[j][11],car_traj_buffer[j][12],car_traj_buffer[j][13],car_traj_buffer[j][14],car_traj_buffer[j][15];
		q_guess << pos_fdb[0],pos_fdb[1],pos_fdb[2],pos_fdb[3],pos_fdb[4],pos_fdb[5];
		q_des = GP50.calIK(q_guess,T_BF);
        q_guess = q_des;
        cout << "q_des:" << q_des[0] * 180 /PI << " " << q_des[1]* 180 /PI  << " " << q_des[2]* 180 /PI  << " " << q_des[3]* 180 /PI  << " " << q_des[4] * 180 /PI << " " << q_des[5]* 180 /PI  << endl;	
        fprintf(pFileData1, "%f %f %f %f %f %f\n", q_des[0], q_des[1], q_des[2], q_des[3], q_des[4], q_des[5]);
    }
    return 1;
    return true;
}