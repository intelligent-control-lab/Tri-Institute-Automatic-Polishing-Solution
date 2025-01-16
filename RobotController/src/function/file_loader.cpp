#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "robot_var.h"
#include "joint_pos_controller.h"
#include "ft_sensor.h"

extern FILE *pFileData;

int loadPoint(double task[][6])
{
	FILE *jointFile;
	char buffer[256];

	char *path = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/traj_generated.txt";
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
		if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &task[i][0], &task[i][1], &task[i][2],
				   &task[i][3], &task[i][4], &task[i][5]) == 6)
		{
		}
		else
		{
			std::cerr << "Error: Failed to read 6 data values from the line." << std::endl;
		}

		i++;
	}
	fclose(jointFile);
	return i-1;
}

int loadForceTraj(double task[][6])
{
	FILE *forceFile;
	char buffer[256];
	char *path = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/force_traj.txt";
	forceFile = fopen(path, "r");
	if (forceFile == NULL)
	{
		std::cerr << "Error: Unable to open the file." << std::endl;
		return 0;
	}


	std::string line;
	int i = 0;
	while (fgets(buffer, sizeof(buffer), forceFile) != NULL)
	{
		if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &task[i][0], &task[i][1], &task[i][2], 
														&task[i][3], &task[i][4], &task[i][5]) == 6)
		{
		}
		else
		{
			std::cerr << "Error: Failed to read 6 data values from the line." << std::endl;
		}

		i++;
	}
	fclose(forceFile);
	return i-1;
}

int loadContactFrame(double task[][9]) {
	FILE *contactFrameFile;
	char buffer[256];
	char *path = (char *)"/home/icl/Documents/zhongqi/Automatic-Polishing-Solution/RobotController/data/contact_frame.txt";
	contactFrameFile = fopen(path, "r");
	if (contactFrameFile == NULL)
	{
		std::cerr << "Error: Unable to open the file." << std::endl;
		return 0;
	}


	std::string line;
	int i = 0;
	while (fgets(buffer, sizeof(buffer), contactFrameFile) != NULL)
	{
		if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf", 
							&task[i][0], &task[i][1], &task[i][2], 
							&task[i][3], &task[i][4], &task[i][5],
							&task[i][6], &task[i][7], &task[i][8]) == 9)
		{
		}
		else
		{
			std::cerr << "Error: Failed to read 9 data values from the line." << std::endl;
		}

		i++;
	}
	fclose(contactFrameFile);
	return i-1;
}

int loadCarPoint(double task[][16])
{
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
		if (sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &task[i][0], &task[i][1], &task[i][2],
				   &task[i][3], &task[i][4], &task[i][5], &task[i][6], &task[i][7], 
				   &task[i][8], &task[i][9], &task[i][10], &task[i][11],&task[i][12], 
				   &task[i][13], &task[i][14], &task[i][15]) == 16)
		{
		}
		else
		{
			std::cerr << "Error: Failed to read 16 data values from the line." << std::endl;
		}

		i++;
	}
	fclose(jointFile);
	return i-1;
}