#ifndef __ROBOT_MODEL_H_
#define __ROBOT_MODEL_H_

#include <iostream>
#include <stdlib.h>
#include <math.h>

#include "modern_robotics.h"

using namespace std;
using namespace mr;
typedef struct
{
	int movement_mode;
}RobotModel_Config;

class RobotModel_GP50{
   public:
	  	RobotModel_GP50(void);
	  	~RobotModel_GP50();

	  	Eigen::MatrixXd calFK(const Eigen::VectorXd& thetaList,char flag);
		Eigen::VectorXd calIK(const Eigen::VectorXd& thetalistguess, const Eigen::MatrixXd& T);
		Eigen::MatrixXd calBodyJaco(const Eigen::VectorXd& thetaList,char flag);
		Eigen::MatrixXd calGeomJaco(const Eigen::VectorXd& thetaList,char flag);
		Eigen::VectorXd calF_ext_S(const Eigen::VectorXd& thetaList,const Eigen::VectorXd& FTS_raw);
		Eigen::VectorXd calToolGravityParameter();

		Eigen::Matrix<double, 4, 4> X_FS = Eigen::Matrix<double, 4, 4>::Zero();
		Eigen::Matrix<double, 4, 4> X_FT = Eigen::Matrix<double, 4, 4>::Zero();
		Eigen::Matrix<double, 4, 4> X_ST = Eigen::Matrix<double, 4, 4>::Zero();

   private:
		Eigen::Matrix<double, 6, 3> plist = Eigen::Matrix<double, 6, 3>::Zero();
		Eigen::Matrix<double, 6, 3> wlist = Eigen::Matrix<double, 6, 3>::Zero();
		Eigen::Matrix<double, 6, 6> Slist = Eigen::Matrix<double, 6, 6>::Zero();
		Eigen::Matrix<double, 4, 4> X_BF0 = Eigen::Matrix<double, 4, 4>::Zero();
};

#endif
