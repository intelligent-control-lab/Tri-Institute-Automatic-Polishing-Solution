#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <stdlib.h>

#include "robot_model_gp50.h"

RobotModel_GP50::RobotModel_GP50(void)
{

	X_BF0 << 0.0, 0.0, -1.0, 1.345,
			 0.0, -1.0, 0.0, 0.0,
			 -1.0, 0.0, 0.0, 1.620,
			 0.0, 0.0, 0.0, 1.0;

	X_FS <<  0.866,-0.5,0,0,
			 -0.5,-0.866,0,0,
			 0,0,-1,-0.055,
			 0,0,0,1;

	X_FT <<  0, -0.866,-0.5,-0.032682,
			0, -0.5, 0.866, 0.062585,
			-1.0, 0, 0, -0.579085,
			0.0, 0.0, 0.0, 1.0;

	X_ST = TransInv(X_FS)* X_FT;

	wlist << 0,0,1,
			0,1,0,
			0,-1,0,
			-1,0,0,
			0,-1,0,
			-1,0,0;

	plist << 0, 0, 0,
			0.145, 0, 0.540,
			0.145, 0, 1.410,
			0.145, 0, 1.620,
			1.17, 0,1.620,
			1.345, 0,1.620;

	Eigen::Vector3d w1(0,0,1);
	Eigen::Vector3d w2(0,1,0);
	Eigen::Vector3d w3(0,-1,0);
	Eigen::Vector3d w4(-1,0,0);
	Eigen::Vector3d w5(0,-1,0);
	Eigen::Vector3d w6(-1,0,0);

	Eigen::Vector3d p1(0, 0, 0);
	Eigen::Vector3d p2(0.145, 0, 0.540);
	Eigen::Vector3d p3(0.145, 0, 1.410);
	Eigen::Vector3d p4(0.145, 0, 1.620);
	Eigen::Vector3d p5(1.17, 0,1.620);
	Eigen::Vector3d p6(1.345, 0,1.620);

	Eigen::VectorXd s1(6);
	s1 << w1,w1.cross(p1);
	Eigen::VectorXd s2(6);
	s2 << w2,w2.cross(p2);
	Eigen::VectorXd s3(6);
	s3 << w3,w3.cross(p3);
	Eigen::VectorXd s4(6);
	s4 << w4,w4.cross(p4);
	Eigen::VectorXd s5(6);
	s5 << w5,w5.cross(p5);
	Eigen::VectorXd s6(6);
	s6 << w6,w6.cross(p6);

	Slist.col(0) = s1;
	Slist.col(1) = s2;
	Slist.col(2) = s3;
	Slist.col(3) = s4;
	Slist.col(4) = s5;
	Slist.col(5) = s6;
	cout << "RobotModel_GP50 is being created" << endl;
}

RobotModel_GP50::~RobotModel_GP50()
{
	cout << "RobotModel_GP50 is being deleted" << endl;
}


Eigen::MatrixXd RobotModel_GP50::calFK(const Eigen::VectorXd& thetaList,char flag)
{
	Eigen::MatrixXd X_BF = mr::FKinSpace(X_BF0, Slist, thetaList);

	if(flag == 'F')
	{
		return X_BF;
	}
	else if(flag == 'S')
	{
		return X_BF * X_FS;
	}
	else if(flag == 'T')
	{
		return X_BF * X_FT;
	}
	return X_BF;
}

Eigen::VectorXd RobotModel_GP50::calIK(const Eigen::VectorXd& thetalistguess,const Eigen::MatrixXd& X_BT)
{
	Eigen::VectorXd	thetalist(6);
	thetalist = thetalistguess;
	double eomg = 0.00001;
	double ev = 0.000001;
	bool IK_success = false;
	Eigen::MatrixXd X_BF = X_BT * TransInv(X_FT);
	IK_success = mr::IKinSpace(Slist, X_BF0, X_BF, thetalist, eomg, ev);
	if(IK_success == false)
	{
		printf("IKinSpace false");
		thetalist << 0,0,0,0,0,0;
		return thetalist;
	}
	else
	{
		return thetalist;
	}
}

Eigen::MatrixXd RobotModel_GP50::calBodyJaco(const Eigen::VectorXd& thetaList,char flag)
{
	Eigen::MatrixXd Js_BF,Jb_BF,Jb_BS,Jb_BT;
	Eigen::MatrixXd X_BF;
	X_BF = calFK(thetaList,'F');
	Js_BF = JacobianSpace(Slist, thetaList);
	Jb_BF = Js_BF * TransInv(X_BF);
	if(flag == 'S')
	{
		return Jb_BS;
	}
	else if(flag == 'T')
	{
		Jb_BS = Jb_BF * TransInv(X_FT);
		return Jb_BS;
	}
	else if(flag == 'F')
	{
		Jb_BS = Jb_BF * TransInv(X_BF);
		return Jb_BS;
	}
	return Jb_BF;
}

Eigen::MatrixXd RobotModel_GP50::calGeomJaco(const Eigen::VectorXd& thetaList,char flag)
{
	Eigen::MatrixXd Jb_BX;
	Jb_BX = calBodyJaco(thetaList,flag);
	Eigen::MatrixXd X_BX,R_BX;
	X_BX = calFK(thetaList,flag);
	R_BX = X_BX.block<3, 3>(0, 0);

	return Jb_BX * Rot66(R_BX);
}

Eigen::VectorXd RobotModel_GP50::calF_ext_S(const Eigen::VectorXd& thetaList,const Eigen::VectorXd& FTS_raw)
{
  Eigen::Vector3d L;
  double x = 0.0020;
  double y = 0.0157;
  double z = 0.1896;
  double Fx0 = -16.1864;
  double Fy0 = -3.7312;
  double Fz0 = 2.3040;
  double Tx0 = 0.0002;
  double Ty0 = 0.3049;
  double Tz0 = -0.0273;  
  
  L << 0.1309,-1.4220,-57.4902;

  Eigen::VectorXd	FTS_ext(6);
  Eigen::MatrixXd R_BS;
  Eigen::MatrixXd T_BS = calFK(thetaList,'S');
  R_BS = T_BS.block<3, 3>(0, 0);

  Eigen::Vector3d temp = R_BS.transpose() * L;

  double Gx = temp(0);
  double Gy = temp(1);
  double Gz = temp(2);

  double Tgx = Gy * z + Gz * y;
  double Tgy = Gz * x + Gy * x;
  double Tgz = Gy * x + Gz * x;
  
  FTS_ext(0) = FTS_raw(0) + Gx - Fx0;
  FTS_ext(1) = FTS_raw(1) + Gy - Fy0;
  FTS_ext(2) = FTS_raw(2) + Gz + Fz0;
  FTS_ext(3) = FTS_raw(3) - Tgx - Tx0;
  FTS_ext(4) = FTS_raw(4) + Tgy - Ty0;
  FTS_ext(5) = FTS_raw(5) - Tgz + Tz0;
  return FTS_ext;
}

Eigen::VectorXd RobotModel_GP50::calToolGravityParameter()
{
	Eigen::VectorXd	tool_gravity_parameter(12);
	return tool_gravity_parameter;
}