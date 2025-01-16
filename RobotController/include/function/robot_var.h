#ifndef _ROBOTVAR_H__
#define _ROBOTVAR_H__

#define SYSTEM_OK    0
#define SYSTEM_ERR  -1

#ifndef SYSTEM_TRUE
#define SYSTEM_TRUE  1
#endif
#ifndef SYSTEM_FALSE
#define SYSTEM_FALSE 0
#endif
#define Square(n)   ((n) * (n))
#define Cube(n)     ((n) * (n) * (n))



#define RigidBlend  0

#ifndef Max
#define Max(a,b)    (((a) > (b)) ? (a) : (b))
#endif
#define Max3(a,b,c) Max(Max(a,b), (c))
#ifndef Min
#define Min(a,b)    (((a) < (b)) ? (a) : (b))
#endif

#define ROBOT_TYPE_4   4
#define ROBOT_TYPE_6   6
#define SERVO_TYPE     0
#define JOINT_NUM	   6
#define POSE_NUM       6
#define DELTA_TAU_MAX  1

#ifndef DOF
#define DOF            9
#endif

#ifndef TEST_ROBOT_CONTROL
#define INTERP_T	   0.004
#define POS_T          0.004
#else
#define INTERP_T       0.016
#define POS_T          0.004
#endif
#define VerifyError 0.000001
#define RT_LITTLE   0.0000001
#define SERR         0.0000001
#define POS_SERR      0.00036
#define POS_PRECISION       1
#ifndef PI
#define PI      3.1415926535898
#endif
#define PI2     6.2831853071796
#define PI_RAD  0.0174532925199
#define PI_DEG  57.2957795130823
#define ERR_STOP_T     1




#define RUN_MODE_TEACH     1
#define RUN_MODE_AUTO      2
#define RUN_MODE_ERR       0
#define RUN_DIR_POS        1
#define RUN_DIR_NEG       -1
#define RUN_STATE_RUNNING  1
#define RUN_STATE_STOP     0
#define RUN_STOPMODE_HOLD  0
#define RUN_STOPMODE_STOP  1
#define RUN_LIM_POS        1
#define RUN_LIM_NOT        0
#define RUN_LIM_NEG       -1



#define COORDINATE_JOINT   1
#define COORDINATE_WORLD   2
#define COORDINATE_TOOL    3
#define COORDINATE_USER    4
#define COORDINATE_ADDITIONALAXIS  5
#define COORDINATE_JOINT_VEL   1
#define COORDINATE_WORLD_VEL   2
#define COORDINATE_TOOL_VEL    3
#define COORDINATE_USER_VEL    4
#define COORDINATE_ADDITIONALAXIS_VEL 5

#define COORDINATE_JOINT_POS  11
#define COORDINATE_WORLD_POS  12
#define COORDINATE_TOOL_POS   13
#define COORDINATE_USER_POS   14
#define COORDINATE_ADDITIONALAXIS_POS 15

#define MOTOR_ENABLE       1
#define MOTOR_DISABLE      0


#define  TrajJoint               0
#define  TrajLine                1
#define  TrajBsLine              2  

#define   VelModeSTsmooth        1 
#define   VelModePoly            2  
#define   VelModeSTshape         3 
#define   VelModeSshapeInv       4   
#define   BlendError             0 
#define   BlendJointST           1  
#define   BlendJointPoly         2 
#define   BlendCarSpace          3  
#define   BlendJointFusion       4   
#define   NoneMode               99 
#define   STSecNum               8
#define   DAxisNum              2*DOF
#define  VelSecNum              2*DOF
#define  QuatSlerpMode          0 
#define  QuatBezierMode         1  
#define  SignleAngleMode        2 
#define  PolyAngleMode          3  
#define   BSNode0                0.0 
#define   BSNode1                0.366 
#define   BSNode2                0.5 
#define   BSNode3                0.634  
#define   BSNode4                1.0    
#define    LFS_ISTshape          1  
#define    LFS_BlendMode         0    

#define RT_LITTLE   0.0000001
#define PI      3.1415926535898

#define PI2     6.2831853071796
#define PI_RAD  0.0174532925199
#define PI_DEG  57.2957795130823
#define PROG_VELMODE_EULER     1
#define PROG_VELMODE_ANGLE     2

#define MAX_OVERFOLW_PULSE  65536
#define MAX_OVERFOLW_DEG    11796480

#define U16 unsigned short int
#define U32 unsigned  int

#endif
