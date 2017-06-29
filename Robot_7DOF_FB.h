#ifndef ROBOT_7DOF_FB__H
#define ROBOT_7DOF_FB__H


#include "Matrix.h"




//固定參數
#define L0 0     //可能要刪掉
#define L1 100     //upper arm
#define L2 100   //forearm
#define L3 10     //length of end effector
#define X_BASE 0  //基準點
#define Y_BASE 0
#define Z_BASE 0



Matrix R_z1x2y3(float alpha,float beta,float gamma);
float norm(const Matrix& v);
Matrix Rogridues(float theta,const Matrix& V_A);
int IK_7DOF_stanley(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta);





enum{
	AXIS1=0,
	AXIS2,
	AXIS3,
	AXIS4,
	AXIS5,
	AXIS6,
	AXIS7
};



#endif    /* ROBOT_7DOF_FB__H */