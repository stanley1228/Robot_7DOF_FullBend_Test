#ifndef ROBOT_7DOF_FB__H
#define ROBOT_7DOF_FB__H


#include "Matrix.h"


#define DEBUG 1

#if (DEBUG)
	#define DBGMSG(x)  printf x;
#else
    #define DBGMSG(x)
#endif

//==========
//==MAX AXIS
//==========
#define MAX_AXIS_NUM  7

//===========================
//==Axis index,ID,NO mapping
//==========================
enum{
	Index_AXIS1=0,
	Index_AXIS2,
	Index_AXIS3,
	Index_AXIS4,
	Index_AXIS5,
	Index_AXIS6,
	Index_AXIS7
};

enum{
	ID_AXIS1=1,
	ID_AXIS2,
	ID_AXIS3,
	ID_AXIS4,
	ID_AXIS5,
	ID_AXIS6,
	ID_AXIS7
};

enum{
	NO_AXIS1=1,
	NO_AXIS2,
	NO_AXIS3,
	NO_AXIS4,
	NO_AXIS5,
	NO_AXIS6,
	NO_AXIS7
};


static const unsigned char gMapAxisNO[MAX_AXIS_NUM]=
{
	NO_AXIS1,
	NO_AXIS2,
	NO_AXIS3,
	NO_AXIS4,
	NO_AXIS5,
	NO_AXIS6,
	NO_AXIS7
};
static const unsigned char gMapAxisID[MAX_AXIS_NUM]=
{
	ID_AXIS1,
	ID_AXIS2,
	ID_AXIS3,
	ID_AXIS4,
	ID_AXIS5,
	ID_AXIS6,
	ID_AXIS7
};


//================
//==Unit transform
//=================
#define DEF_PI (3.1415926F)
#define DEF_RATIO_PUS_TO_DEG (0.0879F)		//360/4096
#define DEF_RATIO_PUS_TO_RAD (0.0015F)		//2pi/4096   0.00153398078788564122971808758949
#define DEF_RATIO_DEG_TO_PUS (11.3778F)		//4096/360
#define DEF_RATIO_DEG_TO_RAD (DEF_PI/180)		//pi/180	0.01745329251994329576923690768489 (0.0175F)
#define DEF_RATIO_RAD_TO_DEG (180/DEF_PI)	
#define DEF_RATIO_RAD_TO_PUS (651.8986F)	//4096/2pi	651.89864690440329530934789477382


//for read pos unit select
enum{
	DEF_UNIT_RAD=1,
	DEF_UNIT_DEG,
	DEF_UNIT_PUS
};

//=====================================
//==robot hard ware dependent parameter
//=====================================

//==robot to Motor offset==//  //robot pos=motor position -M2R_OFFSET

#define AXIS1_R2M_OFFSET_DEG 180
#define AXIS2_R2M_OFFSET_DEG 270
#define AXIS3_R2M_OFFSET_DEG 180
#define AXIS4_R2M_OFFSET_DEG 90
#define AXIS5_R2M_OFFSET_DEG 180
#define AXIS6_R2M_OFFSET_DEG 90
#define AXIS7_R2M_OFFSET_DEG 90

//==robot angle limit==//
#define AXIS1_ROBOT_LIM_DEG_L (-80)
#define AXIS1_ROBOT_LIM_DEG_H 170
#define AXIS2_ROBOT_LIM_DEG_L (-180)
#define AXIS2_ROBOT_LIM_DEG_H 10
#define AXIS3_ROBOT_LIM_DEG_L (-105)
#define AXIS3_ROBOT_LIM_DEG_H 170
#define AXIS4_ROBOT_LIM_DEG_L 0	
#define AXIS4_ROBOT_LIM_DEG_H 170	
#define AXIS5_ROBOT_LIM_DEG_L (-30)
#define AXIS5_ROBOT_LIM_DEG_H 90
#define AXIS6_ROBOT_LIM_DEG_L (-37)
#define AXIS6_ROBOT_LIM_DEG_H 90
#define AXIS7_ROBOT_LIM_DEG_L (-180)
#define AXIS7_ROBOT_LIM_DEG_H 180


//==robot TORQUE limit==//  0~1023
#define AXIS1_MAX_TORQUE 614	//60%
#define AXIS2_MAX_TORQUE 430
#define AXIS3_MAX_TORQUE 300	//40%
#define AXIS4_MAX_TORQUE 415	//40%
#define AXIS5_MAX_TORQUE 55
#define AXIS6_MAX_TORQUE 200
#define AXIS7_MAX_TORQUE 55

static const unsigned short int gr2m_offset_pulse[MAX_AXIS_NUM]=
{
	(unsigned short int)(AXIS1_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXIS2_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXIS3_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXIS4_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXIS5_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXIS6_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS),
	(unsigned short int)(AXIS7_R2M_OFFSET_DEG*DEF_RATIO_DEG_TO_PUS)
};


static const float grobot_lim_rad_L[MAX_AXIS_NUM]=
{
	AXIS1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXIS2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXIS3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXIS4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXIS5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXIS6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD,
	AXIS7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_rad_H[MAX_AXIS_NUM]=
{
	AXIS1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXIS2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXIS3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXIS4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXIS5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXIS6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD,
	AXIS7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_RAD
};

static const float grobot_lim_pus_L[MAX_AXIS_NUM]=
{
	AXIS1_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXIS2_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXIS3_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXIS4_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXIS5_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXIS6_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS,
	AXIS7_ROBOT_LIM_DEG_L*DEF_RATIO_DEG_TO_PUS
};

static const float grobot_lim_pus_H[MAX_AXIS_NUM]=
{
	AXIS1_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXIS2_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXIS3_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXIS4_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXIS5_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXIS6_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS,
	AXIS7_ROBOT_LIM_DEG_H*DEF_RATIO_DEG_TO_PUS
};

//=================================
//==morot max pulse in joint mode
//=================================
#define DEF_JOINT_MODE_MAX_PULSE 4095
#define DEF_JOINT_MODE_MIN_PULSE 0
//=================================
//==morot max pulse in joint mode
//=================================
#define DEF_JOINT_MODE_MAX_PULSE 4095
#define DEF_JOINT_MODE_MIN_PULSE 0

//===================
//==ROBOT link length
//====================
#define L0 225    	//head to shoulder
#define L1 250    	//upper arm
#define L2 50   	//forearm
#define L3 50     	//length of end effector
#define L4 250     	//length of end effector
#define L5 150     	//length of end effector
#define X_BASE 0  	//基準點只能都先設0
#define Y_BASE 0
#define Z_BASE 0



#define DEF_VERY_SMALL (1.e-5)//很小的量判斷為0使用

//==========
//Function
//==========
unsigned char getMapAxisNO(unsigned char index); //index 0~ (MAX_AXIS_NUM-1)
unsigned char getMapAxisID(unsigned char index);
int ROM_Setting();
int Read_pos(float *pos,unsigned char unit);
int Output_to_Dynamixel(const float *Ang_rad,const unsigned short int *velocity) ;
Matrix R_z1x2y3(float alpha,float beta,float gamma);
float norm(const Matrix& v);
Matrix Rogridues(float theta,const Matrix& V_A);
int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta);
int IK_7DOF_FB7roll(const float linkL[6],const float base[3],const float Pend[3],const float PoseAngle[3],const float Rednt_alpha,float* out_theta);
bool AngleOverConstrain(const float theta[MAX_AXIS_NUM],int *OverIndex);


//dynamixel use
int syncWrite_x86(unsigned short int start_addr, unsigned short int data_length, unsigned short int *param, unsigned short int param_length); // WORD(16bit) syncwrite() for DXL
int setPosition_x86(int ServoID, int Position, int Speed);//stanley
int DXL_Initial_x86();
int DXL_Terminate_x86();


#endif    /* ROBOT_7DOF_FB__H */