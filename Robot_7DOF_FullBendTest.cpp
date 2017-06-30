// Robot_7DOF_FullBendTest.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include "Robot_7DOF_FB.h"
#include "dynamixel.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>
using namespace std;


#define DEBUG 1


#if (DEBUG)
	#define DBGMSG(x)  printf x;
#else
    #define DBGMSG(x)
#endif

int TestAction()
{
	float linkL[6]={L0,L1,L2,L3,L4,L5};
	float base[3]={0.0};
	float Pend[3]={500.0,-56.0,0};
	float PoseAngle[3]={30*DEF_RATIO_DEG_TO_RAD,0,0};
	float Rednt_alpha=-90*DEF_RATIO_DEG_TO_RAD;
	float theta[7]={0};

	//int rt = IK_7DOF_nonFB(L1,L2,L3,0,0,0,60,0,0,0,0,0,(float)-M_PI*0.5,theta);
	
	//int IK_7DOF_FB7roll(const float linkL[6],const float base[3],const float Pend[3],const float PoseAngle[3],const float Rednt_alpha,float* out_theta);
	int rt = IK_7DOF_FB7roll(linkL,base,Pend,PoseAngle,Rednt_alpha,theta);
	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		printf("axis %d=%f\n",gMapAxisNO[i],theta[i]);

	//AngleOverConstrain
	int over_index=0;
	bool bOver=AngleOverConstrain(theta,&over_index);
	if(bOver)
	{
		printf("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_H[over_index]*DEF_RATIO_RAD_TO_DEG);  
	}	

	unsigned short int velocity[MAX_AXIS_NUM]={50,50,50,50,50,0,50};
	
	rt=Output_to_Dynamixel(theta,velocity); 

	return 0;
}

int TestSyncwrite()
{
	
	//================================//
	//==output to motor by syncpage===//
	//===============================//
	short int Ang_pulse[MAX_AXIS_NUM]={500,500,500,500,500,400,500};
	short int velocity[MAX_AXIS_NUM]={50,50,50,50,50,0,50};
	unsigned short int SyncPage1[21]=
	{ 
		ID_AXIS1,(unsigned short int)Ang_pulse[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
		ID_AXIS2,(unsigned short int)Ang_pulse[Index_AXIS2],velocity[Index_AXIS2], 
		ID_AXIS3,(unsigned short int)Ang_pulse[Index_AXIS3],velocity[Index_AXIS3], 
		ID_AXIS4,(unsigned short int)Ang_pulse[Index_AXIS4],velocity[Index_AXIS4], 
		ID_AXIS5,(unsigned short int)Ang_pulse[Index_AXIS5],velocity[Index_AXIS5], 
		ID_AXIS6,(unsigned short int)Ang_pulse[Index_AXIS6],velocity[Index_AXIS6], 
		ID_AXIS7,(unsigned short int)Ang_pulse[Index_AXIS7],velocity[Index_AXIS7], 
	};

	
#if (DEBUG)
	for(int i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
		printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
#endif

	int rt=0;
	rt=syncWrite_x86(GOAL_POSITION,2,SyncPage1,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
  
	return 0;

}

void TestReadPos()
{
	//==Read pos_pus==//
	static float pos_pus[MAX_AXIS_NUM]={0};
	int rt=0;
	rt=Read_pos(pos_pus,DEF_UNIT_RAD);

	for(int i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		DBGMSG(("AXIS[%d]=%3f,",gMapAxisNO[i],pos_pus[i]))
	}

	DBGMSG(("\n"))
}

#include <ctime>
#include <iostream>
int clockTest()
{
	std::clock_t start;
    double duration;

	start = std::clock();
	while(1)
	{
   
    /* Your algorithm here */

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;

		if (duration==2)
		{
			 std::cout<<"printf: "<< duration <<'\n';
			 start = std::clock();
		}
	}


}

int _tmain(int argc, _TCHAR* argv[])
{
	int rt=DXL_Initial_x86();
	//ROM_Setting();
	if(rt==0)
		return 0;
	
	TestAction();
	//TestSyncwrite();
	
	TestReadPos();
	DXL_Terminate_x86();
	getchar();
	return 0;
}

