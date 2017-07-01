// Robot_7DOF_FullBendTest.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include "Robot_7DOF_FB.h"
#include "dynamixel.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>
//#include <iostream>
#include <windows.h> //sleep使用
using namespace std;





int MoveToPoint(float x,float y,float z,float alpha_deg,float beta_deg,float gamma_deg,float redant_alpha_deg)  //應該要有一個速度參數
{
	const float linkL[6]={L0,L1,L2,L3,L4,L5};
	const float base[3]={0.0};
	float Pend[3]={x,y,z};
	float PoseAngle[3]={alpha_deg*DEF_RATIO_DEG_TO_RAD,beta_deg*DEF_RATIO_DEG_TO_RAD,gamma_deg*DEF_RATIO_DEG_TO_RAD};
	float Rednt_alpha=redant_alpha_deg*DEF_RATIO_DEG_TO_RAD;
	float theta[7]={0};

	//inverse kinematics
	int rt = IK_7DOF_FB7roll(linkL,base,Pend,PoseAngle,Rednt_alpha,theta);
	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		DBGMSG(("axis %d=%f\n",gMapAxisNO[i],theta[i]))

	//AngleOverConstrain
	int over_index=0;
	bool bOver=AngleOverConstrain(theta,&over_index);
	if(bOver)
		DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_H[over_index]*DEF_RATIO_RAD_TO_DEG));  

	unsigned short int velocity[MAX_AXIS_NUM]={20,20,20,20,20,20};
	
	//output to motor
	rt=Output_to_Dynamixel(theta,velocity); 

	return 0;
}

#define DEF_DESCRETE_POINT 90
int TestRectangle()
{
	//把此路徑分成90份
	float O[3]={500,-50,0};  
	float Q[3]={500,-200,0};
	float R[3]={500,-200,-150};
	float S[3]={500,-50,-150};
	float point[3]={0,0,0};
 
	//Path=zeros(DEF_DESCRETE_POINT,3);//規畫的路徑點
	//PathPoint=zeros(DEF_DESCRETE_POINT,3);//記錄實際上的點，畫圖使用
	//PathTheta=zeros(DEF_DESCRETE_POINT,7);//記錄每軸角度，畫圖使用

	 //畫正方形做IK FK測試
	for(int t=1;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=25)
		{
			for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
				point[f]=O[f]+(Q[f]-O[f])*t/25; 
		}
		else if(t<=50)
		{
			for(int f=0;f<3;f++)  
				point[f]=Q[f]+(R[f]-Q[f])*(t-25)/25;
		}
		else if(t<=75)
		{
			for(int f=0;f<3;f++)  
				point[f]=R[f]+(S[f]-R[f])*(t-50)/25;
		}
		else 
		{
			for(int f=0;f<3;f++)  
				point[f]=S[f]+(O[f]-S[f])*(t-75)/15;
		}
	
		Sleep(100);
		MoveToPoint(point[0],point[1],point[2],30,0,0,-90);
		//DBGMSG(("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]))
		printf("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]);
	}

	return 0;
}



int TestAction()
{
	const float linkL[6]={L0,L1,L2,L3,L4,L5};
	const float base[3]={0.0};
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

	unsigned short int velocity[MAX_AXIS_NUM]={10,10,10,10,10,10,10};
	
	rt=Output_to_Dynamixel(theta,velocity); 

	return 0;
}



int TestOutput()
{
	int rt=0;
	unsigned short int velocity[MAX_AXIS_NUM]={10,10,10,50,50,0,50};
	float theta[7]={60*DEF_RATIO_DEG_TO_RAD,15*DEF_RATIO_DEG_TO_RAD,60*DEF_RATIO_DEG_TO_RAD,0,0,0,0};
	
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
	rt=Read_pos(pos_pus,DEF_UNIT_DEG);

	for(int i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		DBGMSG(("AXIS[%d]=%3.2f,",gMapAxisNO[i],pos_pus[i]))
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
			 //std::cout<<"printf: "<< duration <<'\n';
			 TestReadPos();
			 start = std::clock();
		}
	}


}


int _tmain(int argc, _TCHAR* argv[])
{
	int rt=DXL_Initial_x86();
	if(rt==0)
		return 0;
	
	MoveToPoint(500,-50,0,30,0,0,-90);
	Sleep(3000);

	for(int i=0;i<4;i++)
	{
		TestRectangle();
	}
	//TestOutput();

	//ROM_Setting();
	//TestAction();
	//clockTest();
	
	//TestSyncwrite();
	
	//TestReadPos();

	getchar();

	DXL_Terminate_x86();
	
	return 0;
}

