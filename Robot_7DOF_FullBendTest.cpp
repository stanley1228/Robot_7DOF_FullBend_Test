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


#define DEF_DESCRETE_POINT 90
int TestRectangle_Dual()
{
	float O_R[3]={500,-50 ,0};
	float Q_R[3]={500,-200,0};
	float R_R[3]={500,-200,-200};
	float S_R[3]={500,-50 ,-200};

	float O_L[3]={500,50 ,0};
	float Q_L[3]={500,200,0};
	float R_L[3]={500,200,-200};
	float S_L[3]={500,50 ,-200};
 
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={60,0,0};
	float Rednt_alpha_R=-90;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,0,0};
	float Rednt_alpha_L=90;


	//move to initial point
	MoveToPoint(DEF_RIGHT_HAND,O_R,pose_deg_R,Rednt_alpha_R);
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L);
	Sleep(3000);

	//畫正方形做IK 測試
	for(int t=0;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=25)
		{
			for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
			{
				Pend_R[f]=O_R[f]+(Q_R[f]-O_R[f])*t/25; 
				Pend_L[f]=O_L[f]+(Q_L[f]-O_L[f])*t/25; 
			}
		}
		else if(t<=50)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=Q_R[f]+(R_R[f]-Q_R[f])*(t-25)/25;
				Pend_L[f]=Q_L[f]+(R_L[f]-Q_L[f])*(t-25)/25;
			}
		}
		else if(t<=75)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_R[f]+(S_R[f]-R_R[f])*(t-50)/25;
				Pend_L[f]=R_L[f]+(S_L[f]-R_L[f])*(t-50)/25;
			}
		}
		else 
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=S_R[f]+(O_R[f]-S_R[f])*(t-75)/15;
				Pend_L[f]=S_L[f]+(O_L[f]-S_L[f])*(t-75)/15;
			}
		}
	
		Sleep(500);
		MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,Pend_L,pose_deg_L,Rednt_alpha_L);
		//DBGMSG(("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]))
		printf("Pend_R[%d]=[%f,%f,%f],Pend_L[%d]=[%f,%f,%f]\n",t,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
	}

	return 0;

}

int TestMoveToHome_Dual()
{
	float theta_R[7]={0,0,0,0*DEF_RATIO_DEG_TO_RAD,0,0,0};
	float theta_L[7]={0,0,0,0*DEF_RATIO_DEG_TO_RAD,0,0,0};

	//output to motor
	unsigned short int velocity_R[MAX_AXIS_NUM]={10,10,10,10,10,10,10};
	unsigned short int velocity_L[MAX_AXIS_NUM]={10,10,10,10,10,10,10};
	
	Output_to_Dynamixel_Dual(theta_R,velocity_R,theta_L,velocity_L); 

	Sleep(3000);
	
	return 0;

}




int TestRectangle_RightHand()
{
	//把此路徑分成90份
	float O[3]={500,-50,0};  
	float Q[3]={500,-200,0};
	float R[3]={500,-200,-150};
	float S[3]={500,-50,-150};
	float point[3]={0,0,0};
	float pose_deg[3]={30,0,0};

	

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
		MoveToPoint(DEF_RIGHT_HAND,point,pose_deg,-90);
		//DBGMSG(("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]))
		printf("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]);
	}

	return 0;
}


int TestRectangle_LeftHand()
{
	//把此路徑分成90份
	float O_L[3]={500,50 ,0};
	float Q_L[3]={500,200,0};
	float R_L[3]={500,200,-200};
	float S_L[3]={500,50 ,-200};
 
	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,45,0};
	float Rednt_alpha_L=90;
	
	//move to initial point
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L);
	Sleep(3000);


	 //畫正方形做IK FK測試
	for(int t=1;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=25)
		{
			for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
				Pend_L[f]=O_L[f]+(Q_L[f]-O_L[f])*t/25; 
		}
		else if(t<=50)
		{
			for(int f=0;f<3;f++)  
				Pend_L[f]=Q_L[f]+(R_L[f]-Q_L[f])*(t-25)/25;
		}
		else if(t<=75)
		{
			for(int f=0;f<3;f++)  
				Pend_L[f]=R_L[f]+(S_L[f]-R_L[f])*(t-50)/25;
		}
		else 
		{
			for(int f=0;f<3;f++)  
				Pend_L[f]=S_L[f]+(O_L[f]-S_L[f])*(t-75)/15;
		}
	
		Sleep(500);
		MoveToPoint(DEF_LEFT_HAND,Pend_L,pose_deg_L,Rednt_alpha_L);
		//DBGMSG(("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]))
		printf("Pend_L[%d]=[%f,%f,%f]\n",t,Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
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
	int rt = IK_7DOF_FB7roll(DEF_RIGHT_HAND,linkL,base,Pend,PoseAngle,Rednt_alpha,theta);
	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		printf("axis %d=%f\n",gMapAxisNO[i],theta[i]);

	//AngleOverConstrain
	int over_index=0;
	bool bOver=AngleOverConstrain(DEF_RIGHT_HAND,theta,&over_index);
	if(bOver)
	{
		printf("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_R_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_R_High[over_index]*DEF_RATIO_RAD_TO_DEG);  
	}	

	unsigned short int velocity[MAX_AXIS_NUM]={10,10,10,10,10,10,10};
	
	rt=Output_to_Dynamixel(DEF_RIGHT_HAND,theta,velocity); 

	return 0;
}



int TestOutput()
{
	int rt=0;
	unsigned short int velocity[MAX_AXIS_NUM]={10,10,10,50,50,0,50};
	float theta[7]={60*DEF_RATIO_DEG_TO_RAD,15*DEF_RATIO_DEG_TO_RAD,60*DEF_RATIO_DEG_TO_RAD,0,0,0,0};
	
	rt=Output_to_Dynamixel(DEF_RIGHT_HAND,theta,velocity); 

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
		gMapRAxisID[Index_AXIS1],(unsigned short int)Ang_pulse[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
		gMapRAxisID[Index_AXIS2],(unsigned short int)Ang_pulse[Index_AXIS2],velocity[Index_AXIS2], 
		gMapRAxisID[Index_AXIS3],(unsigned short int)Ang_pulse[Index_AXIS3],velocity[Index_AXIS3], 
		gMapRAxisID[Index_AXIS4],(unsigned short int)Ang_pulse[Index_AXIS4],velocity[Index_AXIS4], 
		gMapRAxisID[Index_AXIS4],(unsigned short int)Ang_pulse[Index_AXIS5],velocity[Index_AXIS5], 
		gMapRAxisID[Index_AXIS6],(unsigned short int)Ang_pulse[Index_AXIS6],velocity[Index_AXIS6], 
		gMapRAxisID[Index_AXIS7],(unsigned short int)Ang_pulse[Index_AXIS7],velocity[Index_AXIS7], 
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
	rt=Read_pos(DEF_RIGHT_HAND,pos_pus,DEF_UNIT_DEG);

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


void TestGripperLattePanda()
{
	char c='a';
	while(1)
	{
		Gripper_LattePanda_Hold(DEF_LEFT_HAND,true);
		getchar();

		Gripper_LattePanda_Hold(DEF_LEFT_HAND,false);
		c=getchar();

		if(c=='c')
		{
			break;
		}
	}
}


#include<fstream>
void Record_LineMove_LeftHand()
{
	//==Move to 以左肩座標系的P1_L[500,-L0,0]
	float P1_L[3]={500,-100 ,0};
	float pose_deg_L[3]={-60,0,0};
	float Rednt_alpha_L=90;
	MoveToPoint(DEF_LEFT_HAND,P1_L,pose_deg_L,Rednt_alpha_L);
	Sleep(3000);

	//==open file
	fstream file;
	file.open("D://linemove_rec.csv",ios::out|ios::trunc);
	
	//==record para
	float pos_deg[MAX_AXIS_NUM]={0};
	int rt=0;
	int n=0;
	char buffer[100];
	
	//==Move to 以左肩座標系的P2_L[500,100,0]
	//畫正方形做IK FK測試
	float P2_L[3]={500,100,0};
	float Pend_L[3]={0,0,0};
	for(int t=1;t<=100;t++)
	{
		rt=Read_pos(DEF_LEFT_HAND,pos_deg,DEF_UNIT_DEG);
		if(rt==0)
		{
			n=sprintf_s(buffer,sizeof(buffer),"%d,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,\n",t,pos_deg[Index_AXIS1],pos_deg[Index_AXIS2],pos_deg[Index_AXIS3],pos_deg[Index_AXIS4],pos_deg[Index_AXIS5],pos_deg[Index_AXIS6],pos_deg[Index_AXIS7]);
			file.write(buffer,n);
			//fprintf(file,buffer);
		}

		for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
			Pend_L[f]=P1_L[f]+(P2_L[f]-P1_L[f])*t/100; 

		MoveToPoint(DEF_LEFT_HAND,Pend_L,pose_deg_L,Rednt_alpha_L);

		Sleep(100);
	}
	
	
	file.close();

}

int Rec_Rectangle_Dual()
{
	float O_R[3]={500,-50 ,0};
	float Q_R[3]={500,-200,0};
	float R_R[3]={500,-200,-200};
	float S_R[3]={500,-50 ,-200};

	float O_L[3]={500,50 ,0};
	float Q_L[3]={500,200,0};
	float R_L[3]={500,200,-200};
	float S_L[3]={500,50 ,-200};
 
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={60,0,0};
	float Rednt_alpha_R=-90;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,0,0};
	float Rednt_alpha_L=90;


	//move to initial point
	MoveToPoint(DEF_RIGHT_HAND,O_R,pose_deg_R,Rednt_alpha_R);
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L);
	Sleep(3000);

	//==open file
	fstream file;
	file.open("D://linemove_rec.csv",ios::out|ios::trunc);
	
	//==record para
	float pos_deg[MAX_AXIS_NUM]={0};
	int rt=0;
	int n=0;
	char buffer[100];
	

	//畫正方形做IK 測試
	for(int t=0;t<=100;t++)
	{
		//Read left hand
		rt=Read_pos(DEF_LEFT_HAND,pos_deg,DEF_UNIT_DEG);
		if(rt==0)
		{
			n=sprintf_s(buffer,sizeof(buffer),"%d,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,\n",t,pos_deg[Index_AXIS1],pos_deg[Index_AXIS2],pos_deg[Index_AXIS3],pos_deg[Index_AXIS4],pos_deg[Index_AXIS5],pos_deg[Index_AXIS6],pos_deg[Index_AXIS7]);
			file.write(buffer,n);
			//fprintf(file,buffer);
		}

		if(t<=25)
		{
			for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
			{
				Pend_R[f]=O_R[f]+(Q_R[f]-O_R[f])*t/25; 
				Pend_L[f]=O_L[f]+(Q_L[f]-O_L[f])*t/25; 
			}
		}
		else if(t<=50)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=Q_R[f]+(R_R[f]-Q_R[f])*(t-25)/25;
				Pend_L[f]=Q_L[f]+(R_L[f]-Q_L[f])*(t-25)/25;
			}
		}
		else if(t<=75)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_R[f]+(S_R[f]-R_R[f])*(t-50)/25;
				Pend_L[f]=R_L[f]+(S_L[f]-R_L[f])*(t-50)/25;
			}
		}
		else 
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=S_R[f]+(O_R[f]-S_R[f])*(t-75)/25;
				Pend_L[f]=S_L[f]+(O_L[f]-S_L[f])*(t-75)/25;
			}
		}
	
		
		MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,Pend_L,pose_deg_L,Rednt_alpha_L);
		//DBGMSG(("point%d=[%f,%f,%f]\n",t,point[0],point[1],point[2]))
		//printf("Pend_R[%d]=[%f,%f,%f],Pend_L[%d]=[%f,%f,%f]\n",t,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		Sleep(100);
	}

	return 0;

}

void TestMoveAndCatch()
{
	//Move to intial
	printf("Move to home...\n");
	TestMoveToHome_Dual();

	//把此路徑分成90份
	float O_L[3]={500,50 ,0};
	float pose_deg_L[3]={-60,0,0};
	float Rednt_alpha_L=90;
	
	//Move to O_L
	printf("Move to 500,50,0...\n");
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L);
	Sleep(10000);

	//Hold gripper
	printf("Hold...\n");
	Gripper_LattePanda_Hold(DEF_LEFT_HAND,true);

	//Move to intial
	printf("Move to home...\n");
	TestMoveToHome_Dual();
	Sleep(10000);

	//Release gripper
	printf("Release...\n");
	Gripper_LattePanda_Hold(DEF_LEFT_HAND,false);

	
}

void TestStillMoving()
{
	int rt=0;
	bool stillmoving=true;
	bool btemp=true;
	for(int i=0;i<3;i++)
	{
		
		setPosition_x86(gMapRAxisID[Index_AXIS1],0, 50);
		printf("move to 0..\n");

		stillmoving=true;
		while(stillmoving)
		{
			rt=IsMoving(DEF_RIGHT_HAND,&btemp);
			if(rt==0)
			{
				stillmoving=btemp;
			}
			Sleep(300);
			printf("stillmoving to 0..\n");
		}

		setPosition_x86(gMapRAxisID[Index_AXIS1],2048, 50);
		printf("move to 2048..\n");

		stillmoving=true;
		while(stillmoving)
		{
			rt=IsMoving(DEF_RIGHT_HAND,&btemp);
			if(rt==0)
			{
				stillmoving=btemp;
			}
			Sleep(300);
			printf("stillmoving to 2048..\n");
		}
	}

}


void QPDelay_ms(int t_ms)
{
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;

	QueryPerformanceFrequency(&nFreq);
	

	QueryPerformanceCounter(&nBeginTime); 
	do
	{
		Sleep(0);
		QueryPerformanceCounter(&nEndTime);
		//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
	}
	while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart < t_ms);
}		
	

void TestOneAxisInterpolation()
{
	float Pstart=0;
	float Ptarget=4000;
	float Pend=0;
	
	//==to start point
	setPosition_x86(gMapRAxisID[Index_AXIS1],0,0);
	QPDelay_ms(2000);

	//==total time record
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBeginTime); 

	//==open file
	//fstream file;
	//file.open("D://one_axis_interpolation_10ms_in1s.csv",ios::out|ios::trunc);
	
	//==record para
	int pos_pus=0;
	int rt=0;
	int n=0;
	char buffer[100];
	

	for(int t=0;t<=10;t++)
	{
		Pend=Pstart+(Ptarget-Pstart)*t/10;

		setPosition_x86(gMapRAxisID[Index_AXIS1],(int)Pend,0);

		printf("Pend=%f\n",Pend);

		//==delay
		QPDelay_ms(10);

		//==Read right hand and record
		//pos_pus=dxl_read_word(gMapRAxisID[Index_AXIS1], PRESENT_POS);
		//if(dxl_get_result()==COMM_RXSUCCESS)
		//{
		//	n=sprintf_s(buffer,sizeof(buffer),"%d,%d\n",t,pos_pus);
		//	file.write(buffer,n);
		//}
	}

	//==delay
	//QPDelay_ms(4000);

	//==Read right hand and record
	//pos_pus=dxl_read_word(gMapRAxisID[Index_AXIS1], PRESENT_POS);
	//if(dxl_get_result()==COMM_RXSUCCESS)
	//{
	//	n=sprintf_s(buffer,sizeof(buffer),"%d,%d\n",501,pos_pus);
	//	file.write(buffer,n);
	//}

	////==close file
	//file.close();


	QueryPerformanceCounter(&nEndTime);
	double time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
	printf("total time=%f s\n",time);

}


int _tmain(int argc, _TCHAR* argv[])
{
	//===initial===///
	int rt=DXL_Initial_x86();
	if(rt==0)
	{
		printf("DXL_Initial_x86 failed\n");
		getchar();
		return 0;
	}

	
	TestOneAxisInterpolation();
	//Gripper_LattePanda_Initial();

	//Test Move And Catch
	//TestMoveAndCatch();
	//TestRectangle_LeftHand();
	
	//================//
	//==Gripper Test==//
	//================//
	//TestGripperLattePanda();
	//Gripper_LattePanda_Close();

	//==========================//
	//==GRecord_LineMove_LeftHand
	//==========================//
	//Rec_Rectangle_Dual();
	//TestRectangle_Dual();
	//ROM_Setting_Dual();

	//TestRectangle_LeftHand();

	//TestRectangle_LeftHand();
	//for(int i=0;i<4;i++)
	//{
	//TestRectangle_Dual();
	//}

	//TestMoveToHome_Dual();
	//TestOutput();

	//
	//TestAction();
	//clockTest();
	
	//TestSyncwrite();
	
	//TestReadPos();
	
	DXL_Terminate_x86();
	//Gripper_LattePanda_Close();
	getchar();
	
	return 0;
}

