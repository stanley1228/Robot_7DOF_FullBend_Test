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


#define DEF_DESCRETE_POINT 1000
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
	float vel_deg_R=13;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,0,0};
	float Rednt_alpha_L=90;
	float vel_deg_L=13;

	//move to initial point
	MoveToPoint(DEF_RIGHT_HAND,O_R,pose_deg_R,Rednt_alpha_R,vel_deg_R);
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
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
		MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
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
	float vel_deg=13;
	

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
		MoveToPoint(DEF_RIGHT_HAND,point,pose_deg,-90,vel_deg);
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
	float vel_deg_L=13;
	//move to initial point
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
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
		MoveToPoint(DEF_LEFT_HAND,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
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
	float vel_deg_L=13;
	MoveToPoint(DEF_LEFT_HAND,P1_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
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

		MoveToPoint(DEF_LEFT_HAND,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);

		Sleep(100);
	}
	
	
	file.close();

}
void WaitMotionDone()
{
	int rt=0;
	bool stillmoving=true;
	bool btemp=true;
	int cnt_err=0;

	//wait it
	while(stillmoving)
	{
		rt=IsMoving(DEF_LEFT_HAND,&btemp);
		if(rt==0)
		{
			stillmoving=btemp;
		}
		else
		{
			cnt_err++;//communication err
			if(cnt_err==10)
			{
				printf("get moving status error\n");	
				return;
			}
		}
		Sleep(500);
		printf("stillmoving..\n");	
	}
	
}

void WaitMotionDoneDual()
{
	int rt_R=0,rt_L=0;
	bool stillmoving=true;
	bool btemp_L=true,btemp_R=true;
	int cnt_err=0;

	//wait it
	while(stillmoving)
	{
		rt_L=IsMoving(DEF_LEFT_HAND,&btemp_L);
		rt_R=IsMoving(DEF_RIGHT_HAND,&btemp_R);
		if((rt_R==0) && (rt_L==0))
		{
			stillmoving=btemp_L | btemp_R;
		}
		else
		{
			cnt_err++;//communication err
			if(cnt_err==10)
			{
				printf("get moving status error\n");	
				getchar();
			}
		}
		Sleep(500);
		printf("stillmoving..\n");	
	}
	
}

int Rec_Rectangle_Dual()
{
	//float O_R[3]={400,-50 ,-100};
	//float Q_R[3]={400,-200,-100};
	//float R_R[3]={400,-200,-300};
	//float S_R[3]={400,-50 ,-300};

	//float O_L[3]={400,50 ,-100};
	//float Q_L[3]={400,200,-100};
	//float R_L[3]={400,200,-300};
	//float S_L[3]={400,50 ,-300};

	
	float O_R[3]={500,-50,0};
	float Q_R[3]={500,-200,0};
	float R_R[3]={500,-200,-200};
	float S_R[3]={500,-50,-200};

	float O_L[3]={500,50,0};
	float Q_L[3]={500,200,0};
	float R_L[3]={500,200,-200};
	float S_L[3]={500,50,-200};
 
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={60,0,0};
	float Rednt_alpha_R=-90;
	float vel_deg_R=13;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,0,0};

	float Rednt_alpha_L=90;
	float vel_deg_L=13;

	int rt=0;
	bool stillmoving=true;
	bool btemp=true;
	int cnt_err=0;

	//move to initial point
	MoveToPoint(DEF_RIGHT_HAND,O_R,pose_deg_R,Rednt_alpha_R,vel_deg_R);
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
	
	//wait it
	while(stillmoving)
	{
		rt=IsMoving(DEF_LEFT_HAND,&btemp);
		if(rt==0)
		{
			stillmoving=btemp;
		}
		else
		{
			cnt_err++;//communication err
			if(cnt_err==10) break;
		}
		Sleep(100);
		printf("stillmoving..\n");	
	}


	//==open file
	fstream file;
	file.open("D://rectangle_rec.csv",ios::out|ios::trunc);
	
	//==record para
	float pos_deg[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok[MAX_AXIS_NUM]={0};
	
	int n=0;
	char buffer[100];
	

	//畫正方形做IK 測試
	for(int t=1;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=0.25*DEF_DESCRETE_POINT)
		{
			for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
			{
				Pend_R[f]=O_R[f]+(Q_R[f]-O_R[f])*t/((float)0.25*DEF_DESCRETE_POINT); 
				Pend_L[f]=O_L[f]+(Q_L[f]-O_L[f])*t/((float)0.25*DEF_DESCRETE_POINT); 
			}
		}
		else if(t<=0.5*DEF_DESCRETE_POINT)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=Q_R[f]+(R_R[f]-Q_R[f])*(t-(float)0.25*DEF_DESCRETE_POINT)/((float)0.25*DEF_DESCRETE_POINT);
				Pend_L[f]=Q_L[f]+(R_L[f]-Q_L[f])*(t-(float)0.25*DEF_DESCRETE_POINT)/((float)0.25*DEF_DESCRETE_POINT);
			}
		}
		else if(t<=0.75*DEF_DESCRETE_POINT)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_R[f]+(S_R[f]-R_R[f])*(t-(float)0.5*DEF_DESCRETE_POINT)/((float)0.25*DEF_DESCRETE_POINT);
				Pend_L[f]=R_L[f]+(S_L[f]-R_L[f])*(t-(float)0.5*DEF_DESCRETE_POINT)/((float)0.25*DEF_DESCRETE_POINT);
			}
		}
		else 
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=S_R[f]+(O_R[f]-S_R[f])*(t-(float)0.75*DEF_DESCRETE_POINT)/((float)0.25*DEF_DESCRETE_POINT);
				Pend_L[f]=S_L[f]+(O_L[f]-S_L[f])*(t-(float)0.75*DEF_DESCRETE_POINT)/((float)0.25*DEF_DESCRETE_POINT);
			}
		}
	
		vel_deg_L=0;//MAX speed
		MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
		//printf("Pend_R[%d]=[%4.2f,%4.2f,%4.2f],Pend_L[%d]=[%4.2f,%4.2f,%4.2f]\n",t,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],t,Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		
		//==Delay
		//QPDelay_ms(10);

		//==Read left hand
		rt=Read_pos(DEF_LEFT_HAND,pos_deg,DEF_UNIT_DEG);
		if(rt==0)
		{
			for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			{
				printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg[i]);
			}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg[Index_AXIS1],pos_deg[Index_AXIS2],pos_deg[Index_AXIS3],pos_deg[Index_AXIS4],pos_deg[Index_AXIS5],pos_deg[Index_AXIS6],pos_deg[Index_AXIS7]);
			file.write(buffer,n);
			
			memcpy(pos_deg_last_ok,pos_deg,sizeof(pos_deg_last_ok));
		}
		else //讀取失敗時，拿前一筆來補
		{
			for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			{
				printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok[i]);
			}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_last_ok[Index_AXIS1],pos_deg_last_ok[Index_AXIS2],pos_deg_last_ok[Index_AXIS3],pos_deg_last_ok[Index_AXIS4],pos_deg_last_ok[Index_AXIS5],pos_deg_last_ok[Index_AXIS6],pos_deg_last_ok[Index_AXIS7]);
			file.write(buffer,n);
		}
	}

	return 0;

}

int Get_Linear_fun_point(float t,float tk,float TotalTime,float *Seqt,float (*SeqPt)[3],float (*SeqVel)[3],float (*SeqAcc)[3],float *P_out)
{
	int Pseg=0;
	int VSeg=0;
	int ASeg=0;
	float P[3]={0};
	int f=0;

	//right
	int Pcnt_R=0;//輸出總點數
	
	
	if(t<tk)                //parabolic
	{
		Pseg=0;
		VSeg=0;
		ASeg=0;

		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[Pseg][f]*(t-0)+(float)0.5*SeqAcc[ASeg][f]*pow((t-0),2);  
	}
	else if (t<Seqt[1]-0.5*tk)   //linear
	{
		Pseg=0;
		VSeg=1;
		ASeg=1;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[VSeg][f]*(float)(t-0.5*tk);  
	}
	else if(t<Seqt[1]+0.5*tk)//parabolic
	{
		Pseg=0;
		VSeg=1;    
		ASeg=1;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[VSeg][f]*(float)((t-0.5*tk))+(float)(0.5*SeqAcc[ASeg][f]*pow((t-(Seqt[1]-0.5*tk)),2));  
	}
	else if (t<Seqt[2]-0.5*tk)      //linear 
	{
		Pseg=1;
		VSeg=2;    
		ASeg=1;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[VSeg][f]*(t-Seqt[1]);
	}
	else if( t< Seqt[2]+0.5*tk) //parabolic
	{
		Pseg=1;
		VSeg=2;   
		ASeg=2;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+(float)SeqVel[VSeg][f]*(t-Seqt[1])+(float)(0.5*SeqAcc[ASeg][f]*pow((t-(Seqt[2]-0.5*tk)),2));  
	}
	else if (t<Seqt[3]-0.5*tk)      //linear 
	{
		Pseg=2;
		VSeg=3;    
		ASeg=2;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[VSeg][f]*(t-Seqt[2]);
	}
	else if (t< Seqt[3]+0.5*tk) //parabolic
	{
		Pseg=2;
		VSeg=3;   
		ASeg=3;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+(float)SeqVel[VSeg][f]*(t-Seqt[2])+(float)(0.5*SeqAcc[ASeg][f]*pow((t-(Seqt[3]-0.5*tk)),2));          
	}
            
	else if (t< Seqt[4]-tk) //linear before final
	{
		Pseg=3;
		VSeg=4;   
		ASeg=3;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[VSeg][f]*(t-Seqt[3]);
	}
	else if (t< Seqt[4])//parabolic  final
	{
		Pseg=3;
		VSeg=4;   
		ASeg=4;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f]+SeqVel[VSeg][f]*(t-Seqt[3])+(float)0.5*SeqAcc[ASeg][f]*pow((t-(Seqt[4]-tk)),2);  
	}
	else if(t==TotalTime)
	{
		Pseg=4;
		for(f=0;f<3;f++)
			P[f]=SeqPt[Pseg][f];  
	}

	memcpy(P_out,P,sizeof(P));

	return 0;
}


void Rec_Rectangle_Linear_function_Dual()
{
	const int RowSize=5;
	const int ColSize=3;
	float SeqPt_R[RowSize][ColSize]={{500,-50,0},
						{500,-200,0},
						{500,-200,-200},
						{500,-50,-200},
						{500,-50,0}};

	float SeqPt_L[RowSize][ColSize]={{500,50,0},
						{500,200,0},
						{500,200,-200},
						{500,50,-200},
						{500,50,0}};

	//float Seqt_R[RowSize]={0,2,4,6,8};
	//float TotalTime_R=8;
	//float tk_R=0.5;//二次曲線的時間
	float Seqt_R[RowSize]={0,5,15,20,40};
	float TotalTime_R=40;
	float tk_R=1.5;//二次曲線的時間

	//float Seqt_L[RowSize]={0,2,4,6,8};
	//float TotalTime_L=8;
	//float tk_L=0.5;//二次曲線的時間
	float Seqt_L[RowSize]={0,5,15,20,40};
	float TotalTime_L=40;
	float tk_L=1.5;//二次曲線的時間

	float SeqVel_R[RowSize+1][ColSize]={0};
	float SeqAcc_R[RowSize][ColSize]={0};

	float SeqVel_L[RowSize+1][ColSize]={0};
	float SeqAcc_L[RowSize][ColSize]={0};

	//==計算Cartesian Space下各段速度==%
	//right
	int i=0;
	int f=0;
	for(i=0;i<RowSize;i++)
	{
		if(i==0)        //V0
		{
			for(f=0;f<3;f++)
				SeqVel_R[i][f]=0;
		}
		else if (i==1 || i==(RowSize-1))  //V1 or Vf前一筆
		{
			for(f=0;f<3;f++)
			{
				SeqVel_R[i][f]=(SeqPt_R[i][f]-SeqPt_R[i-1][f])/(float)(Seqt_R[i]-Seqt_R[i-1]-0.5*tk_R); 
			
			}
		}
		else if (i==RowSize-1)
		{
			for(f=0;f<3;f++)
				SeqVel_R[i][f]=0;
		}
		else
		{
			for(f=0;f<3;f++)
				SeqVel_R[i][f]=(SeqPt_R[i][f]-SeqPt_R[i-1][f])/(Seqt_R[i]-Seqt_R[i-1]);   
		}
	}

	//left
	for(i=0;i<RowSize;i++)
	{
		if(i==0)        //V0
		{
			for(f=0;f<3;f++)
				SeqVel_L[i][f]=0;
		}
		else if (i==1 || i==(RowSize-1))  //V1 or Vf前一筆
		{
			for(f=0;f<3;f++)
			{
				SeqVel_L[i][f]=(SeqPt_L[i][f]-SeqPt_L[i-1][f])/(float)(Seqt_L[i]-Seqt_L[i-1]-0.5*tk_L); 
			
			}
		}
		else if (i==RowSize-1)
		{
			for(f=0;f<3;f++)
				SeqVel_L[i][f]=0;
		}
		else
		{
			for(f=0;f<3;f++)
				SeqVel_L[i][f]=(SeqPt_L[i][f]-SeqPt_L[i-1][f])/(Seqt_L[i]-Seqt_L[i-1]);   
		}
	}
	//==計算Cartesian Space各段加速度==//
	//right
	for(i=0;i<RowSize;i++)
	{
		for(f=0;f<3;f++)
			SeqAcc_R[i][f]=(SeqVel_R[i+1][f]-SeqVel_R[i][f])/tk_R;
	}
	//left
	for(i=0;i<RowSize+1;i++)
	{
		for(f=0;f<3;f++)
			SeqAcc_L[i][f]=(SeqVel_L[i+1][f]-SeqVel_L[i][f])/tk_L;
	}

	//==手臂參數設定==//
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={60,0,0};
	float Rednt_alpha_R=-90;
	float vel_deg_R=13;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,0,0};
	float Rednt_alpha_L=90;
	float vel_deg_L=13;

	int rt=0;
	bool stillmoving=true;
	bool btemp=true;
	int cnt_err=0;

	//move to initial point
	MoveToPoint(DEF_RIGHT_HAND,SeqPt_R[0],pose_deg_R,Rednt_alpha_R,vel_deg_R);
	MoveToPoint(DEF_LEFT_HAND,SeqPt_L[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);
	
	//wait it
	WaitMotionDoneDual();


	//==open file
	fstream fileR;
	fstream fileL;
	fileR.open("D://Rec_Linear_fun_R.csv",ios::out|ios::trunc);
	fileL.open("D://Rec_Linear_fun_L.csv",ios::out|ios::trunc);
	//==record para
	float pos_deg_R[MAX_AXIS_NUM]={0};
	float pos_deg_L[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
	int n=0;
	char buffer[100];



	//==使用linear fuction 規劃方形各段軌跡  共5點  4段直線斷  5段二次段==//
	float DEF_CYCLE_TIME=0.025f;
	float t=0;
	
	
	for(t=0;t<TotalTime_R;t+=DEF_CYCLE_TIME)
	{
		//int Get_Linear_fun_point(float t,float tk,float *Seqt,float *SeqPt[3],float *SeqVel[3],float *SeqAcc[3],float *P_out);

		Get_Linear_fun_point(t,tk_R,TotalTime_R,Seqt_R,SeqPt_R,SeqVel_R,SeqAcc_R,Pend_R);//right
		Get_Linear_fun_point(t,tk_L,TotalTime_L,Seqt_L,SeqPt_L,SeqVel_L,SeqAcc_L,Pend_L);//left

		//==確認軌跡點==//
		//n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",t,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z]);
		//fileR.write(buffer,n);

		//n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",t,Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		//fileL.write(buffer,n);

		vel_deg_R=15;
		vel_deg_L=15;
		//MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);  //22ms
		printf("Pend_R=[%4.2f,%4.2f,%4.2f],Pend_L=[%4.2f,%4.2f,%4.2f]\n",Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		
		

		//==Read right hand
		//rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);

		//if(rt==0)
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
		//	fileR.write(buffer,n);
		//	
		//	memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
		//}
		//else //讀取失敗時，拿前一筆來補
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
		//	fileR.write(buffer,n);
		//}

		////==Read left hand
		//rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);

		//if(rt==0)
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]);
		//	fileL.write(buffer,n);
		//	
		//	memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));
		//}
		//else //讀取失敗時，拿前一筆來補
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
		//	fileL.write(buffer,n);
		//}
	}
	fileR.close();
	fileL.close();
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
	float vel_deg_L=13; //deg/s
	//Move to O_L
	printf("Move to 500,50,0...\n");
	MoveToPoint(DEF_LEFT_HAND,O_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);
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



void TestGetDrink()
{
	float R_p0[3]={300,-300,-200};//起始點
	float L_p0[3]={300,200,-200};//起始點

	float R_p1[3]={500,50,-210};//門把位置
	float L_p1[3]={300,200,-200};//左手不動

	float Rp2x = 500;
	float Rp2y = (50-650)*0.5;
	float Rp2z = -210;
	float rR2 = 50-Rp2y;
	float R_p2[3] = {Rp2x,Rp2y,Rp2z};//拉門半徑圓心
	float R_pp2[3]={150,-300,-210};
	float L_p2[3]={300,200,-200};//左手不動


	float R_p3[3]={150,-300,-210};//右手不動
	float L_p3[3]={400,200,-200};//左手往前

	float R_p4[3]={150,-300,-210};//右手不動
	float L_p4[3]={400,100,-200};//左手往右

	float R_p5[3]={150,-300,-210};
	float L_p5[3]={520,-60,-200};//左手移動到綠色飲料  % pick the object
	//====================

	float R_p6[3]={150,-300,-210};//右手不動
	float L_p6[3]={400,100,-200};//左手退回

	float R_p7[3]={150,-300,-210};//右手不動
	float L_p7[3]={400,200,-210};//左手下降

	float Rp8x = 500;
	float Rp8y = (50-650)*0.5;
	float Rp8z = -210;
	float rR8 = 50-Rp8y;
	float R_p8[3] = {Rp8x,Rp8y,Rp8z};
	float R_pp8[3] = {500,50,-210};//門把位置
	float L_p8[3] = {400,200,-210};//左手不動

	float R_p9[3]={500,50,-210};
	float L_p9[3]={300,200,-210};//左手退回

	float R_p10[3]={200,0,-300};
	float L_p10[3]={200,0,-300};

 
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={60,0,0};
	float Rednt_alpha_R=-60;
	float vel_deg_R=13;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-50,0,0};
	float Rednt_alpha_L=30;
	float vel_deg_L=13;

	//==open file
	fstream fileR;
	fstream fileL;
	fileR.open("D://GetDrinkRec_R.csv",ios::out|ios::trunc);
	fileL.open("D://GetDrinkRec_L.csv",ios::out|ios::trunc);
	//==record para
	float pos_deg_R[MAX_AXIS_NUM]={0};
	float pos_deg_L[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
	
	int n=0;
	char buffer[100];
	int rt=0;

	//move to initial point
	//MoveToPoint(DEF_RIGHT_HAND,R_p0,pose_deg_R,Rednt_alpha_R,vel_deg_R);
	//MoveToPoint(DEF_LEFT_HAND,L_p0,pose_deg_L,Rednt_alpha_L,vel_deg_L);

	printf("move to p0..\n");
	//WaitMotionDoneDual();


	//開冰箱拿飲料流程
	
	//各段點數
	int BonPt[10]={0};//BonPt=boundary point	區段分界點		
	//int SegPt[10]={600,2000,300,200,800,800,300,400,400,400};  //SegPt=segment point 各區段點數
	
	//SegPt=[600 2000 300 200 800 800 300 400 400 400];  %SegPt=segment point 各區段點數
	int SegPt[10]={6,20,3,2,8,8,3,4,4,4};  //SegPt=segment point 各區段點數
	//計算區段分界點
	for(int i=0;i<10;i++)
	{
		for (int j=0;j<=i;j++)
		{
			BonPt[i]= BonPt[i]+SegPt[j];
		}
	}	    
	

	//總流程點數
	int TotalPt=0;
	for(int i=0;i<10;i++)
	{
		TotalPt+=SegPt[i];
	}
	
	for(int t=1;t<=TotalPt;t++)
	{
		if(t<=BonPt[0])//右手移動到冰箱門把
		{	
			for(int f=0;f<3;f++)   //對xyz座標分別運算 f=x,y,z
			{
				Pend_R[f]=R_p0[f]+(R_p1[f]-R_p0[f])*t/(float)SegPt[0]; 
				Pend_L[f]=L_p0[f]+(L_p1[f]-L_p0[f])*t/(float)SegPt[0]; 
			}
		}
		else if(t<=BonPt[1])//右手開冰箱門
		{
			if(t==(BonPt[0]+1))
			{
				//WaitMotionDoneDual();
				//Gripper_LattePanda_Hold(DEF_RIGHT_HAND,true);
			}

			// R_p2為拉門半徑圓心
			Pend_R[DEF_X]=R_p2[DEF_X]+rR2*((float)sin(0.5*DEF_PI*(t-BonPt[0])/(SegPt[1])+ DEF_PI));
			Pend_R[DEF_Y]=R_p2[DEF_Y]+rR2*((float)cos(0.5*DEF_PI*(t-BonPt[0])/(SegPt[1])));
			Pend_R[DEF_Z]=R_p2[DEF_Z]+rR2*0;	

			for(int f=0;f<3;f++)  
			{
				Pend_L[f]=L_p1[f]+(L_p2[f]-L_p1[f])*(t-BonPt[0])/((float)SegPt[1]); 
			}
		}
		else if(t<=BonPt[2])//左手往x移動100
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_pp2[f]+(R_p3[f]-R_pp2[f])*(t-(float)BonPt[1])/((float)SegPt[2]);
				Pend_L[f]=L_p2[f]+(L_p3[f]-L_p2[f])*(t-(float)BonPt[1])/((float)SegPt[2]);
			}
		}
		else if(t<=BonPt[3])//左手往y移動-100
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p3[f]+(R_p4[f]-R_p3[f])*(t-(float)BonPt[2])/((float)SegPt[3]);
				Pend_L[f]=L_p3[f]+(L_p4[f]-L_p3[f])*(t-(float)BonPt[2])/((float)SegPt[3]);
			}		
		}
		else if(t<=BonPt[4])//左手移動到飲料點
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p4[f]+(R_p5[f]-R_p4[f])*(t-(float)BonPt[3])/((float)SegPt[4]);
				Pend_L[f]=L_p4[f]+(L_p5[f]-L_p4[f])*(t-(float)BonPt[3])/((float)SegPt[4]);
			}	
		}
		else if(t<=BonPt[5])//左手退回
		{
			if(t==(BonPt[4]+1))//左手夾飲料
			{
				//WaitMotionDoneDual();
				//Gripper_LattePanda_Hold(DEF_LEFT_HAND,true);
			}

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p5[f]+(R_p6[f]-R_p5[f])*(t-(float)BonPt[4])/((float)SegPt[5]);
				Pend_L[f]=L_p5[f]+(L_p6[f]-L_p5[f])*(t-(float)BonPt[4])/((float)SegPt[5]);
			}	
		}
		else if(t<=BonPt[6])//左手往z移動-10
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p6[f]+(R_p7[f]-R_p6[f])*(t-(float)BonPt[5])/((float)SegPt[6]);
				Pend_L[f]=L_p6[f]+(L_p7[f]-L_p6[f])*(t-(float)BonPt[5])/((float)SegPt[6]);
			}	
		}
		else if(t<=BonPt[7])//右手關冰箱門
		{
			// R_p8為關門半徑圓心
			Pend_R[DEF_X]=R_p8[DEF_X]+rR8*((float)cos(0.5*DEF_PI*(t-BonPt[6])/(SegPt[7])+ DEF_PI));
			Pend_R[DEF_Y]=R_p8[DEF_Y]+rR8*((float)sin(0.5*DEF_PI*(t-BonPt[6])/(SegPt[7])));
			Pend_R[DEF_Z]=R_p8[DEF_Z]+rR8*0;	

			for(int f=0;f<3;f++)  
			{
				Pend_L[f]=L_p7[f]+(L_p8[f]-L_p7[f])*(t-(float)BonPt[6])/((float)SegPt[7]);
			}	
		}
		else if(t<=BonPt[8])//左手往x移動-100
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_pp8[f]+(R_p9[f]-R_pp8[f])*(t-(float)BonPt[7])/((float)SegPt[8]);
				Pend_L[f]=L_p8[f]+(L_p9[f]-L_p8[f])*(t-(float)BonPt[7])/((float)SegPt[8]);
			}	
		}
		else if(t<=BonPt[9])//右手靠上去拿飲料
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p9[f]+(R_p10[f]-R_p9[f])*(t-(float)BonPt[8])/((float)SegPt[9]);
				Pend_L[f]=L_p9[f]+(L_p10[f]-L_p9[f])*(t-(float)BonPt[8])/((float)SegPt[9]);
			}	
		}

		//vel_deg_R=20;//MAX speed
		//vel_deg_L=20;//MAX speed
		//MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms

		printf("Pend_R[%d]=[%f,%f,%f],Pend_L[%d]=[%f,%f,%f]\n",t,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);

		n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f\n",Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z]);
		fileR.write(buffer,n);

		n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f\n",Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		fileL.write(buffer,n);



		////==Read right hand
		//rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);

		//if(rt==0)
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
		//	fileR.write(buffer,n);
		//	
		//	memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
		//}
		//else //讀取失敗時，拿前一筆來補
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
		//	fileR.write(buffer,n);
		//}

		////==Read left hand
		//rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);

		//if(rt==0)
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]);
		//	fileL.write(buffer,n);
		//	
		//	memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));
		//}
		//else //讀取失敗時，拿前一筆來補
		//{
		//	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		//	{
		//		printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
		//	}
		//	printf("\n");

		//	n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
		//	fileL.write(buffer,n);
		//}
	}	

	fileR.close();
	fileL.close();
	

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
	//Rec_Rectangle_Linear_function_Dual();


	//TestGetDrink();
	//PID_Setting_Dual();
	//getchar();
	//TestOneAxisInterpolation();
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
	ROM_Setting_Dual();

	//TestRectangle_LeftHand();

	//TestRectangle_LeftHand();
	//for(int i=0;i<4;i++)
	//{
	//TestRectangle_Dual();
	//}

	//getchar();
	//TestMoveToHome_Dual();
	//TestOutput();

	//
	//TestAction();
	//clockTest();
	
	//TestSyncwrite();
	
	//TestReadPos();
	
	//DXL_Terminate_x86();
	//Gripper_LattePanda_Close();
	getchar();
	
	return 0;
}

