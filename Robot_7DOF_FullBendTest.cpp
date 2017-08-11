// Robot_7DOF_FullBendTest.cpp : �w�q�D���x���ε{�����i�J�I�C
//

#include "stdafx.h"
#include "Robot_7DOF_FB.h"
#include "dynamixel.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>
//#include <iostream>
#include <windows.h> //sleep�ϥ�
using namespace std;


#define DEF_DESCRETE_POINT 1000


#define CHECK_CARTESIAN_PATH 
//#define GRIPPER_ON_LATTE
//#define MOVETOPOINT_DUAL
//#define CHECK_JOINT_PATH   //MoveToPoint_Dual�禡 ����]�ndef
//#define MOVE_TO_INITIAL_POINT
//#define RECORD_JOINT_ANGLE
#define DEF_WAIT_ENTER
#ifdef  CHECK_JOINT_PATH
fstream gfileR;
fstream gfileL;
#endif

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

	//�e����ΰ�IK ����
	for(int t=0;t<=90;t++)
	{
		if(t<=25)
		{
			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
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
	//�⦹���|����90��
	float O[3]={500,-50,0};  
	float Q[3]={500,-200,0};
	float R[3]={500,-200,-150};
	float S[3]={500,-50,-150};
	float point[3]={0,0,0};
	float pose_deg[3]={30,0,0};
	float vel_deg=13;
	

	 //�e����ΰ�IK FK����
	for(int t=1;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=25)
		{
			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
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
	//�⦹���|����90��
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


	 //�e����ΰ�IK FK����
	for(int t=1;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=25)
		{
			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
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
	int delay_ms=1200;
	while(1)
	{
		Gripper_LattePanda_Hold(DEF_RIGHT_HAND,true,delay_ms);
		getchar();

		Gripper_LattePanda_Hold(DEF_RIGHT_HAND,false,delay_ms);
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
	//==Move to �H���Ӯy�Шt��P1_L[500,-L0,0]
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
	
	//==Move to �H���Ӯy�Шt��P2_L[500,100,0]
	//�e����ΰ�IK FK����
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

		for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
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
	

	//�e����ΰ�IK ����
	for(int t=1;t<=DEF_DESCRETE_POINT;t++)
	{
		if(t<=0.25*DEF_DESCRETE_POINT)
		{
			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
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
		else //Ū�����ѮɡA���e�@���Ӹ�
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
	int Pcnt_R=0;//��X�`�I��
	
	
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
	//float tk_R=0.5;//�G�����u���ɶ�
	float Seqt_R[RowSize]={0,5,15,20,25};
	float TotalTime_R=25;
	float tk_R=1.5;//�G�����u���ɶ�

	//float Seqt_L[RowSize]={0,2,4,6,8};
	//float TotalTime_L=8;
	//float tk_L=0.5;//�G�����u���ɶ�
	float Seqt_L[RowSize]={0,5,15,20,25};
	float TotalTime_L=25;
	float tk_L=1.5;//�G�����u���ɶ�

	float SeqVel_R[RowSize+1][ColSize]={0};
	float SeqAcc_R[RowSize][ColSize]={0};

	float SeqVel_L[RowSize+1][ColSize]={0};
	float SeqAcc_L[RowSize][ColSize]={0};

	//==�p��Cartesian Space�U�U�q�t��==%
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
		else if (i==1 || i==(RowSize-1))  //V1 or Vf�e�@��
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
		else if (i==1 || i==(RowSize-1))  //V1 or Vf�e�@��
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
	//==�p��Cartesian Space�U�q�[�t��==//
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

	//==���u�ѼƳ]�w==//
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

	//==�ϥ�linear fuction �W����ΦU�q�y��  �@5�I  4�q���u�_  5�q�G���q==//
	//float DEF_CYCLE_TIME=0.022f; //��Ūfeedback�� �j��22m
	float DEF_CYCLE_TIME=0.056f; //��Ūfeedback�ɻݭn 22+17+17=56ms 19+16+16=51ms
	//float DEF_CYCLE_TIME=0.040f; //�氦��S������Ū��
	//float DEF_CYCLE_TIME=0.03f; //��Ūfeedback�� �j��22m ���Lcycle time�]�j�@�I���|���b���Iover load���p�Acycltime���ڤj�A��ܷ|�b��F�e�U�s�R�O
	float t=0;
	
	//LARGE_INTEGER nFreq;
	//LARGE_INTEGER nBeginTime;
	//LARGE_INTEGER nEndTime;
	//QueryPerformanceFrequency(&nFreq);

	//QueryPerformanceCounter(&nBeginTime); 
	//QueryPerformanceCounter(&nEndTime);
	//double time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart;
	//printf("total time=%f s\n",time);

	for(t=0;t<TotalTime_R;t+=DEF_CYCLE_TIME)
	{
		Get_Linear_fun_point(t,tk_R,TotalTime_R,Seqt_R,SeqPt_R,SeqVel_R,SeqAcc_R,Pend_R);//right  1us
		Get_Linear_fun_point(t,tk_L,TotalTime_L,Seqt_L,SeqPt_L,SeqVel_L,SeqAcc_L,Pend_L);//left 1us

		//==�T�{�y���I==//
		//n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",t,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z]);
		//fileR.write(buffer,n);

		//n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",t,Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		//fileL.write(buffer,n);

		vel_deg_R=30;
		vel_deg_L=30;
		
		MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20~22ms
		printf("Pend_R=[%4.1f,%4.1f,%4.1f],Pend_L=[%4.1f,%4.1f,%4.1f]\n",Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		
		//==Read right hand
		rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);  //Ū����246 mS  ���\17ms
		
		if(rt==0)
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",t,pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
			fileR.write(buffer,n);
			
			memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
		}
		else //Ū�����ѮɡA���e�@���Ӹ�
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",t,pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
			fileR.write(buffer,n);
		}

		//==Read left hand
		rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);

		if(rt==0)
		{		
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)  //�L��0.6 ms
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
			//}
		
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",t,pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]); //33us
			fileL.write(buffer,n); //5us
			memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));//1 us
		}
		else //Ū�����ѮɡA���e�@���Ӹ�
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",t,pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
			fileL.write(buffer,n);
		}
	}
	fileR.close();
	fileL.close();
}

void TestMoveAndCatch()
{
	//Move to intial
	printf("Move to home...\n");
	TestMoveToHome_Dual();

	//�⦹���|����90��
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
	int delay_ms=500;
	Gripper_LattePanda_Hold(DEF_LEFT_HAND,true,delay_ms);

	//Move to intial
	printf("Move to home...\n");
	TestMoveToHome_Dual();
	Sleep(10000);

	//Release gripper
	printf("Release...\n");
	Gripper_LattePanda_Hold(DEF_LEFT_HAND,false,delay_ms);

	
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
	//==���|�I
	float R_p[7][3]={   
		{300, -300, -200},	//�_�l�I
		{480,	80, -100},	//�����������A��m
		{130, -270, -100},	//����}�����A��m
		{480,	80, -100},	//�����������A��m
		{410,	 0, -80},	//�k��q�����������A��m�h�^���~1
		{350,	 0, -80},	//�k��q�����������A��m�h�^���~2
		{200, -100, -200}};//�̫����ⴤ���Ʀ�m

	float L_p[8][3]={  
		{300, 200, -200},	//�_�l�I
		{350, 150, -150},	//���⩹�񶼮��I�e�i���~1
		{400, 30, -80},	//���⩹�񶼮��I�e�i���~2
		{570, -60, -40},	//�񶼮��I
		{430, 30, -40},	//����q�񶼮��I�h�^���~1
		{300, 150, -200},	//����q�񶼮��I�h�^���~2
		{300, 200, -200},	//����q�񶼮��I�h�^���~3
		{200, 100, -200}};	//�̫����ⴤ���Ʀ�m

	//==�}�������|�b�|�ζ��
	float rDoorPath = 50-(50-650)*0.5;
	float Cen_DoorPath[3] = {500,(50-650)*0.5, -100}; //�Ԫ��b�|��� center of open door path

	//==���ɶ��Эp���`�ɶ�
	const int SegTimeSize=16;
	float Seqt[SegTimeSize]= {0};//����ɶ��Эp Sequence time

	int i=0;
	Seqt[i]=0;
	i=i+1;
	Seqt[i]=20;//�k�⩹�����������A��m����
	i=i+1;
	Seqt[i]=24;//�k�⧨��hold
	i=i+1;
	Seqt[i]=58;//�k�⩹����}�����A��m����
	i=i+1;
	Seqt[i]=60;//���⩹�񶼮��I�e�i���~1
	i=i+1;
	Seqt[i]=75;//���⩹�񶼮��I�e�i���~2
	i=i+1;
	Seqt[i]=90;//���⩹�񶼮��I
	i=i+1;
	Seqt[i]=95;//���⧨��hold
	i=i+1;
	Seqt[i]=110;//����q�񶼮��I�h�^���~1
	i=i+1;
	Seqt[i]=120;//����q�񶼮��I�h�^���~2
	i=i+1;
	Seqt[i]=150;//�k�⩹�����������A��m����
	i=i+1;
	Seqt[i]=155;//�k�⧨��rlease
	i=i+1;
	Seqt[i]=175;//�k��q�����������A��m�h�^���~1
	i=i+1;
	Seqt[i]=195;//�k��q�����������A��m�h�^���~2
	i=i+1;
	Seqt[i]=215;//����q�񶼮��I�h�^���~3
	i=i+1;
	Seqt[i]=230;//���k��^��̫����ⴤ���Ʀ�m

	float TotalTime=230;
	float SeqItv[SegTimeSize-1]={0};//Sequence  invterval
	
	for(i=0;i<SegTimeSize-1;i++)
	{
		SeqItv[i]=Seqt[i+1]-Seqt[i];
	}

	//==�y�{�ݥ��ܼ�
	//float CycleT=0.1f;
#ifndef RECORD_JOINT_ANGLE
	float CycleT=0.04f; //��Ūfeedback�� �j��22m
#else
	float CycleT=0.1f; //��Ūfeedback�ɻݭn 22+17+17=56ms 19+16+16=51ms
#endif

	//float CycleT=0.040f; //�氦��S������Ū��
	//float CycleT=0.03f; //��Ūfeedback�� �j��22m ���Lcycle time�]�j�@�I���|���b���Iover load���p�Acycltime���ڤj�A��ܷ|�b��F�e�U�s�R�O

	float Itv=0; //interval of the segment
	float t=0; //t of each segment (0~interval)
	int GripperAlreadyAct=0;// already send command to gripper
	

	//==variable for reocrd file==//
	//open file
#ifdef	RECORD_JOINT_ANGLE
	fstream fileR;
	fstream fileL;
	fileR.open("D://GetDrinkJointAngle_R.csv",ios::out|ios::trunc);
	fileL.open("D://GetDrinkJointAngle_L.csv",ios::out|ios::trunc);
#endif

#ifdef CHECK_CARTESIAN_PATH
	fstream fileR;
	fstream fileL;
	fileR.open("D://GetDrinkCartesian_L.csv",ios::out|ios::trunc);
	fileL.open("D://GetDrinkCartesian_L.csv",ios::out|ios::trunc);

#endif

#ifdef	CHECK_JOINT_PATH
	gfileR.open("D://IK_CMD_R.csv",ios::out|ios::trunc);
	gfileL.open("D://IK_CMD_L.csv",ios::out|ios::trunc);
#endif
	
	
	//==record para==//
	float pos_deg_R[MAX_AXIS_NUM]={0};
	float pos_deg_L[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
	int n=0;
	int rt=0;
	char buffer[100];
	//==Robotic arm pose==//
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={60,0,0};
	float Rednt_alpha_R=-60;
	float vel_deg_R=10;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-50,0,0};
	float Rednt_alpha_L=50;
	float vel_deg_L=5;

	//==move to initial point==//
#ifdef	MOVE_TO_INITIAL_POINT
	//MoveToPoint_Dual(R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
	MoveToPoint(DEF_RIGHT_HAND,R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R);
	MoveToPoint(DEF_LEFT_HAND,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);
	printf("move to p0..\n");
	WaitMotionDoneDual();
#endif
	
	printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
	getchar();
#endif
	//==���Ƭy�{�}�l==//
	float abst=0.0;
	for(abst=0.0;abst<(TotalTime+CycleT);abst+=CycleT)
	{
		if(abst<=Seqt[1])//�k�⩹�����������A��m����
		{	
			Itv=SeqItv[0];
			t=abst-Seqt[0];

			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
			{
				Pend_R[f]=R_p[0][f]+(R_p[1][f]-R_p[0][f])*t/Itv; 
				Pend_L[f]=L_p[0][f]; 
			}
		}
		else if(abst<=Seqt[2])//�k�⧨��hold
		{		
			Itv=SeqItv[1];
			t=abst-Seqt[1];

			if(GripperAlreadyAct==0)
			{
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
#ifdef GRIPPER_ON_LATTE
				Gripper_LattePanda_Hold(DEF_RIGHT_HAND,true,1200);
#endif
				GripperAlreadyAct=1; 
			}	

			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
			{
				Pend_R[f]=R_p[1][f]; 
				Pend_L[f]=L_p[0][f]; 
			}
		}
		else if(abst<=Seqt[3])//�k�⩹����}�����A��m����
		{
			GripperAlreadyAct=0; //clear the gripper status of last segemnt

			Itv=SeqItv[2];
			t=abst-Seqt[2];

			//Cen_DoorPath���}�����|�b�|���
			Pend_R[DEF_X]=Cen_DoorPath[DEF_X]+rDoorPath*((float)sin(0.5*DEF_PI*t/Itv+ DEF_PI));
			Pend_R[DEF_Y]=Cen_DoorPath[DEF_Y]+rDoorPath*((float)cos(0.5*DEF_PI*t/Itv));
			Pend_R[DEF_Z]=Cen_DoorPath[DEF_Z]+rDoorPath*0;	

			for(int f=0;f<3;f++)  
			{
				Pend_L[f]=L_p[0][f]; 
			}
		}
		else if(abst<=Seqt[4])//���⩹�񶼮��I�e�i���~1 %���⩹x����100
		{
			Itv=SeqItv[3];
			t=abst-Seqt[3];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[2][f];							//�k������b����}�����A��m
				Pend_L[f]=L_p[0][f]+(L_p[1][f]-L_p[0][f])*t/Itv;//���⩹�񶼮��I�e�i���~1
			}
		}
		else if(abst<=Seqt[5])//���⩹�񶼮��I�e�i���~2 ���⩹y����-100
		{
			Itv=SeqItv[4];
			t=abst-Seqt[4];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[2][f];							//�k������b����}�����A��m
				Pend_L[f]=L_p[1][f]+(L_p[2][f]-L_p[1][f])*t/Itv;//���⩹�񶼮��I�e�i���~2
			}		
		}
		else if(abst<=Seqt[6])//���Ⲿ�ʨ�񶼮��I
		{
			Itv=SeqItv[5];
			t=abst-Seqt[5];
			
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[2][f];							//�k������b����}�����A��m
				Pend_L[f]=L_p[2][f]+(L_p[3][f]-L_p[2][f])*t/Itv;//���Ⲿ�ʨ�񶼮��I
			}	
		}
		else if(abst<=Seqt[7])//���⧨��hold
		{
			Itv=SeqItv[6];
			t=abst-Seqt[6];

			if(GripperAlreadyAct==0)
			{
#ifdef GRIPPER_ON_LATTE
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
				Gripper_LattePanda_Hold(DEF_LEFT_HAND,true,500);
#endif
				GripperAlreadyAct=1;
			}

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[2][f];//�k��T�w
				Pend_L[f]=L_p[3][f];//����T�w
			}	
		}
		else if(abst<=Seqt[8])//%����q�񶼮��I�h�^���~1
		{
			GripperAlreadyAct=0; //clear the gripper status of last segemnt

			Itv=SeqItv[7];
			t=abst-Seqt[7];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[2][f];//�k��T�w
				Pend_L[f]=L_p[3][f]+(L_p[4][f]-L_p[3][f])*t/Itv;
			}	
		}
		else if(abst<=Seqt[9])//����q�񶼮��I�h�^���~2 %���⩹z����-10
		{
			Itv=SeqItv[8];
			t=abst-Seqt[8];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[2][f];//�k��T�w
				Pend_L[f]=L_p[4][f]+(L_p[5][f]-L_p[4][f])*t/Itv;
			}	
		}
		else if(abst<=Seqt[10])//�k�⩹�����������A��m����
		{
			Itv=SeqItv[9];
			t=abst-Seqt[9];

			// Cen_DoorPath�������b�|���
			Pend_R[DEF_X]=Cen_DoorPath[DEF_X]+rDoorPath*((float)cos(0.5*DEF_PI*t/Itv+ DEF_PI));
			Pend_R[DEF_Y]=Cen_DoorPath[DEF_Y]+rDoorPath*((float)sin(0.5*DEF_PI*t/Itv));
			Pend_R[DEF_Z]=Cen_DoorPath[DEF_Z]+rDoorPath*0;	

			for(int f=0;f<3;f++)  
			{
				Pend_L[f]=L_p[5][f];//����T�w
			}	
		}
		else if(abst<=Seqt[11])//�k�⧨��rlease
		{
			Itv=SeqItv[10];
			t=abst-Seqt[10];
			
			if(GripperAlreadyAct==0)
			{
#ifdef GRIPPER_ON_LATTE
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
				Gripper_LattePanda_Hold(DEF_RIGHT_HAND,false,500);
#endif
				GripperAlreadyAct=1;
			}

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[3][f];//�k��T�w
				Pend_L[f]=L_p[5][f];//����T�w
			}	
		}
		else if(abst<=Seqt[12])//)�k��q�����������A��m�h�^���~1
		{
			GripperAlreadyAct=0; //clear the gripper status of last segemnt

			Itv=SeqItv[11];
			t=abst-Seqt[11];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[3][f]+(R_p[4][f]-R_p[3][f])*t/Itv;
				Pend_L[f]=L_p[5][f];//����T�w
			}	
		}
		else if(abst<=Seqt[13])//)�k��q�����������A��m�h�^���~2
		{
			Itv=SeqItv[12];
			t=abst-Seqt[12];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[4][f]+(R_p[5][f]-R_p[4][f])*t/Itv;
				Pend_L[f]=L_p[5][f];//����T�w
			}	
		}

//////////////////
		else if(abst<=Seqt[14])//����q�񶼮��I�h�^���~3 %���⩹x����-100
		{
			GripperAlreadyAct=0; //clear the gripper status of last segemnt

			Itv=SeqItv[13];
			t=abst-Seqt[13];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[5][f];//�k��T�w
				Pend_L[f]=L_p[5][f]+(L_p[6][f]-L_p[5][f])*t/Itv;
			}	
		}
		else if(abst<=Seqt[15])//���k��^��̫����ⴤ���Ʀ�m
		{
			Itv=SeqItv[14];
			t=abst-Seqt[14];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[5][f]+(R_p[6][f]-R_p[5][f])*t/Itv; 
				Pend_L[f]=L_p[6][f]+(L_p[7][f]-L_p[6][f])*t/Itv;
			}	
		}
		else if(abst==TotalTime)
		{
			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[6][f]; 
				Pend_L[f]=L_p[7][f];
			}
		}

		vel_deg_R=30;//
		vel_deg_L=30;//

		//static int cnt=0;//test
		//cnt++;
		//if(cnt==1556)
		//	cnt=cnt;


#ifdef MOVETOPOINT_DUAL
			MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
#endif
		printf("Pend_R=[%4.1f,%4.1f,%4.1f],Pend_L=[%4.1f,%4.1f,%4.1f]\n",Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);

		//==�T�{�y���I==//
#ifdef CHECK_CARTESIAN_PATH
		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",abst,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z]);
		fileR.write(buffer,n);

		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",abst,Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		fileL.write(buffer,n);
#endif 

#ifdef RECORD_JOINT_ANGLE
		////==Read right hand
		rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);

		if(rt==0)
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
			//}
			printf("\n");

			
			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
			fileR.write(buffer,n);
			
			memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
		}
		else //Ū�����ѮɡA���e�@���Ӹ�
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
			fileR.write(buffer,n);
		}

		//==Read left hand
		rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);

		if(rt==0)
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]);
			fileL.write(buffer,n);
			
			memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));
		}
		else //Ū�����ѮɡA���e�@���Ӹ�
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
			fileL.write(buffer,n);
		}
#endif
	}	

#if	defined(RECORD_JOINT_ANGLE) || defined(CHECK_CARTESIAN_PATH) 
	fileR.close();
	fileL.close();
#endif
	
#ifdef	CHECK_JOINT_PATH
	gfileR.close();
	gfileL.close();
#endif
}

void MoveToSelectPoint()
{
	//==Robotic arm pose==//
	int   RL=DEF_RIGHT_HAND;
	float Pend[3]={0,0,0};
	float pose_deg[3]={60,0,0};
	float Rednt_alpha=-60;
	float vel_deg=10;

	
	while(1)
	{
		printf("please enter==> R:1 L:2, p_x, p_y, p_z, alpha, beta, gamma, rednt, vel....\n");
		scanf("%d,%f,%f,%f,%f,%f,%f,%f,%f",&RL,&Pend[DEF_X],&Pend[DEF_Y],&Pend[DEF_Z],&pose_deg[0],&pose_deg[1],&pose_deg[2],&Rednt_alpha,&vel_deg);
		
		MoveToPoint(RL,Pend,pose_deg,Rednt_alpha,vel_deg);
	}
}


enum{
	S_INITIAL=0,
	S_RL_HOLD_1,
	S_RL_F_200,
	S_R_REL_1,
	S_R_X_B_200_S1,
	S_R_HOLD_1,
	S_R_X_CIRF_200_L_X_CIRB_200,
	S_R_REL_2,
	S_R_X_B_200_S2,
	S_R_HOLD_2,
	S_R_X_F_200_L_X_F_200,
};
void TestSewingAction()
{
	const int SegSize=11;
	//==���|�I
	float R_p[SegSize][3]={   
		{300,	-10, 0},	//�_�l�I
		{300,	-10, 0},	//���k�⧨��1
		{500,	-10, 0},	//�k�⩹�e200 %���⩹�e200
		{500,	-10, 0},	//�k���P�}1
		{300,	-10, 0},	//�k��x����h200 %���⤣��
		{300,	-10, 0},	//�k�⧨��1
		{500,	-10, 0},	//�k��x ��P���e200 %����x��P����200
		{500,	-10, 0},	//�k���P�}2
		{300,	-10, 0},	//�k��x����200 %���⤣��
		{300,	-10, 0},	//�k�⧨��2
		{500,	-10, 0}};	//�k�⩹�e200 %���⩹�e200
		
	float L_p[SegSize][3]={  
		{300, 90, 0},	//�_�l�I
		{300, 90, 0},	//���k�⧨��1
		{500, 90, 0},	//�k�⩹�e200 %����x���e200
		{500, 90, 0},	//�k���P�}1
		{500, 90, 0},	//�k��x����h200 %���⤣��
		{500, 90, 0},	//�k�⧨��1
		{300, 90, 0},	//�k��x ��P���e200 %����x��P����200  
		{300, 90, 0},	//�k���P�}2
		{300, 90, 0},	//�k��x����200 %���⤣��
		{300, 90, 0},	//�k�⧨��2
		{500, 90, 0}};	//�k��x���e200 %����x���e200
		
	//==�U�q��O�ɶ�==//
	float SeqItv[SegSize]={0};//Sequence  invterval

	SeqItv[S_INITIAL]=0;
	SeqItv[S_RL_HOLD_1]=2;
	SeqItv[S_RL_F_200]=10;
	SeqItv[S_R_REL_1]=2;
	SeqItv[S_R_X_B_200_S1]=10;
	SeqItv[S_R_HOLD_1]=2;
	SeqItv[S_R_X_CIRF_200_L_X_CIRB_200]=10;
	SeqItv[S_R_REL_2]=2;
	SeqItv[S_R_X_B_200_S2]=10;
	SeqItv[S_R_HOLD_2]=2;
	SeqItv[S_R_X_F_200_L_X_F_200]=10;

	//==����ɶ��Эp==//
	float Seqt[SegSize]={0};
	float CurT=0;

	for(int i=0;i<SegSize;i++)
	{
		CurT=CurT+SeqItv[i];
		Seqt[i]=CurT;
	}
	float TotalTime=CurT;
	
	

	//==�y�{�ݥ��ܼ�
	//float CycleT=0.1f;
#ifndef RECORD_JOINT_ANGLE
	//float CycleT=0.04f; //��Ūfeedback�� �j��22m
	float CycleT=0.01f;
#else
	float CycleT=0.1f; //��Ūfeedback�ɻݭn 22+17+17=56ms 19+16+16=51ms
#endif

	//float CycleT=0.040f; //�氦��S������Ū��
	//float CycleT=0.03f; //��Ūfeedback�� �j��22m ���Lcycle time�]�j�@�I���|���b���Iover load���p�Acycltime���ڤj�A��ܷ|�b��F�e�U�s�R�O

	float Itv=0; //interval of the segment
	float t=0; //t of each segment (0~interval)
	int GripperAlreadyAct=0;// already send command to gripper
	

	//==variable for reocrd file==//
	//open file
#ifdef	RECORD_JOINT_ANGLE
	fstream fileR;
	fstream fileL;
	fileR.open("D://GetDrinkJointAngle_R.csv",ios::out|ios::trunc);
	fileL.open("D://GetDrinkJointAngle_L.csv",ios::out|ios::trunc);
#endif

#ifdef CHECK_CARTESIAN_PATH
	fstream fileR;
	fstream fileL;
	fileR.open("D://GetSewCartesian_R.csv",ios::out|ios::trunc);
	fileL.open("D://GetSewCartesian_L.csv",ios::out|ios::trunc);

#endif

#ifdef	CHECK_JOINT_PATH
	gfileR.open("D://IK_CMD_R.csv",ios::out|ios::trunc);
	gfileL.open("D://IK_CMD_L.csv",ios::out|ios::trunc);
#endif
	
	
	//==record para==//
	float pos_deg_R[MAX_AXIS_NUM]={0};
	float pos_deg_L[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
	float pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
	int n=0;
	int rt=0;
	char buffer[100];
	//==Robotic arm pose==//
	float Pend_R[3]={0,0,0};
	float pose_deg_R[3]={70,-90,0};
	float Rednt_alpha_R=-90;
	float vel_deg_R=10;

	float Pend_L[3]={0,0,0};
	float pose_deg_L[3]={-60,90,0};
	float Rednt_alpha_L=90;
	float vel_deg_L=5;

	//==move to initial point==//
#ifdef	MOVE_TO_INITIAL_POINT
	//MoveToPoint_Dual(R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
	MoveToPoint(DEF_RIGHT_HAND,R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R);
	MoveToPoint(DEF_LEFT_HAND,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);
	printf("move to p0..\n");
	WaitMotionDoneDual();
	
#endif

#ifdef	DEF_WAIT_ENTER
	printf("In initial point. press any key to continue...\n");
	getchar();
#endif
	//==Sewing process start ==//
	float abst=0.0;
	for(abst=0.0;abst<(TotalTime+CycleT);abst+=CycleT)
	{
		if(abst<=Seqt[S_RL_HOLD_1])//���k�⧨��
		{
			Itv=SeqItv[S_RL_HOLD_1];
			t=abst-Seqt[S_INITIAL];

			if(GripperAlreadyAct==0)
			{
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
#ifdef GRIPPER_ON_LATTE
				Gripper_LattePanda_Hold(DEF_RIGHT_HAND,true,1200);
#endif
				GripperAlreadyAct=1; 
			}	

			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
			{
				Pend_R[f]=R_p[S_INITIAL][f]; 
				Pend_L[f]=L_p[S_INITIAL][f]; 
			}
		}
		else if(abst<=Seqt[S_RL_F_200])//�k�⩹�e200 %���⩹�e200
		{	
			GripperAlreadyAct=0; //clear the gripper status of last segemnt

			Itv=SeqItv[S_RL_F_200];
			t=abst-Seqt[S_RL_HOLD_1];

			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
			{
				Pend_R[f]=R_p[S_RL_HOLD_1][f]+(R_p[S_R_REL_1][f]-R_p[S_RL_HOLD_1][f])*t/Itv; 
				Pend_L[f]=L_p[S_RL_HOLD_1][f]+(L_p[S_R_REL_1][f]-L_p[S_RL_HOLD_1][f])*t/Itv;
			}
		}
		else if(abst<=Seqt[S_R_REL_1])//�k���P�}1
		{		
			Itv=SeqItv[S_R_REL_1];
			t=abst-Seqt[S_RL_F_200];

			if(GripperAlreadyAct==0)
			{
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
#ifdef GRIPPER_ON_LATTE
				Gripper_LattePanda_Hold(DEF_RIGHT_HAND,false,1200);
#endif
				GripperAlreadyAct=1; 
			}	

			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
			{
				Pend_R[f]=R_p[S_R_REL_1][f]; 
				Pend_L[f]=L_p[S_R_REL_1][f]; 
			}
		}
		else if(abst<=Seqt[S_R_X_B_200_S1])//�k��x����h200 %���⤣��
		{
			GripperAlreadyAct=0;

			Itv=SeqItv[S_R_X_B_200_S1];
			t=abst-Seqt[S_R_REL_1];

			for(int f=0;f<3;f++)   
			{
				Pend_R[f]=R_p[S_R_REL_1][f]+(R_p[S_R_X_B_200_S1][f]-R_p[S_R_REL_1][f])*t/Itv; 
				Pend_L[f]=L_p[S_R_REL_1][f]; 
			}		
		}
		else if(abst<=Seqt[S_R_HOLD_1])//�k�⧨��1
		{
			Itv=SeqItv[S_R_HOLD_1];
			t=abst-Seqt[S_R_X_B_200_S1];

			for(int f=0;f<3;f++)   //��xyz�y�Ф��O�B�� f=x,y,z
			{
				Pend_R[f]=R_p[S_R_X_B_200_S1][f]; 
				Pend_L[f]=L_p[S_R_X_B_200_S1][f]; 
			}
		}
		else if(abst<=Seqt[S_R_X_CIRF_200_L_X_CIRB_200])//�k��x ��P���e200 %����x��P����200
		{
			Itv=SeqItv[S_R_X_CIRF_200_L_X_CIRB_200];
			t=abst-Seqt[S_R_HOLD_1];

			float Cen_Path_R[3]={(500+300)*0.5,-10,0};
			float rR = 500-Cen_Path_R[DEF_X];

			float Cen_Path_L[3]={(500+300)*0.5,90,0};
			float rL = 500-Cen_Path_L[DEF_X];

			for(int f=0;f<3;f++)  
			{
				Pend_R[DEF_X]=Cen_Path_R[DEF_X]+rR*(cos(DEF_PI*t/Itv + DEF_PI)); 
				Pend_R[DEF_Y]=Cen_Path_R[DEF_Y]+rR*(sin(DEF_PI*t/Itv + DEF_PI)); 
				Pend_R[DEF_Z]=Cen_Path_R[DEF_Z];

				Pend_L[DEF_X]=Cen_Path_L[DEF_X]+rL*(cos(DEF_PI*t/Itv)); 
				Pend_L[DEF_Y]=Cen_Path_L[DEF_Y]+rL*(sin(DEF_PI*t/Itv)); 
				Pend_L[DEF_Z]=Cen_Path_L[DEF_Z];
			}		
		}
		else if(abst<=Seqt[S_R_REL_2])//�k���P�}2
		{
			Itv=SeqItv[S_R_REL_2];
			t=abst-Seqt[S_R_X_CIRF_200_L_X_CIRB_200];
			
			if(GripperAlreadyAct==0)
			{
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
#ifdef GRIPPER_ON_LATTE
				Gripper_LattePanda_Hold(DEF_RIGHT_HAND,false,1200);
#endif
				GripperAlreadyAct=1; 
			}	

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[S_R_X_CIRF_200_L_X_CIRB_200][f];							
				Pend_L[f]=L_p[S_R_X_CIRF_200_L_X_CIRB_200][f];
			}	
		}
		else if(abst<=Seqt[S_R_X_B_200_S2])//�k��x����200 %���⤣��
		{
			GripperAlreadyAct=0;

			Itv=SeqItv[S_R_X_B_200_S2];
			t=abst-Seqt[S_R_REL_2];


			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[S_R_REL_2][f]+(R_p[S_R_X_B_200_S2][f]-R_p[S_R_REL_2][f])*t/Itv;
				Pend_L[f]=L_p[S_R_REL_2][f]+(L_p[S_R_X_B_200_S2][f]-L_p[S_R_REL_2][f])*t/Itv;
			}	
		}
		else if(abst<=Seqt[S_R_HOLD_2])//%�k�⧨��2
		{
			Itv=SeqItv[S_R_HOLD_2];
			t=abst-Seqt[S_R_X_B_200_S2];
			
			if(GripperAlreadyAct==0)
			{
				printf("press any key to continue...\n");
#ifdef	DEF_WAIT_ENTER
				getchar();
#endif
#ifdef GRIPPER_ON_LATTE
				Gripper_LattePanda_Hold(DEF_RIGHT_HAND,true,1200);
#endif
				GripperAlreadyAct=1; 
			}	

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[S_R_X_B_200_S2][f];//�k��T�w
				Pend_L[f]=L_p[S_R_X_B_200_S2][f];
			}	
		}
		else if(abst<=Seqt[S_R_X_F_200_L_X_F_200])//%�k��x���e200 %����x���e200
		{
			GripperAlreadyAct=1;

			Itv=SeqItv[S_R_X_F_200_L_X_F_200];
			t=abst-Seqt[S_R_HOLD_2];

			for(int f=0;f<3;f++)  
			{
				Pend_R[f]=R_p[S_R_HOLD_2][f]+(R_p[S_R_X_F_200_L_X_F_200][f]-R_p[S_R_HOLD_2][f])*t/Itv;
				Pend_L[f]=L_p[S_R_HOLD_2][f]+(L_p[S_R_X_F_200_L_X_F_200][f]-L_p[S_R_HOLD_2][f])*t/Itv;
			}	
		}

		vel_deg_R=30;
		vel_deg_L=30;

		//static int cnt=0;//test
		//cnt++;
		//if(cnt==1556)
		//	cnt=cnt;


#ifdef MOVETOPOINT_DUAL
		MoveToPoint_Dual(Pend_R,pose_deg_R,Rednt_alpha_R,vel_deg_R,Pend_L,pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
#endif
		printf("Pend_R=[%4.1f,%4.1f,%4.1f],Pend_L=[%4.1f,%4.1f,%4.1f]\n",Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z],Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);

		//==�T�{�y���I==//
#ifdef CHECK_CARTESIAN_PATH
		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",abst,Pend_R[DEF_X],Pend_R[DEF_Y],Pend_R[DEF_Z]);
		fileR.write(buffer,n);

		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",abst,Pend_L[DEF_X],Pend_L[DEF_Y],Pend_L[DEF_Z]);
		fileL.write(buffer,n);
#endif 

#ifdef RECORD_JOINT_ANGLE
		////==Read right hand
		rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);

		if(rt==0)
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
			//}
			printf("\n");

			
			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
			fileR.write(buffer,n);
			
			memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
		}
		else //Ū�����ѮɡA���e�@���Ӹ�
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
			fileR.write(buffer,n);
		}

		//==Read left hand
		rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);

		if(rt==0)
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]);
			fileL.write(buffer,n);
			
			memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));
		}
		else //Ū�����ѮɡA���e�@���Ӹ�
		{
			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
			//{
			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
			//}
			printf("\n");

			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
			fileL.write(buffer,n);
		}
#endif
	}	

#if	defined(RECORD_JOINT_ANGLE) || defined(CHECK_CARTESIAN_PATH) 
	fileR.close();
	fileL.close();
#endif
	
#ifdef	CHECK_JOINT_PATH
	gfileR.close();
	gfileL.close();
#endif
}

int _tmain(int argc, _TCHAR* argv[])
{
	
	//=================//
	//===initial DXL===//
	//=================//
	//int rt=DXL_Initial_x86();
	//if(rt==0)
	//{
	//	printf("DXL_Initial_x86 failed\n");
	//	getchar();
	//	return 0;
	//}



	//====================//
	//===initial gripper==//
	//====================//
	//printf("Gripper_LattePanda_Initial...\n");
	//Gripper_LattePanda_Initial();

	//====================//
	//===MoveToSelectPoint==//
	//====================//
	//MoveToSelectPoint();
	//TestGetDrink();
	//======================================//
	//==Rec_Rectangle_Linear_function_Dual==//
	//======================================//
	//PID_Setting_Dual();
	//Rec_Rectangle_Linear_function_Dual();

	//================//
	//==TestGetDrink==//
	//================//
	//PID_Setting_Dual();
	//printf("TestGetDrink...\n");
	//TestGetDrink();
	//printf("Enter any key to go home...\n");
	//getchar();
	//printf("MoveToHome...\n");
	//TestMoveToHome_Dual();

	//================//
	//==TestSewing==//
	//================//
	TestSewingAction();


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

	

	//Sleep(1500);

	DXL_Terminate_x86();
	//Gripper_LattePanda_Close();
	
	return 0;
}

