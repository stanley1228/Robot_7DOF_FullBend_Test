


#include "Robot_7DOF_FB.h"
#include <vector>
#include "Matrix.h"
#include "MatrixMath.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>


#include "dynamixel.h"


#pragma comment(lib,"dynamixel.lib") 
#pragma comment(lib,"modbus.lib")




using namespace std;

#if (DEBUG)
	#define DBGMSG(x)  printf x;
#else
    #define DBGMSG(x)
#endif

unsigned char getMapAxisNO(unsigned char index)
{
	return gMapAxisNO[index];

}

unsigned char getMapRAxisID(unsigned char index)
{
	return gMapRAxisID[index];
}

unsigned char getMapLAxisID(unsigned char index)
{
	return gMapLAxisID[index];
}


int ROM_Setting_Dual()
{
	//==calculate Max torque to set in rom 
	const short int Max_torque_R[MAX_AXIS_NUM]=
	{
		AXISR1_MAX_TORQUE,
		AXISR2_MAX_TORQUE,
		AXISR3_MAX_TORQUE,
		AXISR4_MAX_TORQUE,
		AXISR5_MAX_TORQUE,
		AXISR6_MAX_TORQUE,
		AXISR7_MAX_TORQUE
	};

	const short int Max_torque_L[MAX_AXIS_NUM]=
	{
		AXISL1_MAX_TORQUE,
		AXISL2_MAX_TORQUE,
		AXISL3_MAX_TORQUE,
		AXISL4_MAX_TORQUE,
		AXISL5_MAX_TORQUE,
		AXISL6_MAX_TORQUE,
		AXISL7_MAX_TORQUE
	};



	//==Calculate angle limit==//
	const short int R2M_OFFSET_DEG_R[MAX_AXIS_NUM]=
	{
		AXISR1_R2M_OFFSET_DEG,
		AXISR2_R2M_OFFSET_DEG,
		AXISR3_R2M_OFFSET_DEG,
		AXISR4_R2M_OFFSET_DEG,
		AXISR5_R2M_OFFSET_DEG,
		AXISR6_R2M_OFFSET_DEG,
		AXISR7_R2M_OFFSET_DEG
	};

	const short int R2M_OFFSET_DEG_L[MAX_AXIS_NUM]=
	{
		AXISL1_R2M_OFFSET_DEG,
		AXISL2_R2M_OFFSET_DEG,
		AXISL3_R2M_OFFSET_DEG,
		AXISL4_R2M_OFFSET_DEG,
		AXISL5_R2M_OFFSET_DEG,
		AXISL6_R2M_OFFSET_DEG,
		AXISL7_R2M_OFFSET_DEG
	};
	
	//right hand
	const short int ROBOT_LIM_DEG_R_LOW[MAX_AXIS_NUM]=
	{
		AXISR1_ROBOT_LIM_DEG_L,
		AXISR2_ROBOT_LIM_DEG_L,
		AXISR3_ROBOT_LIM_DEG_L,
		AXISR4_ROBOT_LIM_DEG_L,
		AXISR5_ROBOT_LIM_DEG_L,
		AXISR6_ROBOT_LIM_DEG_L,
		AXISR7_ROBOT_LIM_DEG_L
	};

	const short int ROBOT_LIM_DEG_R_HIGH[MAX_AXIS_NUM]=
	{
		AXISR1_ROBOT_LIM_DEG_H,
		AXISR2_ROBOT_LIM_DEG_H,
		AXISR3_ROBOT_LIM_DEG_H,
		AXISR4_ROBOT_LIM_DEG_H,
		AXISR5_ROBOT_LIM_DEG_H,
		AXISR6_ROBOT_LIM_DEG_H,
		AXISR7_ROBOT_LIM_DEG_H
	};

	//left hand
	const short int ROBOT_LIM_DEG_L_LOW[MAX_AXIS_NUM]= 
	{
		AXISL1_ROBOT_LIM_DEG_L,
		AXISL2_ROBOT_LIM_DEG_L,
		AXISL3_ROBOT_LIM_DEG_L,
		AXISL4_ROBOT_LIM_DEG_L,
		AXISL5_ROBOT_LIM_DEG_L,
		AXISL6_ROBOT_LIM_DEG_L,
		AXISL7_ROBOT_LIM_DEG_L
	};

	const short int ROBOT_LIM_DEG_L_HIGH[MAX_AXIS_NUM]= 
	{
		AXISL1_ROBOT_LIM_DEG_H,
		AXISL2_ROBOT_LIM_DEG_H,
		AXISL3_ROBOT_LIM_DEG_H,
		AXISL4_ROBOT_LIM_DEG_H,
		AXISL5_ROBOT_LIM_DEG_H,
		AXISL6_ROBOT_LIM_DEG_H,
		AXISL7_ROBOT_LIM_DEG_H
	};



	unsigned short int Motor_lim_pulse_R_high[MAX_AXIS_NUM]={0};
	unsigned short int Motor_lim_pulse_R_low[MAX_AXIS_NUM]={0};
	unsigned short int Motor_lim_pulse_L_high[MAX_AXIS_NUM]={0};
	unsigned short int Motor_lim_pulse_L_low[MAX_AXIS_NUM]={0};
	

	int i=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//right hand
		Motor_lim_pulse_R_low[i]=(unsigned short int)((ROBOT_LIM_DEG_R_LOW[i]+R2M_OFFSET_DEG_R[i])*DEF_RATIO_DEG_TO_PUS);
		Motor_lim_pulse_R_high[i]=(unsigned short int)((ROBOT_LIM_DEG_R_HIGH[i]+R2M_OFFSET_DEG_R[i])*DEF_RATIO_DEG_TO_PUS);

		//left hand
		Motor_lim_pulse_L_low[i]=(unsigned short int)((ROBOT_LIM_DEG_L_LOW[i]+R2M_OFFSET_DEG_L[i])*DEF_RATIO_DEG_TO_PUS);
		Motor_lim_pulse_L_high[i]=(unsigned short int)((ROBOT_LIM_DEG_L_HIGH[i]+R2M_OFFSET_DEG_L[i])*DEF_RATIO_DEG_TO_PUS);
	}

	//==writing to ROM==//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//==Set MAX_torgue==//
		dxl_write_word(gMapRAxisID[i],MAX_TORQUE,Max_torque_R[i]);//right
		dxl_write_word(gMapLAxisID[i],MAX_TORQUE,Max_torque_L[i]);//left
	
		//==Set angel limit==//
		dxl_write_word(gMapRAxisID[i],CW_ANGLE_LIMIT_L,Motor_lim_pulse_R_low[i]);//right
		dxl_write_word(gMapRAxisID[i],CCW_ANGLE_LIMIT_L,Motor_lim_pulse_R_high[i]);//right  

		dxl_write_word(gMapLAxisID[i],CW_ANGLE_LIMIT_L,Motor_lim_pulse_L_low[i]);//left
		dxl_write_word(gMapLAxisID[i],CCW_ANGLE_LIMIT_L,Motor_lim_pulse_L_high[i]);//left  

		//==Set MULTITURN_OFFSET to 0==//
		dxl_write_word(gMapRAxisID[i],MULTITURN_OFFSET,0);//right
		dxl_write_word(gMapLAxisID[i],MULTITURN_OFFSET,0);//left
	}
	

	//==read and check right hand==//
	int	txrx_result=0;
	short int max_torque=0;
	short int cw_angel_lim=0,ccw_angle_lim=0;
	short int multi_turn_offset=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		printf("===AXIS_R%d===\n",gMapAxisNO[i]);

		//==MAX_torgue==//
		max_torque = dxl_read_word(gMapRAxisID[i], MAX_TORQUE);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read MAX_TORQUE error=%d\n",txrx_result);
		else
			printf("MAX_TORQUE=%d\n",max_torque);
	
		//==CW_ANGLE_LIMIT,CCW_ANGLE_LIMIT==//
		cw_angel_lim=dxl_read_word(gMapRAxisID[i],CW_ANGLE_LIMIT_L);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read CW_ANGLE_LIMIT error=%d\n",txrx_result);
		else	
			printf("CW_ANGLE_LIMIT=%d,degree=%f\n",cw_angel_lim,cw_angel_lim*DEF_RATIO_PUS_TO_DEG);

		ccw_angle_lim=dxl_read_word(gMapRAxisID[i],CCW_ANGLE_LIMIT_L);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed Read CCW_ANGLE_LIMIT failed error=%d\n",txrx_result);
		else	
			printf("CCW_ANGLE_LIMIT=%d,degree=%f\n",ccw_angle_lim,ccw_angle_lim*DEF_RATIO_PUS_TO_DEG);
		

		//==multi turn offset==//
		multi_turn_offset=dxl_read_word(gMapRAxisID[i],MULTITURN_OFFSET);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed Read MULTITURN_OFFSET failed error=%d\n",txrx_result);
		else	
			printf("MULTITURN_OFFSET=%d\n",multi_turn_offset);
	}

	//==read and check left hand==//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		printf("===AXIS_L%d===\n",gMapAxisNO[i]);

		//==MAX_torgue==//
		max_torque = dxl_read_word(gMapLAxisID[i], MAX_TORQUE);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read MAX_TORQUE error=%d\n",txrx_result);
		else
			printf("MAX_TORQUE=%d\n",max_torque);
	
		//==CW_ANGLE_LIMIT,CCW_ANGLE_LIMIT==//
		cw_angel_lim=dxl_read_word(gMapLAxisID[i],CW_ANGLE_LIMIT_L);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read CW_ANGLE_LIMIT error=%d\n",txrx_result);
		else	
			printf("CW_ANGLE_LIMIT=%d,degree=%f\n",cw_angel_lim,cw_angel_lim*DEF_RATIO_PUS_TO_DEG);

		ccw_angle_lim=dxl_read_word(gMapLAxisID[i],CCW_ANGLE_LIMIT_L);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed Read CCW_ANGLE_LIMIT failed error=%d\n",txrx_result);
		else	
			printf("CCW_ANGLE_LIMIT=%d,degree=%f\n",ccw_angle_lim,ccw_angle_lim*DEF_RATIO_PUS_TO_DEG);
		

		//==multi turn offset==//
		multi_turn_offset=dxl_read_word(gMapLAxisID[i],MULTITURN_OFFSET);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed Read MULTITURN_OFFSET failed error=%d\n",txrx_result);
		else	
			printf("MULTITURN_OFFSET=%d\n",multi_turn_offset);
	}

	return 0;
}

//rt=Read_pos(pos_pus,DEF_UNIT_PUS)
int Read_pos(int RLHand,float *pos,unsigned char unit)
{
	int i=0;
	short int pulse=0;
	int rt=0;

	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//read pulse
		if(RLHand==DEF_RIGHT_HAND)
			pulse = dxl_read_word(gMapRAxisID[i], PRESENT_POS);
		else if(RLHand==DEF_LEFT_HAND)
			pulse = dxl_read_word(gMapLAxisID[i], PRESENT_POS);

		//If communication ok calculate pulse offset and unit transform
		if(dxl_get_result()!=COMM_RXSUCCESS)
		{
			rt=-gMapAxisNO[i];
			pos[i]=0xffff;
		}
		else
		{
			if(RLHand==DEF_RIGHT_HAND)
				pulse-=gr2m_offset_pulse_R[i]; //motor to robot offset =>minus offset
			else if(RLHand==DEF_LEFT_HAND)
				pulse-=gr2m_offset_pulse_L[i];

			if(unit==DEF_UNIT_RAD)
				pos[i]=pulse*DEF_RATIO_PUS_TO_RAD;
			else if(unit==DEF_UNIT_DEG)
				pos[i]=pulse*DEF_RATIO_PUS_TO_DEG;
			else if(unit==DEF_UNIT_PUS)	
				pos[i]=pulse;
			else//non offset pulse
			{
				if(RLHand==DEF_RIGHT_HAND)
					pulse+=gr2m_offset_pulse_R[i];
				else if(RLHand==DEF_LEFT_HAND)
					pulse+=gr2m_offset_pulse_L[i];
				pos[i]=pulse;
			}
		}
	}
		
	return rt;
}




int Output_to_Dynamixel(int RLHand,const float *Ang_rad,const unsigned short int *velocity) 
{
	unsigned char i=0;

	//===================================================//
	//==trnasformat to pulse and offset to motor domain==//
	//====================================================//
	short int Ang_pulse[MAX_AXIS_NUM]={0};
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		Ang_pulse[i]=(short int)(Ang_rad[i]*DEF_RATIO_RAD_TO_PUS);

		if(RLHand==DEF_RIGHT_HAND)
			Ang_pulse[i]+=gr2m_offset_pulse_R[i];
		else if(RLHand==DEF_LEFT_HAND)
			Ang_pulse[i]+=gr2m_offset_pulse_L[i];

		if( Ang_pulse[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXIS%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
	}

	//================================//
	//==output to motor by syncpage===//
	//===============================//
	unsigned short int SyncPageR[21]=
	{ 
		ID_RAXIS1,(unsigned short int)Ang_pulse[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
		ID_RAXIS2,(unsigned short int)Ang_pulse[Index_AXIS2],velocity[Index_AXIS2], 
		ID_RAXIS3,(unsigned short int)Ang_pulse[Index_AXIS3],velocity[Index_AXIS3], 
		ID_RAXIS4,(unsigned short int)Ang_pulse[Index_AXIS4],velocity[Index_AXIS4], 
		ID_RAXIS5,(unsigned short int)Ang_pulse[Index_AXIS5],velocity[Index_AXIS5], 
		ID_RAXIS6,(unsigned short int)Ang_pulse[Index_AXIS6],velocity[Index_AXIS6], 
		ID_RAXIS7,(unsigned short int)Ang_pulse[Index_AXIS7],velocity[Index_AXIS7], 
	};

	unsigned short int SyncPageL[21]=
	{ 
		ID_LAXIS1,(unsigned short int)Ang_pulse[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
		ID_LAXIS2,(unsigned short int)Ang_pulse[Index_AXIS2],velocity[Index_AXIS2], 
		ID_LAXIS3,(unsigned short int)Ang_pulse[Index_AXIS3],velocity[Index_AXIS3], 
		ID_LAXIS4,(unsigned short int)Ang_pulse[Index_AXIS4],velocity[Index_AXIS4], 
		ID_LAXIS5,(unsigned short int)Ang_pulse[Index_AXIS5],velocity[Index_AXIS5], 
		ID_LAXIS6,(unsigned short int)Ang_pulse[Index_AXIS6],velocity[Index_AXIS6], 
		ID_LAXIS7,(unsigned short int)Ang_pulse[Index_AXIS7],velocity[Index_AXIS7], 
	};
	
#if (DEBUG)
	//for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	//	printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
#endif
	if(RLHand==DEF_RIGHT_HAND)
		syncWrite_x86(GOAL_POSITION,2,SyncPageR,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
	else if(RLHand==DEF_LEFT_HAND)
		syncWrite_x86(GOAL_POSITION,2,SyncPageL,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
	
	
	return 0;
}


int Output_to_Dynamixel_Dual(const float *Ang_rad_R,const unsigned short int *velocity_R,const float *Ang_rad_L,const unsigned short int *velocity_L) 
{
	unsigned char i=0;

	//===================================================//
	//==trnasformat to pulse and offset to motor domain==//
	//====================================================//
	short int Ang_pulse_R[MAX_AXIS_NUM]={0};
	short int Ang_pulse_L[MAX_AXIS_NUM]={0};

	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		Ang_pulse_R[i]=(short int)(Ang_rad_R[i]*DEF_RATIO_RAD_TO_PUS);
		Ang_pulse_L[i]=(short int)(Ang_rad_L[i]*DEF_RATIO_RAD_TO_PUS);

		
		Ang_pulse_R[i]+=gr2m_offset_pulse_R[i];
		Ang_pulse_L[i]+=gr2m_offset_pulse_L[i];

		if( Ang_pulse_R[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXISR%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse_R[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
		if( Ang_pulse_L[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXISL%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse_L[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
	}

	//================================//
	//==output to motor by syncpage===//
	//===============================//
	unsigned short int SyncPageR[42]=
	{ 
		ID_RAXIS1,(unsigned short int)Ang_pulse_R[Index_AXIS1],velocity_R[Index_AXIS1], //ID,goal,velocity
		ID_RAXIS2,(unsigned short int)Ang_pulse_R[Index_AXIS2],velocity_R[Index_AXIS2], 
		ID_RAXIS3,(unsigned short int)Ang_pulse_R[Index_AXIS3],velocity_R[Index_AXIS3], 
		ID_RAXIS4,(unsigned short int)Ang_pulse_R[Index_AXIS4],velocity_R[Index_AXIS4], 
		ID_RAXIS5,(unsigned short int)Ang_pulse_R[Index_AXIS5],velocity_R[Index_AXIS5], 
		ID_RAXIS6,(unsigned short int)Ang_pulse_R[Index_AXIS6],velocity_R[Index_AXIS6], 
		ID_RAXIS7,(unsigned short int)Ang_pulse_R[Index_AXIS7],velocity_R[Index_AXIS7], 
		ID_LAXIS1,(unsigned short int)Ang_pulse_L[Index_AXIS1],velocity_L[Index_AXIS1], 
		ID_LAXIS2,(unsigned short int)Ang_pulse_L[Index_AXIS2],velocity_L[Index_AXIS2], 
		ID_LAXIS3,(unsigned short int)Ang_pulse_L[Index_AXIS3],velocity_L[Index_AXIS3], 
		ID_LAXIS4,(unsigned short int)Ang_pulse_L[Index_AXIS4],velocity_L[Index_AXIS4], 
		ID_LAXIS5,(unsigned short int)Ang_pulse_L[Index_AXIS5],velocity_L[Index_AXIS5], 
		ID_LAXIS6,(unsigned short int)Ang_pulse_L[Index_AXIS6],velocity_L[Index_AXIS6], 
		ID_LAXIS7,(unsigned short int)Ang_pulse_L[Index_AXIS7],velocity_L[Index_AXIS7], 
	};
	
#if (DEBUG)
	//for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	//	printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
#endif
	
	syncWrite_x86(GOAL_POSITION,2,SyncPageR,42);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
	
	
	return 0;
}

int Output_to_Dynamixel_pulse(const unsigned short int *Ang_pulse,const unsigned short int *velocity) 
{
	unsigned char i=0;

	//===================================================================//
	//==limit axis  if not zero ,the return value is the overlimit axis==//
	//===================================================================//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		if( (Ang_pulse[i] > grobot_lim_pus_R_High[i]) || (Ang_pulse[i] < grobot_lim_pus_R_Low[i]) )
		{
			DBGMSG(("AXIS%d over limit Ang_pus=%d,grobot_lim_pus_L=%d,grobot_lim_pus_H=%d\n",gMapAxisNO[i],Ang_pulse[i],grobot_lim_pus_R_Low[i],grobot_lim_pus_R_High[i]))
			return -gMapAxisNO[i];
		}
	}

	//====================================================//
	//==trnasformat to pulse and offset to motor domain===//
	//====================================================//
	unsigned short int Ang_pulse_with_offset[MAX_AXIS_NUM]={0};
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		Ang_pulse_with_offset[i]=Ang_pulse[i]+gr2m_offset_pulse_R[i];

		if( Ang_pulse_with_offset[i] > DEF_JOINT_MODE_MAX_PULSE )//  0~4095
		{
			DBGMSG(("AXIS%d over range of mortor  Ang_pulse=%d,JOINT_MODE_MIN_PULSE=%d,JOINT_MODE_MAX_PULSE=%d\n",gMapAxisNO[i],Ang_pulse_with_offset[i],DEF_JOINT_MODE_MIN_PULSE,DEF_JOINT_MODE_MAX_PULSE))
			return -gMapAxisNO[i];
		}
	}

	//================================//
	//==output to motor by syncpage===//
	//===============================//
	unsigned short int SyncPage1[21]=
	{ 
		ID_RAXIS1,Ang_pulse_with_offset[Index_AXIS1],velocity[Index_AXIS1], //ID,goal,velocity
		ID_RAXIS2,Ang_pulse_with_offset[Index_AXIS2],velocity[Index_AXIS2], 
		ID_RAXIS3,Ang_pulse_with_offset[Index_AXIS3],velocity[Index_AXIS3], 
		ID_RAXIS4,Ang_pulse_with_offset[Index_AXIS4],velocity[Index_AXIS4], 
		ID_RAXIS5,Ang_pulse_with_offset[Index_AXIS5],velocity[Index_AXIS5], 
		ID_RAXIS6,Ang_pulse_with_offset[Index_AXIS6],velocity[Index_AXIS6], 
		ID_RAXIS7,Ang_pulse_with_offset[Index_AXIS7],velocity[Index_AXIS7], 
	};

	
#if (DEBUG)
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
		printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
#endif

	syncWrite_x86(GOAL_POSITION,2,SyncPage1,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
  
	return 0;
}

//�ک� Z1X2Y3 Intrinsic Rotaions�۹���e���Шt��������
Matrix R_z1x2y3(float alpha,float beta,float gamma)
{
	Matrix ans(3,3);
	ans << cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma)	<< -cos(beta)*sin(alpha)	<< cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta)
        << cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)	<< cos(alpha)*cos(beta)		<< sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta)
        << -cos(beta)*sin(gamma)									<< sin(beta)				<< cos(beta)*cos(gamma);

	return ans;

}

float norm(const Matrix& v)
{
	int i=0;
	float ans=0;

	for(i=1;i<=v.getRows();i++)
	{
		ans+=pow(v(i,1),2);
	}
	
	ans=sqrt(ans);

	return ans;
}


Matrix Rogridues(float theta,const Matrix& V_A)
{
	Matrix R_a(4,4);

	float cs = cos( theta );
    float sn = sin( theta );

	R_a	<<	cs + pow(V_A.getNumber(1,1),2)*(1-cs)								<<	V_A.getNumber(1,1)*V_A.getNumber(2,1)*(1-cs)-V_A.getNumber(3,1)*sn	<<	V_A.getNumber(1,1)*V_A.getNumber(3,1)*(1-cs)+V_A.getNumber(2,1)*sn  <<	0
		<<	V_A.getNumber(1,1)*V_A.getNumber(2,1)*(1-cs)+V_A.getNumber(3,1)*sn	<<	cos(theta)+pow(V_A.getNumber(2,1),2)*(1-cs)							<<	V_A.getNumber(2,1)*V_A.getNumber(3,1)*(1-cs)-V_A.getNumber(1,1)*sn	<<	0
        <<	V_A.getNumber(1,1)*V_A.getNumber(3,1)*(1-cs)-V_A.getNumber(2,1)*sn	<<	V_A.getNumber(2,1)*V_A.getNumber(3,1)*(1-cs)+V_A.getNumber(1,1)*sn	<<	cs+pow(V_A.getNumber(3,1),2)*(1-cs)									<<	0
        <<	0																	<<	0																	<<	0																	<<	1;
    
	return R_a;
}
int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta)
{
	int i=0;
	
	//Out put initial to zero
	for(i=Index_AXIS1;i<=Index_AXIS7;i++)
	{
		theta[i]=0;
	}

	Matrix R(3,3);
	R=R_z1x2y3(alpha,beta,gamma);

	Matrix V_H_hat_x(3,1);
	V_H_hat_x=Matrix::ExportCol(R,1);//���X�کԨ��ഫ������x�}�A���X��1�欰X�b�����V�q
	V_H_hat_x*=1/norm(V_H_hat_x);
	
	Matrix V_H_hat_y(3,1);
	V_H_hat_y=Matrix::ExportCol(R,2);//���X�کԨ��ഫ������x�}�A���X��2�欰Y�b�����V�q
	V_H_hat_y*=1/norm(V_H_hat_y);
	

	Matrix V_r_end(3,1);
	V_r_end	<<x_end-x_base
			<<y_end-y_base
			<<z_end-z_base;


	Matrix V_r_h(3,1);
	V_r_h=V_H_hat_x*L3;

	Matrix V_r_wst(3,1);
	V_r_wst=V_r_end-V_r_h;	

	//theat 4
	theta[Index_AXIS4]=-(float)(M_PI-acos((pow(l1,2)+pow(l2,2)-pow(norm(V_r_wst),2))/(2*l1*l2)));


	Matrix V_r_m(3,1);
	V_r_m=(pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;



	//Redundant circle �b�|R
	float Rednt_cir_R = pow(l1,2)- pow( (pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
	Rednt_cir_R=sqrt(Rednt_cir_R);


	//�ꤤ���I��Elbow�V�q V_r_u
	Matrix V_shz(3,1);
	V_shz	<<0
			<<0
			<<1;

	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
	Matrix temp_cross(3,1);
	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //���~
	V_alpha_hat=temp_cross*(1/norm(temp_cross));

	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
	V_beta_hat=temp_cross*(1/norm(temp_cross));



	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha����V�M�פ�W����V�ʬۤ�
	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
	Matrix V_temp3x1(3,1);//�ݭn�g�@�ӥi�H��1�����
	Matrix V_temp4x1(4,1);
	V_temp3x1=V_beta_hat*Rednt_cir_R;
	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1

	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;


	Matrix V_R_u(3,1);
	V_R_u.Vec_export_3_row(temp);
	

	Matrix V_r_u(3,1);
	V_r_u=V_r_m+V_R_u;

	theta[Index_AXIS1]=atan2(-V_r_u(1,1),-V_r_u(3,1));//theta(1)=atan2(-V_r_u(1),-V_r_u(3));


	if (theta[Index_AXIS1] !=0) 
		theta[Index_AXIS2]=atan2(V_r_u(2,1),-V_r_u(1,1)/sin(theta[Index_AXIS1]));
	else
		theta[Index_AXIS2]=atan2(-V_r_u(2,1),V_r_u(3,1));
	


	//theat 3
	//theta(3)=atan2( sin(theta(2))*sin(theta(1))*V_r_wst(1)+cos(theta(2))*V_r_wst(2)+sin(theta(2))*cos(theta(1))*V_r_wst(3),cos(theta(1))*V_r_wst(1)-sin(theta(1))*V_r_wst(3));
	theta[Index_AXIS3]=atan2( sin(theta[Index_AXIS2])*sin(theta[Index_AXIS1])*V_r_wst(1,1)+cos(theta[Index_AXIS2])*V_r_wst(2,1)+sin(theta[Index_AXIS2])*cos(theta[Index_AXIS1])*V_r_wst(3,1),cos(theta[Index_AXIS1])*V_r_wst(1,1)-sin(theta[Index_AXIS1])*V_r_wst(3,1));



	//theat 5
	Matrix V_r_f(3,1);
	V_r_f=V_r_wst-V_r_u;

	Matrix V_Axis6(3,1);
	V_Axis6=MatrixMath::cross(V_H_hat_y,-V_r_f)*(1/norm(MatrixMath::cross(V_H_hat_y,-V_r_f)));//V_Axis6=cross(V_H_hat_y,-V_r_f)/norm(cross(V_H_hat_y,-V_r_f));

	Matrix V_r_wst_u(3,1);
	V_r_wst_u=V_r_wst+V_Axis6;

	Matrix A1_4(4,4);
	A1_4=MatrixMath::RotY(theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*MatrixMath::RotZ(theta[Index_AXIS3])*MatrixMath::Tz(-l1)*MatrixMath::RotY(theta[Index_AXIS4]);//A1_4=Ry(theta(1))*Rx(theta(2))*Rz(theta(3))*Tz(-L1)*Ry(theta(4));
	

	Matrix V_temp_f(4,1);
	Matrix V_r_wst_u_4x1(4,1);
	V_r_wst_u_4x1.Vec_ext_1_row(V_r_wst_u,1);
	

	V_temp_f=MatrixMath::Inv(A1_4)*V_r_wst_u_4x1;//V_temp_f=inv(A1_4)*[V_r_wst_u;1]; //(3.31) �o�ӬO�ɤ@�C1�W�h���N��,need fix
	theta[Index_AXIS5]=atan2(V_temp_f(2,1),V_temp_f(1,1));//theta(5)=atan2(V_temp_f(2),V_temp_f(1));

	
	//theat 6
	Matrix V_r_wst_r(3,1);
	V_r_wst_r=V_r_wst+V_H_hat_y;

	Matrix A1_5(4,4);
	A1_5=A1_4*MatrixMath::RotZ(theta[Index_AXIS5])*MatrixMath::Tz(-l2);//A1_5=A1_4*Rz(theta(5))*Tz(-L2);
	
	Matrix V_temp_g(4,4);
	Matrix V_r_wst_r_4x1(4,1);
	V_r_wst_r_4x1.Vec_ext_1_row(V_r_wst_r,1);
	
	V_temp_g=MatrixMath::Inv(A1_5)*V_r_wst_r_4x1; //V_temp_g=inv(A1_5)*[V_r_wst_r;1]; //(3.38)  �o�ӬO�ɤ@�C1�W�h���N��,need fix
	
	theta[Index_AXIS6]=atan2(V_temp_g(3,1),V_temp_g(2,1));


	//theat 7
	Matrix V_r_wst_f(3,1);
	V_r_wst_f=V_r_wst+V_H_hat_x;

	Matrix A1_6(4,4);
	A1_6=A1_5*MatrixMath::RotX(theta[Index_AXIS6]);

	Matrix V_temp_h(3,1);
	Matrix V_r_wst_f_4x1(4,1);
	V_r_wst_f_4x1.Vec_ext_1_row(V_r_wst_f,1);
	
	V_temp_h=MatrixMath::Inv(A1_6)*V_r_wst_f_4x1; //V_temp_h=inv(A1_6)*[V_r_wst_f;1]; 
	
	theta[Index_AXIS7]=atan2(-V_temp_h(1,1),-V_temp_h(3,1));//theta(7)=atan2(-V_temp_h(1),-V_temp_h(3));


	return 0;
}


//�ĤC�b��roll�b

int IK_7DOF_FB7roll(int RLHand,const float linkL[6],const float base[3],const float Pend[3],const float PoseAngle[3],const float Rednt_alpha,float* out_theta)
{
    //��X�Ѽ�initial
	float theta[7]={0};
	float tempfloat=0.0;
    //��J�s�����
	//linkL[0];//L0 �Y��ӻH
	//linkL[1];//L1 �W�uL������
	//linkL[2];//L2 �W�uL���u��
	//linkL[3];//L3 �W�uL���u��
	//linkL[4];//L4 �W�uL������
	//linkL[5];//L5 end effector

    // == �D�XH_hat_x ==//
	Matrix  R(3,3);
    R=R_z1x2y3(PoseAngle[0],PoseAngle[1],PoseAngle[2]); //alpha,beta,gamma

    Matrix V_H_hat_x(3,1);
	V_H_hat_x=Matrix::ExportCol(R,1);//���X�کԨ��ഫ������x�}�A���X��1�欰X�b�����V�q
	V_H_hat_x*=1/norm(V_H_hat_x);

    Matrix V_H_hat_z(3,1);
	V_H_hat_z=Matrix::ExportCol(R,3);//���X�کԨ��ഫ������x�}�A���X��3�欰Z�b�����V�q
	V_H_hat_z*=1/norm(V_H_hat_z);
 
	Matrix V_r_end(3,1);
	V_r_end	<<Pend[0]-base[0]//x
			<<Pend[1]-base[1]//y
			<<Pend[2]-base[2];//z
    
	Matrix V_r_h(3,1);
    V_r_h=V_H_hat_x*linkL[5]; //L5

	Matrix V_r_wst(3,1);
	V_r_wst=V_r_end-V_r_h;	
  
	// ==Axis4== //
    float ru_norm=sqrt(pow(linkL[1],2)+pow(linkL[2],2)); //L�����������
    float rf_norm=sqrt(pow(linkL[3],2)+pow(linkL[4],2));
	
    float theta_tmp=acos((pow(ru_norm,2) + pow(rf_norm,2)- pow(norm(V_r_wst),2)) / (2*ru_norm*rf_norm));
    theta[Index_AXIS4]=(float)(2*M_PI)-atan2(linkL[1],linkL[2])-atan2(linkL[4],linkL[3])-theta_tmp;

    // ==AXIS1 2== //
	Matrix V_r_m(3,1);
    V_r_m=(pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;

	//Redundant circle �b�|R
	float Rednt_cir_R = pow(ru_norm,2)- pow((pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
	Rednt_cir_R=sqrt(Rednt_cir_R);

   
	//�ꤤ���I��Elbow�V�q V_r_u
	Matrix V_shx(3,1);
	V_shx	<<1
			<<0
			<<0;

	Matrix V_shy(3,1);
	V_shy	<<0
			<<1
			<<0;

	Matrix V_shz(3,1);
	V_shz	<<0
			<<0
			<<1;

	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
	Matrix temp_cross(3,1);
	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //���~
	V_alpha_hat=temp_cross*(1/norm(temp_cross));

	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
	V_beta_hat=temp_cross*(1/norm(temp_cross));

	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha����V�M�פ�W����V�ʬۤ�
	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
	Matrix V_temp3x1(3,1);//�ݭn�g�@�ӥi�H��1�����
	Matrix V_temp4x1(4,1);
	V_temp3x1=V_beta_hat*Rednt_cir_R;
	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1
	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;

	
	Matrix V_R_u(3,1);//V_R_u=temp(1:3,1);
	V_R_u.Vec_export_3_row(temp);
	

	Matrix V_r_u(3,1);//V_r_u=V_r_m+V_R_u;
	V_r_u=V_r_m+V_R_u;

	Matrix V_r_f(3,1);// V_r_f=V_r_wst-V_r_u;
	V_r_f=V_r_wst-V_r_u;

	Matrix Vn_u_f(3,1);//Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); //ru �� rf��z�k�V�q
	temp_cross=MatrixMath::cross(V_r_u,V_r_f); 
	Vn_u_f=temp_cross*(1/norm(temp_cross));
	float theta_upoff=atan(linkL[2]/linkL[1]);
	V_temp4x1.Vec_ext_1_row(V_r_u,1);//temp=Rogridues(-theta_upoff,Vn_u_f)*[V_r_u;1];  
	temp=Rogridues(-theta_upoff,Vn_u_f)*V_temp4x1;
	Matrix V_ru_l1(3,1);//���� V_r_u  ��V_ru_l1
	V_ru_l1.Vec_export_3_row(temp);
	
	V_ru_l1=linkL[1]*V_ru_l1*(1/norm(V_ru_l1)); //�վ㦨L1����
	
	theta[Index_AXIS1]=atan2(V_ru_l1(1,1),-V_ru_l1(3,1));//theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));


	if (theta[Index_AXIS1] !=0) 
		theta[Index_AXIS2]=atan2(V_ru_l1(2,1),V_ru_l1(1,1)/sin(theta[Index_AXIS1]));
	else
		theta[Index_AXIS2]=atan2(V_ru_l1(2,1),-V_ru_l1(3,1));


	// ==AXIS3== //
	//��shy(V_r_u,V_r_f���k�V�q)�g�L1,2�b�����  �PV_r_u,V_r_f �ݭn��3�b��h��
	Matrix nV_shy;
	nV_shy=V_shy*(-1);
	V_temp4x1.Vec_ext_1_row(nV_shy,1);// V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  //�Ĥ@�b�M�j�aY�y�Ф�V�ۤ�
	temp=MatrixMath::RotY(-theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*V_temp4x1;
	
	Matrix V_n_yrot12(3,1);
	V_n_yrot12.Vec_export_3_row(temp);//V_n_yrot12=V_n_yrot12(1:3,1);

	Matrix Vn_nuf_nyrot12;
	Vn_nuf_nyrot12=MatrixMath::cross(Vn_u_f,V_n_yrot12);
	Vn_nuf_nyrot12=Vn_nuf_nyrot12*(1/norm(Vn_nuf_nyrot12));
	tempfloat=MatrixMath::dot(V_n_yrot12,Vn_u_f)*(1/norm(V_n_yrot12))*(1/norm(Vn_u_f));// temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 


	//Vn_u_f �M V_n_yrot12���k�V�q   �P V_ru_l1�P��V theta(3)�ݭn�[�t��
	if ( norm(Vn_nuf_nyrot12 - V_ru_l1*(1/norm(V_ru_l1))) < DEF_VERY_SMALL )
		theta[Index_AXIS3]=-acos(tempfloat);
	else
		theta[Index_AXIS3]=acos(tempfloat);
	
	
    // ==Axis5== //
    float theat_lowoff=atan(linkL[3]/linkL[4]); //����V_r_f �� V_rf_l4
	V_temp4x1.Vec_ext_1_row(V_r_f,1);//temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  //���� V_r_f  V_rf_l4
	temp=Rogridues(theat_lowoff,Vn_u_f)*V_temp4x1; 
	Matrix V_rf_l4(3,1);
	V_rf_l4.Vec_export_3_row(temp);// V_rf_l4=temp(1:3,1);
	V_rf_l4=V_rf_l4*linkL[4]*(1/norm(V_rf_l4)); //�վ㦨L4����
	
	//V_n_rfl4 ��V_n_rf�Φ������� ���k�V�q
	Matrix Vn_rfl4_nuf;
	Vn_rfl4_nuf=MatrixMath::cross(V_rf_l4,Vn_u_f)*(1/norm(MatrixMath::cross(V_rf_l4,Vn_u_f)));//Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
	float t_rfl4_nuf=(MatrixMath::dot(Vn_rfl4_nuf,V_r_wst)- MatrixMath::dot(Vn_rfl4_nuf,V_r_end))/pow(norm(Vn_rfl4_nuf),2);//  t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); //V_n_rf,V_n_rfl4�����W�A�B�g�LV_r_end�I�����u�ѼƦ���t ��rfl4_nuf
	Matrix Vproj_end_rfl4_nuf;
	Vproj_end_rfl4_nuf=V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;//V_r_end �u��V_n_rfl4,V_n_rf�����k�V�q��v�b�����W���I
	Matrix V_wst_to_projend_rfl4_nuf;
	V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;
	
	//V_n_rfl4 ��V_n_rf�Φ������� ���k�V�q
	//����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
	tempfloat=MatrixMath::dot(V_rf_l4,V_wst_to_projend_rfl4_nuf)*(1/norm(V_rf_l4))*(1/norm(V_wst_to_projend_rfl4_nuf));//temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);
	if (abs(tempfloat-1) < DEF_VERY_SMALL) 
	{
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}	

	Matrix Vn_rfl4_WstToProjEndRfl4Nuf;
	Vn_rfl4_WstToProjEndRfl4Nuf=MatrixMath::cross(V_rf_l4*(1/norm(V_rf_l4)) , V_wst_to_projend_rfl4_nuf*(1/norm(V_wst_to_projend_rfl4_nuf)));
	Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf*(1/norm(Vn_rfl4_WstToProjEndRfl4Nuf));
 
	//�����k�V�q �M Vn_rfl4_nuf  �P��n�[�t��  �P�_theta5�n���W�Ω��U��
	if (norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf) < DEF_VERY_SMALL)
        theta[Index_AXIS5]=-acos(tempfloat); 
    else
        theta[Index_AXIS5]=acos(tempfloat); 

	// ==Axis6== //
	V_temp4x1.Vec_ext_1_row(Vn_u_f,1);
	temp=Rogridues(-theta[Index_AXIS5],Vn_rfl4_nuf)*V_temp4x1; 
	Matrix Vn_nuf_rotx5_along_NRfl4Nuf(3,1);
	Vn_nuf_rotx5_along_NRfl4Nuf.Vec_export_3_row(temp);  //Vn_nuf_rotx5_along_NRfl4Nuf=temp(1:3,1);//nuf �u�� Vn_rfl4_nuf �����5�b���ױo���v�I�P�ؼ��I�������k�V�q
	Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf*(1/norm(Vn_nuf_rotx5_along_NRfl4Nuf));

	Matrix V_wst_to_end;
	V_wst_to_end=V_r_end-V_r_wst;

	Matrix Vn_WstToEnd_WstToProjEndRfl4Nuf;
	Vn_WstToEnd_WstToProjEndRfl4Nuf=MatrixMath::cross(V_wst_to_end,V_wst_to_projend_rfl4_nuf);//V_wst_to_projend �M V_wst_to_end���k�V�q
	Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf*(1/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf));
    
	//�Q�Ϊk�V�q��V �P�_theta6�����V
	tempfloat=MatrixMath::dot(V_wst_to_projend_rfl4_nuf,V_wst_to_end)*(1/norm(V_wst_to_projend_rfl4_nuf)*(1/norm(V_wst_to_end))); //temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);

	if (norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_VERY_SMALL)
        theta[Index_AXIS6]=-acos(tempfloat); 
    else
        theta[Index_AXIS6]=acos(tempfloat); 
	
	// ==Axis7== //
	Matrix V_x_rot1to6;
	V_temp4x1.Vec_ext_1_row(V_shx,1);
	V_x_rot1to6=MatrixMath::RotY(-theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*V_temp4x1;//V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  //�Ĥ@�b�M�j�aZ�y�Ф�V�ۤ�
	
	//V_shx�g�L1to�b����������ӭn�P���I�y�Шt��Z�b�K��
	temp=Rogridues(theta[Index_AXIS3],V_ru_l1*(1/norm(V_ru_l1)))*V_x_rot1to6;		//temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
	temp=Rogridues(theta[Index_AXIS4],Vn_u_f*(1/norm(Vn_u_f)))*temp;				//temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
	temp=Rogridues(theta[Index_AXIS5],Vn_rfl4_nuf*(1/norm(Vn_rfl4_nuf)))*temp;	//temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
	temp=Rogridues(theta[Index_AXIS6],Vn_nuf_rotx5_along_NRfl4Nuf)*temp;			//temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
	V_x_rot1to6.Vec_export_3_row(temp);										//V_x_rot1to6=temp(1:3,1); 
	V_x_rot1to6=V_x_rot1to6*(1/norm(V_x_rot1to6));

	//xrot1to6 �M V_H_hat_z ���k�V�q�ӧP�_��7�b�����V
	Matrix Vn_xrot1to6_VHhatz;
	Vn_xrot1to6_VHhatz=MatrixMath::cross(V_x_rot1to6,V_H_hat_z);
	Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz*(1/norm(Vn_xrot1to6_VHhatz));

	//V_shx�g�L123456�b�����M���I�y�Шt��Z�b�ٮt�X��
	theta[Index_AXIS7]=acos(MatrixMath::dot(V_x_rot1to6,V_H_hat_z)*(1/norm(V_x_rot1to6))*(1/norm(V_H_hat_z)));// theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));

	if (norm(Vn_xrot1to6_VHhatz - V_H_hat_x) <  DEF_VERY_SMALL)
        theta[Index_AXIS7]=theta[Index_AXIS7];
    else
        theta[Index_AXIS7]=-theta[Index_AXIS7];  


	// ==���k���1�b��V�ۤ�== //
    if (RLHand == DEF_LEFT_HAND) //����M�k��Ĥ@�b��V�ۤ�
        theta[Index_AXIS1]=-theta[Index_AXIS1];
    
	//==output degree==//
	memcpy(out_theta,theta,sizeof(theta));

	return 0;
}

//================================================================================//
//==prevent angle over constrain.If over occur,over_index shows which axis over ==//
//================================================================================//
bool AngleOverConstrain(int RLHand, const float theta[MAX_AXIS_NUM],int *OverIndex)
{
	bool bOver=false;
	*OverIndex=NULL;

	if (RLHand ==DEF_RIGHT_HAND)
	{
		for(int index=Index_AXIS1;index<=Index_AXIS7;index++)
		{
			if (theta[index]<grobot_lim_rad_R_Low[index] || theta[index]>grobot_lim_rad_R_High[index])  
			{
				bOver=true;
				*OverIndex=index;
				break;
			}
		}
	}
	else if(RLHand ==DEF_LEFT_HAND)
	{
		for(int index=Index_AXIS1;index<=Index_AXIS7;index++)
		{
			if (theta[index]<grobot_lim_rad_L_Low[index] || theta[index]>grobot_lim_rad_L_High[index])  
			{
				bOver=true;
				*OverIndex=index;
				break;
			}
		}
	}


	return bOver;
}


int MoveToPoint(int RLHand,float Pend[3],float Pose_deg[3],float redant_alpha_deg)  //���ӭn���@�ӳt�װѼ�
{
	const float linkL[6]={L0,L1,L2,L3,L4,L5};
	float base[3]={0.0};
	//float Pend[3]={x,y,z};
	float Pose_rad[3]={Pose_deg[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Pose_deg[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Pose_deg[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};
	float Rednt_alpha=redant_alpha_deg*DEF_RATIO_DEG_TO_RAD;
	float theta[7]={0};
	int rt=0;

	//inverse kinematics
	if(RLHand==DEF_RIGHT_HAND)
		base[DEF_Y]=-L0;
	else if(RLHand==DEF_LEFT_HAND)
		base[DEF_Y]=L0;

	rt= IK_7DOF_FB7roll(RLHand,linkL,base,Pend,Pose_rad,Rednt_alpha,theta);
	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		DBGMSG(("axis %d=%f\n",gMapAxisNO[i],theta[i]))
	
	//==prevent angle over constrain
	int over_index=0;
	bool bOver=AngleOverConstrain(RLHand,theta,&over_index);
	if(bOver)
	{
		if(RLHand==DEF_RIGHT_HAND)
			DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_R_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_R_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		else if(RLHand==DEF_LEFT_HAND)
			DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_L_High[over_index]*DEF_RATIO_RAD_TO_DEG))

		return 1;
	}

	//output to motor
	unsigned short int velocity[MAX_AXIS_NUM]={10,10,10,10,10,10,10};
	
	rt=Output_to_Dynamixel(RLHand,theta,velocity); 

	return 0;
}


int MoveToPoint_Dual(float Pend_R[3],float Pose_deg_R[3],float Rednt_alpha_deg_R,float Pend_L[3],float Pose_deg_L[3],float Rednt_alpha_deg_L)  //���ӭn���@�ӳt�װѼ�
{
	const float linkL[6]={L0,L1,L2,L3,L4,L5};
	float base_R[3]={0,-L0,0};
	float base_L[3]={0,L0,0};

	float Pose_rad_R[3]={Pose_deg_R[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Pose_deg_R[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Pose_deg_R[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};
	float Pose_rad_L[3]={Pose_deg_L[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Pose_deg_L[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Pose_deg_L[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};

	float Rednt_alpha_rad_R=Rednt_alpha_deg_R*DEF_RATIO_DEG_TO_RAD;
	float Rednt_alpha_rad_L=Rednt_alpha_deg_L*DEF_RATIO_DEG_TO_RAD;

	float theta_R[7]={0};
	float theta_L[7]={0};
	int rt=0;
	int over_index=0;
	bool bOver=false;

	//inverse kinematics right hand
	rt= IK_7DOF_FB7roll(DEF_RIGHT_HAND,linkL,base_R,Pend_R,Pose_rad_R,Rednt_alpha_rad_R,theta_R);
	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		DBGMSG(("axis %d=%f\n",gMapAxisNO[i],theta_R[i]))

	//==prevent angle over constrain right hand 
	over_index=0;
	bOver=AngleOverConstrain(DEF_RIGHT_HAND,theta_R,&over_index);
	if(bOver)
	{
		DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta_R[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_R_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_R_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		return 1;
	}

	//inverse kinematics left hand
	rt= IK_7DOF_FB7roll(DEF_LEFT_HAND,linkL,base_L,Pend_L,Pose_rad_L,Rednt_alpha_rad_L,theta_L);
	for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		DBGMSG(("axis %d=%f\n",gMapAxisNO[i],theta_L[i]))
	
	//==prevent angle over constrain left hand 
	over_index=0;
	bOver=AngleOverConstrain(DEF_LEFT_HAND,theta_L,&over_index);
	if(bOver)
	{
		DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta_L[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_L_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		return 1;
	}


	//output to motor
	unsigned short int velocity_R[MAX_AXIS_NUM]={20,20,20,20,20,20,20};
	unsigned short int velocity_L[MAX_AXIS_NUM]={20,20,20,20,20,20,20};
	
	rt=Output_to_Dynamixel_Dual(theta_R,velocity_R,theta_L,velocity_L); 

	return 0;
}

int syncWrite_x86(unsigned short int start_addr, unsigned short int data_length, unsigned short int *param, unsigned short int param_length) // WORD(16bit) syncwrite() for DXL  stanley
{
    //syncWrite_u16base(GOAL_POSITION,2,SyncPage1,21);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);
	dxl_set_txpacket_id(BROADCAST_ID);
	dxl_set_txpacket_instruction(INST_SYNC_WRITE);
	dxl_set_txpacket_parameter(0, start_addr);
	dxl_set_txpacket_parameter(1,data_length*2);

	int slaveNum=param_length/(data_length+1);
    int OneRawByte=(1+data_length*2);//ID(1byte) + worddata*len(2byte*len)

    int i=0; //offset of slave number(number of row)
    int j=0; //offset of data in raw
    int k=1;//offset of int *param 
    int index=0;

    for( i=0; i<slaveNum; i++ )
    { 
        index=OneRawByte*i+2;
        dxl_set_txpacket_parameter(index,(unsigned char)param[i*(data_length+1)]);//ID
        k=1;

        for(j=1;j<OneRawByte;j+=2)
        {
            dxl_set_txpacket_parameter(index+j,(unsigned char)(param[i*(data_length+1)+k]&0xff)); //DATA L    
            dxl_set_txpacket_parameter(index+j+1,(unsigned char)(param[i*(data_length+1)+k]>>8)); //DATA H
            k++;
        }
    } 
	
    dxl_set_txpacket_length(OneRawByte*slaveNum+4);

	//for(int i=0;i<50;i++)
	//	printf("gbInstructionPacket[%d]=%x\n",i,gbInstructionPacket[i]);//stanley test

    dxl_txrx_packet();
	int CommStatus =0;
	CommStatus=dxl_get_result();

    return CommStatus;

}

int setPosition_x86(int ServoID, int Position, int Speed)//stanley
{
	dxl_set_txpacket_id(ServoID);
	dxl_set_txpacket_instruction(INST_WRITE);
	dxl_set_txpacket_parameter(0,GOAL_POSITION);
	dxl_set_txpacket_parameter(1,(unsigned char)dxl_get_lowbyte(Position));
	dxl_set_txpacket_parameter(2,(unsigned char)dxl_get_highbyte(Position));
	dxl_set_txpacket_parameter(3,(unsigned char)dxl_get_lowbyte(Speed));
	dxl_set_txpacket_parameter(4,(unsigned char)dxl_get_highbyte(Speed));
	dxl_set_txpacket_length(7);

	dxl_txrx_packet();

	int CommStatus =0;
	CommStatus=dxl_get_result();

	return CommStatus;
}


int DXL_Initial_x86()
{
	int rt=0;
	const int default_portnum=5;
	const int default_baudnum=1;

	rt=dxl_initialize( default_portnum,default_baudnum);
	
	return rt;
}

int DXL_Terminate_x86()
{
	dxl_terminate();
	
	return 0;
}

//========================
//==Modbus control gripper
//========================
modbus_t *ctx;  //CLI���঳�����ܼƩҥH�|��LNK4248

int Initial_Modbus()
{
	
	ctx = modbus_new_rtu("COM8",460800,'N',8,1);
	
	//modbus_set_debug(ctx, 1);//�|print���檬�p
 
	//�]�wdebug�������~
	int rt=0;
	rt=modbus_connect(ctx);//�s��modbus
 
	modbus_set_slave(ctx,1);//��J�]��ID��1
 
	
	//�P�_�O�_�����`�s�u
	if (rt == -1)
	{
		printf("modbus link error");
		modbus_free(ctx);
		return -1;
	}
 
	uint16_t dest[10]={0};
	/*modbus_read_registers(ctx, 4001,10, dest);
 
	modbus_write_register(ctx,4001,1000);

	modbus_read_registers(ctx, 4001,10, dest);*/
	//modbus_write_register(ctx,0x0259,0x0003); 
	//�g�J���PLC����m modbus_write_register(usb port,PLC��m,�ƭ�)
 
	///*  �����y�{ */
	//modbus_close(ctx);
 //
	//modbus_free(ctx);
	return 0;
}

void Terminate_Modbus()
{
	/*  �����y�{ */
	modbus_close(ctx);
	modbus_free(ctx);
}

#define DEF_HOLD 1
#define DEF_RELEASE 0

int GripperHold(int RLHand,bool Hold)
{
	//address4001  //RIGHT_HAND
	//address4002  //Left_HAND

	if(RLHand==DEF_RIGHT_HAND)
	{
		if(Hold)
			modbus_write_register(ctx,4000,DEF_HOLD);
		else
			modbus_write_register(ctx,4000,DEF_RELEASE);
	}
	else if(RLHand==DEF_LEFT_HAND)
	{
		if(Hold)
			modbus_write_register(ctx,4001,DEF_HOLD);
		else
			modbus_write_register(ctx,4001,DEF_RELEASE);
	}

	uint16_t dest[10]={0};
	modbus_read_registers(ctx, 4000,10, dest);
	return 0;
}



//=========================================
//==LattePanda Arduino Leonardo for Gripper
//==using C# "LattePanda Firmata
//=========================================
using namespace  LattePandaFirmata;
#using "LattePandaFirmata.dll"
#include <windows.h> //sleep�ϥ�
#pragma warning (disable: 4538)

#define DEF_LATTE_D2_GRIPPER_R1	6
#define DEF_LATTE_D3_GRIPPER_R2	8
#define DEF_LATTE_D4_GRIPPER_L1	10
#define DEF_LATTE_D5_GRIPPER_L2	12
#define DEF_LATTE_D13_LED		13
ref class GlobalObjects
{
public:
	static Arduino ^arduino=nullptr;

	static int Initial()
	{
		arduino = gcnew Arduino();
		arduino->pinMode(DEF_LATTE_D2_GRIPPER_R1, arduino->OUTPUT);//Set the digital pin 6 as output      
		arduino->pinMode(DEF_LATTE_D3_GRIPPER_R2, arduino->OUTPUT);//Set the digital pin 8 as output
		arduino->pinMode(DEF_LATTE_D4_GRIPPER_L1, arduino->OUTPUT);//Set the digital pin 10 as output      
		arduino->pinMode(DEF_LATTE_D5_GRIPPER_L2, arduino->OUTPUT);//Set the digital pin 12 as output

		//arduino->pinMode(DEF_LATTE_D13_LED, arduino->OUTPUT);//Set the digital pin 13 as output just test

		return 0;
	}
	static int Close()
	{
		arduino->Close();
		return 0;
	}
};


int Gripper_LattePanda_Initial()
{
	 GlobalObjects::Initial();
	 return 0;
}

void Gripper_LattePanda_Close()
{
	 GlobalObjects::Close();
}

int Gripper_LattePanda_Hold(int RLHand,bool Hold)
{
	//=========
	//=Test LED
	//=========
    //for(int i=0;i<5;i++)
    //{
    //    // ==== set the led on or off  
    //    GlobalObjects::arduino->digitalWrite(DEF_LATTE_D13_LED, GlobalObjects::arduino->HIGH);//set the LED�@on  
    //    Sleep(1000);//delay a seconds  
    //    GlobalObjects::arduino->digitalWrite(DEF_LATTE_D13_LED, GlobalObjects::arduino->LOW);//set the LED�@off  
    //    Sleep(1000);//delay a seconds  
    //}


	if(RLHand==DEF_RIGHT_HAND)
	{
		if(Hold)
		{
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R1, GlobalObjects::arduino->HIGH);
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D3_GRIPPER_R2, GlobalObjects::arduino->LOW);
		}
		else
		{
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R1, GlobalObjects::arduino->LOW);
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D3_GRIPPER_R2, GlobalObjects::arduino->HIGH);
		}
	}
	else if(RLHand==DEF_LEFT_HAND)
	{
		if(Hold)
		{
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D4_GRIPPER_L1, GlobalObjects::arduino->HIGH);
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D5_GRIPPER_L2, GlobalObjects::arduino->LOW);
		}
		else
		{
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D4_GRIPPER_L1, GlobalObjects::arduino->LOW);
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D5_GRIPPER_L2, GlobalObjects::arduino->HIGH);
		}
	}
	return 0;
}