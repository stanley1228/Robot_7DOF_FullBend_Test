


#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>  //���hGUI
#include <opencv2/legacy/legacy.hpp>


#include "Robot_7DOF_FB.h"

#include <vector>

#include "Matrix.h"

#include "MatrixMath.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>


#include "dynamixel.h"
#include <windows.h> //QPDelay_ms�ϥ�
#include<fstream>//�g�ɨϥ�

#pragma comment(lib,"dynamixel.lib") 


#pragma comment(lib,"opencv_core2413d.lib")	//��ܹϤ��ϥ�
#pragma comment(lib,"opencv_highgui2413d.lib") 
#pragma comment(lib,"opencv_imgproc2413d.lib")
#pragma comment(lib,"opencv_video2413d.lib")
#pragma comment(lib,"opencv_legacy2413d.lib")
#pragma comment(lib,"opencv_objdetect2413d.lib")


				
//#define F446RE_GRIPPER_EN
#define CHECK_CARTESIAN_PATH 
//#define GRIPPER_ON_LATTE
#define MOVETOPOINT_DUAL
//#define CHECK_JOINT_PATH  
//#define MOVE_TO_INITIAL_POINT
//#define RECORD_JOINT_ANGLE
//#define DEF_WAIT_ENTER

const float gCycleT=0.5;

using namespace cv;
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

void PID_Setting_Dual()
{
	const short int P_GAIN_R[MAX_AXIS_NUM]=
	{
		AXISR1_P_GAIN,
		AXISR2_P_GAIN,
		AXISR3_P_GAIN,
		AXISR4_P_GAIN,
		AXISR5_P_GAIN,
		AXISR6_P_GAIN,
		AXISR7_P_GAIN
	};

	const short int P_GAIN_L[MAX_AXIS_NUM]=
	{
		AXISL1_P_GAIN,
		AXISL2_P_GAIN,
		AXISL3_P_GAIN,
		AXISL4_P_GAIN,
		AXISL5_P_GAIN,
		AXISL6_P_GAIN,
		AXISL7_P_GAIN
	};
	const short int I_GAIN_R[MAX_AXIS_NUM]=
	{
		AXISR1_I_GAIN,
		AXISR2_I_GAIN,
		AXISR3_I_GAIN,
		AXISR4_I_GAIN,
		AXISR5_I_GAIN,
		AXISR6_I_GAIN,
		AXISR7_I_GAIN
	};
	const short int I_GAIN_L[MAX_AXIS_NUM]=
	{
		AXISL1_I_GAIN,
		AXISL2_I_GAIN,
		AXISL3_I_GAIN,
		AXISL4_I_GAIN,
		AXISL5_I_GAIN,
		AXISL6_I_GAIN,
		AXISL7_I_GAIN
	};
	const short int D_GAIN_R[MAX_AXIS_NUM]=
	{
		AXISR1_D_GAIN,
		AXISR2_D_GAIN,
		AXISR3_D_GAIN,
		AXISR4_D_GAIN,
		AXISR5_D_GAIN,
		AXISR6_D_GAIN,
		AXISR7_D_GAIN
	};
	const short int D_GAIN_L[MAX_AXIS_NUM]=
	{
		AXISL1_D_GAIN,
		AXISL2_D_GAIN,
		AXISL3_D_GAIN,
		AXISL4_D_GAIN,
		AXISL5_D_GAIN,
		AXISL6_D_GAIN,
		AXISL7_D_GAIN
	};

	//==write PID para  ==//
	int i=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//==Set P==//
		dxl_write_byte(gMapRAxisID[i],P_GAIN,P_GAIN_R[i]);//right
		dxl_write_byte(gMapLAxisID[i],P_GAIN,P_GAIN_L[i]);//left
	
		//==Set I==//
		dxl_write_byte(gMapRAxisID[i],I_GAIN,I_GAIN_R[i]);//right
		dxl_write_byte(gMapLAxisID[i],I_GAIN,I_GAIN_L[i]);//left  

		//==Set D==//
		dxl_write_byte(gMapRAxisID[i],D_GAIN,D_GAIN_R[i]);//right
		dxl_write_byte(gMapLAxisID[i],D_GAIN,D_GAIN_L[i]);//left
	}


	//==read and check right hand==//
	int	txrx_result=0;
	short int p_gain=0;
	short int i_gain=0,d_gain=0;
	short int multi_turn_offset=0;
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		printf("===AXIS_R%d===\n",gMapAxisNO[i]);

		//==P GAIN==//
		p_gain = dxl_read_byte(gMapRAxisID[i], P_GAIN);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read P_GAIN error=%d\n",txrx_result);
		else
			printf("P_GAIN=%d\n",p_gain);
	
		//==I GAIN==//
		i_gain=dxl_read_byte(gMapRAxisID[i],I_GAIN);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read I_GAIN error=%d\n",txrx_result);
		else	
			printf("I_GAIN=%d\n",i_gain);

		//==D GAIN==//
		d_gain=dxl_read_byte(gMapRAxisID[i],D_GAIN);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed Read D_GAIN error=%d\n",txrx_result);
		else	
			printf("D_GAIN=%d\n",d_gain);
	}	
	//==read and check left hand==//
	for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		printf("===AXIS_L%d===\n",gMapAxisNO[i]);

		//==P GAIN==//
		p_gain = dxl_read_byte(gMapLAxisID[i], P_GAIN);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read P_GAIN error=%d\n",txrx_result);
		else
			printf("P_GAIN=%d\n",p_gain);
	
		//==I GAIN==//
		i_gain=dxl_read_byte(gMapLAxisID[i],I_GAIN);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed read I_GAIN error=%d\n",txrx_result);
		else	
			printf("I_GAIN=%d\n",i_gain);

		//==D GAIN==//
		d_gain=dxl_read_byte(gMapLAxisID[i],D_GAIN);
		txrx_result = dxl_get_result();
		if(txrx_result!=COMM_RXSUCCESS)
			printf("Failed Read D_GAIN error=%d\n",txrx_result);
		else	
			printf("D_GAIN=%d\n",d_gain);
	}	

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

void WaitMotionDoneDual()
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


#define DEF_TYPE_LINE		1
#define DEF_TYPE_ARC		2
#define DEF_TYPE_GRIP_NOACT	3
#define DEF_TYPE_GRIP_HOLD	4
#define DEF_TYPE_GRIP_REL	5
#define DEF_TYPE_SP_ON		6 
#define DEF_TYPE_SP_OFF		7
#define DEF_TYPE_FL_UP		8 
#define DEF_TYPE_FL_DOWN	9
#define DEF_TYPE_TRIM_ON	10 
#define DEF_TYPE_TRIM_OFF	11

//enum{
//	S_INITIAL=0,
//	S_FL_UP_1,//�����}��1
//	S_R_HOLD_L_HOLD_1,
//	S_FL_DOWN_1,//�����}��1
//	S_SP_ON_1,
//	S_R_FX_L_FX_1,
//	S_SP_OFF_1,
//	S_R_KEEP_L_REL_1,
//	S_R_KEEP_L_FY_1,
//	S_R_KEEP_L_FX_1,
//	S_R_KEEP_L_BY_1,
//	S_R_KEEP_L_HOLD_1,
//	S_FL_UP_2,//�����}��2
//	S_R_FCIRX_L_BCIRX_1,
//	S_FL_DOWN_2,//�����}��2
//	S_R_REL_L_HOLD_1,
//	S_R_BXY_L_KEEP_1,
//	S_R_BX_L_KEEP_1,
//	S_R_FXY_L_KEEP_1,
//	S_R_HOLD_L_KEEP_1,
//};




//#include <array>
//void SetArray(std::array<float,m_ARR_SIZE> &arr,float x,float y,float z,float alpha,float beta,float gamma,float rednt_alpha)
//{
//		arr.at(DEF_X)=x;
//		arr.at(DEF_Y)=y;;
//		arr.at(DEF_Z)=z;
//		arr.at(DEF_ALPHA)=alpha;
//		arr.at(DEF_BETA)=beta;
//		arr.at(DEF_GAMMA)=gamma;
//		arr.at(DEF_REDNT_ALPHA)=rednt_alpha;
//}
//
//std::array<float, 3> operator-( std::array<float, 3>& arr1,  std::array<float, 3>& arr2) 
//{
//    std::array<float, 3U> dst;
//    for ( auto itArr1 = arr1.begin(), itArr2 = arr2.begin(), itDst = dst.begin(); itArr1 != arr1.end(); ++itArr1, ++itArr2, ++itDst )
//        *itDst = *itArr1 - *itArr2;
//    return dst;
//}




//template <class T, int N>
//std::array<float, m_ARR_SIZE> operator-(std::array<float, m_ARR_SIZE>& arr1, std::array<float, m_ARR_SIZE>& arr2) 
//{
//    std::array<float, m_ARR_SIZE> dst;
//    for ( auto itArr1 = arr1.begin(), itArr2 = arr2.begin(), itDst = dst.begin(); itArr1 != arr1.end(); ++itArr1, ++itArr2, ++itDst )
//        *itDst = *itArr1 - *itArr2;
//    return dst;
//}
//
//std::array<float, m_ARR_SIZE> operator+(std::array<float, m_ARR_SIZE>& arr1, std::array<float, m_ARR_SIZE>& arr2) 
//{
//    std::array<float, m_ARR_SIZE> dst;
//    for ( auto itArr1 = arr1.begin(), itArr2 = arr2.begin(), itDst = dst.begin(); itArr1 != arr1.end(); ++itArr1, ++itArr2, ++itDst )
//        *itDst = *itArr1 + *itArr2;
//    return dst;
//}
//
//std::array<float, m_ARR_SIZE> operator*(std::array<float, m_ARR_SIZE>& arr1, float m) 
//{
//    std::array<float, m_ARR_SIZE> dst;
//    for ( auto itArr1 = arr1.begin(),itDst = dst.begin(); itArr1 != arr1.end(); ++itArr1, ++itDst )
//        *itDst = (*itArr1)*m;
//    return dst;
//}

//=====================================
//�ۤv�إߪ��}�Cclass�A���F�n�}�C�ۥ[��
//=====================================
CStaArray::CStaArray()
{
	m_ARR_SIZE=7;
	SetArray( 0, 0, 0, 0, 0, 0, 0);
}
CStaArray::CStaArray(float x,float y,float z,float alpha,float beta,float gamma,float rednt_alpha)
{
	m_ARR_SIZE=7;
	SetArray( x, y, z, alpha, beta, gamma, rednt_alpha);
}

float CStaArray::at(int index)
{
	return m_arr[index];
}

void CStaArray::SetArray(float x,float y,float z,float alpha,float beta,float gamma,float rednt_alpha)
{
	m_arr[DEF_X]=x;
	m_arr[DEF_Y]=y;;
	m_arr[DEF_Z]=z;
	m_arr[DEF_ALPHA]=alpha;
	m_arr[DEF_BETA]=beta;
	m_arr[DEF_GAMMA]=gamma;
	m_arr[DEF_REDNT_ALPHA]=rednt_alpha;
}


CStaArray CStaArray::operator*(float k)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]*k;
    
	return temp;
}

CStaArray CStaArray::operator+(CStaArray &other)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]+other.m_arr[i];
    
	return temp;
}

CStaArray CStaArray::operator-(CStaArray &other)
{
	CStaArray temp;
    for (int i=0;i<m_ARR_SIZE;i++)
        temp.m_arr[i]=m_arr[i]-other.m_arr[i];
    
	return temp;
}


CStaArray gNeedle_RobotF(350,-300,30,0,0,0,0);//�w�I�b���u���Шt��m   
CStaArray gNeedle_ini_Plate(30,-30,0,0,0,0,0);//�U�w�I�b�[�lplate�y�Шt�W����l�I
CStaArray gTranFrameToRobot=gNeedle_RobotF-gNeedle_ini_Plate;//�Q�Ψ�Ӫ��t�ȥh�����
fstream gfileR;
fstream gfileL;

void IKOutputToArm(CStaArray &PathPlanPoint_R,CStaArray &PathPlanPoint_L)
{
	static float static_abst=0;
	
	//==Output to arm ==//
	float vel_deg_R=30;
	float vel_deg_L=30;
		
#ifdef MOVETOPOINT_DUAL
	MoveToPoint_Dual(PathPlanPoint_R.m_arr,vel_deg_R,PathPlanPoint_L.m_arr,vel_deg_L);  //�ϥέ쥻matrix�j��20ms    �אּopencv matri�ᥭ��2.5ms �]��cycle time�Q��10ms  
#endif
		printf("Pend_R=[%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f],Pend_L=[%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f]\n",PathPlanPoint_R.at(DEF_X),PathPlanPoint_R.at(DEF_Y),PathPlanPoint_R.at(DEF_Z),PathPlanPoint_R.at(DEF_ALPHA),PathPlanPoint_R.at(DEF_BETA),PathPlanPoint_R.at(DEF_GAMMA),PathPlanPoint_R.at(DEF_REDNT_ALPHA),PathPlanPoint_L.at(DEF_X),PathPlanPoint_L.at(DEF_Y),PathPlanPoint_L.at(DEF_Z),PathPlanPoint_L.at(DEF_ALPHA),PathPlanPoint_L.at(DEF_BETA),PathPlanPoint_L.at(DEF_GAMMA),PathPlanPoint_L.at(DEF_REDNT_ALPHA));

		
		//==�T�{�y���I==//
#ifdef CHECK_CARTESIAN_PATH
		char buffer[100];
		int k=0;
		k=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",static_abst,PathPlanPoint_R.at(DEF_X),PathPlanPoint_R.at(DEF_Y),PathPlanPoint_R.at(DEF_Z));
		gfileR.write(buffer,k);

		k=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",static_abst,PathPlanPoint_L.at(DEF_X),PathPlanPoint_L.at(DEF_Y),PathPlanPoint_L.at(DEF_Z));
		gfileL.write(buffer,k);
#endif 

#ifdef RECORD_JOINT_ANGLE
		//==record para==//
		float pos_deg_R[MAX_AXIS_NUM]={0};
		float pos_deg_L[MAX_AXIS_NUM]={0};
		float pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
		float pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
		int n=0;
		int rt=0;

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
			gfileR.write(buffer,n);
			
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
			gfileR.write(buffer,n);
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
			gfileL.write(buffer,n);
			
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
			gfileL.write(buffer,n);
		}
#endif

		static_abst+=gCycleT;


}

void LineMoveTo(CStaArray &L_starP,CStaArray &L_endP,CStaArray &R_starP,CStaArray &R_endP,float CostTime)
{
	//==transfer frame coordinate to robot coordinate==//
	for(int i=0;i<3;i++)
	{
		L_starP.m_arr[i]=L_starP.at(i)+gTranFrameToRobot.at(i);
		L_endP.m_arr[i]=L_endP.at(i)+gTranFrameToRobot.at(i);
		R_starP.m_arr[i]=R_starP.at(i)+gTranFrameToRobot.at(i);
		R_endP.m_arr[i]=R_endP.at(i)+gTranFrameToRobot.at(i);
	}

	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);

	CStaArray PathPlanPoint_R(0,0,0,0,0,0,0);
	CStaArray PathPlanPoint_L(0,0,0,0,0,0,0);
	
	for(float t=0;t<=CostTime;t+=gCycleT)
	{
		QueryPerformanceCounter(&nBeginTime); //Record cycle start time

		//==calculate cartisian point==//
		PathPlanPoint_R=R_starP+(R_endP-R_starP)*(t/CostTime);
		PathPlanPoint_L=L_starP+(L_endP-L_starP)*(t/CostTime);
		
		//==calculate IK and Output to arm==//
		IKOutputToArm(PathPlanPoint_R,PathPlanPoint_L);

		do
		{
			Sleep(0);
			QueryPerformanceCounter(&nEndTime);
			//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
		}
		while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart < gCycleT);
	}
    
   
}
	
void RotateMoveTo(	CStaArray &L_starP,
					CStaArray &L_endP,
					CStaArray &R_starP,
					CStaArray &R_endP,
					CStaArray &arc_cen,
					float rot_rad,
					float CostTime)
{
	
	for(int i=0;i<3;i++)
	{
		arc_cen.m_arr[i]=arc_cen.at(i)+gTranFrameToRobot.at(i);
		L_starP.m_arr[i]=L_starP.at(i)+gTranFrameToRobot.at(i);
		L_endP.m_arr[i]=L_endP.at(i)+gTranFrameToRobot.at(i);
		R_starP.m_arr[i]=R_starP.at(i)+gTranFrameToRobot.at(i);
		R_endP.m_arr[i]=R_endP.at(i)+gTranFrameToRobot.at(i);
	}

	//�k���P���|
	float rR=sqrt(pow(R_starP.at(DEF_X)-arc_cen.at(DEF_X),2)+pow(R_starP.at(DEF_Y)-arc_cen.at(DEF_Y),2));//�k�����b�|
	float ini_rad_R=DEF_PI+atan((R_starP.at(DEF_Y)-arc_cen.at(DEF_Y))/(R_starP.at(DEF_X)-arc_cen.at(DEF_X)));//����ɪ��_�l���ਤ��

	//�����P���|
	float rL=sqrt(pow(L_starP.at(DEF_X)-arc_cen.at(DEF_X),2)+pow(L_starP.at(DEF_Y)-arc_cen.at(DEF_Y),2));
	float ini_rad_L=atan((L_starP.at(DEF_Y)-arc_cen.at(DEF_Y))/(L_starP.at(DEF_X)-arc_cen.at(DEF_X)));
	
	LARGE_INTEGER nFreq;
	LARGE_INTEGER nBeginTime;
	LARGE_INTEGER nEndTime;
	QueryPerformanceFrequency(&nFreq);

	CStaArray PathPlanPoint_R(0,0,0,0,0,0,0);
	CStaArray PathPlanPoint_L(0,0,0,0,0,0,0);

	for(float t=0;t<=CostTime;t+=gCycleT)
	{
		QueryPerformanceCounter(&nBeginTime); //Record cycle start time

		PathPlanPoint_R.m_arr[DEF_X]=arc_cen.at(DEF_X)+(float)(rR*(cos(rot_rad*t/CostTime + ini_rad_R))); 
		PathPlanPoint_R.m_arr[DEF_Y]=arc_cen.at(DEF_Y)+(float)(rR*(sin(rot_rad*t/CostTime + ini_rad_R))); 
		PathPlanPoint_R.m_arr[DEF_Z]=arc_cen.at(DEF_Z);
		PathPlanPoint_R.m_arr[DEF_ALPHA]=R_starP.at(DEF_ALPHA)+(R_endP.at(DEF_ALPHA)-R_starP.at(DEF_ALPHA))*t/CostTime; 
		PathPlanPoint_R.m_arr[DEF_BETA]=R_starP.at(DEF_BETA)+(R_endP.at(DEF_BETA)-R_starP.at(DEF_BETA))*t/CostTime;
		PathPlanPoint_R.m_arr[DEF_GAMMA]=R_starP.at(DEF_GAMMA)+(R_endP.at(DEF_GAMMA)-R_starP.at(DEF_GAMMA))*t/CostTime;

		PathPlanPoint_L.m_arr[DEF_X]=arc_cen.at(DEF_X)+(float)(rL*(cos(rot_rad*t/CostTime + ini_rad_L))); 
		PathPlanPoint_L.m_arr[DEF_Y]=arc_cen.at(DEF_Y)+(float)(rL*(sin(rot_rad*t/CostTime + ini_rad_L))); 
		PathPlanPoint_L.m_arr[DEF_Z]=arc_cen.at(DEF_Z);
		PathPlanPoint_L.m_arr[DEF_ALPHA]=L_starP.at(DEF_ALPHA)+(L_endP.at(DEF_ALPHA)-L_starP.at(DEF_ALPHA))*t/CostTime; 
		PathPlanPoint_L.m_arr[DEF_BETA]=L_starP.at(DEF_BETA)+(L_endP.at(DEF_BETA)-L_starP.at(DEF_BETA))*t/CostTime;
		PathPlanPoint_L.m_arr[DEF_GAMMA]=L_starP.at(DEF_GAMMA)+(L_endP.at(DEF_GAMMA)-L_starP.at(DEF_GAMMA))*t/CostTime;
		
		//==calculate IK and Output to arm==//
		IKOutputToArm(PathPlanPoint_R,PathPlanPoint_L);

		do
		{
			Sleep(0);
			QueryPerformanceCounter(&nEndTime);
			//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
		}
		while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart < gCycleT);


	}

}

void MoveToInitailPoint(CStaArray &L_starP,CStaArray &R_starP)
{
	//Transfer To Robot frame
 	for(int i=0;i<3;i++)
	{
		L_starP.m_arr[i]=L_starP.at(i)+gTranFrameToRobot.at(i);
		R_starP.m_arr[i]=R_starP.at(i)+gTranFrameToRobot.at(i);	
	}

	float vel_deg_R=20;
	float vel_deg_L=20;

	//MoveToPoint_Dual(R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
	MoveToPoint(DEF_LEFT_HAND,L_starP.m_arr,vel_deg_L);
	MoveToPoint(DEF_RIGHT_HAND,R_starP.m_arr,vel_deg_R);
	
	printf("move to p0..\n");
	WaitMotionDoneDual();


#ifdef	DEF_WAIT_ENTER
	printf("In initial point. press any key to continue...\n");
	getchar();
#endif
}

void TestSewingAction()
{
	//==variable for reocrd file==//
	//open file
#ifdef	RECORD_JOINT_ANGLE
	gfileR.open("D://GetDrinkJointAngle_R.csv",ios::out|ios::trunc);
	gfileL.open("D://GetDrinkJointAngle_L.csv",ios::out|ios::trunc);
#endif

#ifdef CHECK_CARTESIAN_PATH
	gfileR.open("C://stanley//GetSewCartesian_R.csv",ios::out|ios::trunc);
	gfileL.open("C://stanley//GetSewCartesian_L.csv",ios::out|ios::trunc);

#endif

#ifdef	CHECK_JOINT_PATH
	gfileR.open("C://stanley//SewJoint_CMD_R.csv",ios::out|ios::trunc);
	gfileL.open("C://stanley//SewJoint_CMD_L.csv",ios::out|ios::trunc);
#endif


	//==static parameter==//
	const float MovOutLen=50;//���X����I������
	const float SewingLength=60;//�_����{
	const float RelMovLen=180;//�ج[����I���Z

	//==MoveToInitailPoint==//
#ifdef	MOVE_TO_INITIAL_POINT
	CStaArray R_IniP(-90,-90,0,50,0,0,-50);
	CStaArray L_IniP(-90,90,0,-90,0,0,90);
	MoveToInitailPoint(R_IniP,L_IniP);
#endif

	//�����} ��
#ifdef F446RE_GRIPPER_EN
	F446RE_FootLifter(true);

	//�k�⧨ ���⧨
	F446RE_Gripper_Hold(DEF_RIGHT_HAND,true,1000);
	F446RE_Gripper_Hold(DEF_LEFT_HAND,true,1000);

	//�����} ��
	F446RE_FootLifter(false);

	//�D�b�Ұ�
	F446RE_Spindle(true);
#endif
	//�k�⩹��X SewingLenth ���⩹��X �_�u���� SewingLenth
	CStaArray R_starP(-90,-90,0,50,0,0,-50);
	CStaArray R_endP(-90+SewingLength,-90,0,50,0,0,-50);
	CStaArray L_starP(-90,90,0,-90,0,0,90);
	CStaArray L_endP(-90+SewingLength,90,0,-90,0,0,90);

	float CostTime=3;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);
#ifdef F446RE_GRIPPER_EN	
	//�D�b����
	F446RE_Spindle(false);

	//�k�⤣�� ����}
	F446RE_Gripper_Hold(DEF_LEFT_HAND,false,800);
#endif

	//�k�⤣�� ���⩹��y���� 
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength,90,0,-90,0,0,90);
	L_endP.SetArray(-90+SewingLength,90+MovOutLen,0,-90,0,0,90);
	CostTime=3;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);

	//�k�⤣�� ���⩹��X ����I���j����(Release move length)
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength,90+MovOutLen,0,-90,0,0,90);
	L_endP.SetArray(-90+SewingLength+RelMovLen,90+MovOutLen,0,-90,0,0,90);
	CostTime=4;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);

	//�k�⤣�� ���⩹�ty����MovOutLen
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength+RelMovLen,90+MovOutLen,0,-90,0,0,90);
	L_endP.SetArray(-90+SewingLength+RelMovLen,90,0,-90,0,0,90);
	CostTime=3;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);

#ifdef F446RE_GRIPPER_EN
	//�k�⤣�� ���⧨
	F446RE_Gripper_Hold(DEF_LEFT_HAND,true,800);

	//�����}��
	F446RE_FootLifter(true);
#endif

	//�k����੹��X ������੹�tX
	R_starP.SetArray(-90+SewingLength,-90,0,50,0,0,-50);
	R_endP.SetArray(90,-90,0,50,0,0,-50);
	L_starP.SetArray(-90+SewingLength+RelMovLen,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CStaArray arc_cen(gNeedle_ini_Plate); //�����߬��w�b�[�l�W���_�l�I
	float rot_rad=0.5*DEF_PI; //����ɪ��_�l���ਤ��
	CostTime=3;
	RotateMoveTo(L_starP,L_endP,R_starP,R_endP,arc_cen,rot_rad,CostTime);

#ifdef F446RE_GRIPPER_EN
	//�����}��
	F446RE_FootLifter(false);

	//�k��} ���⤣��
	F446RE_Gripper_Hold(DEF_LEFT_HAND,false,800);
#endif

	//�k�⩹X�tY�t���X  ���⤣��1 
	R_starP.SetArray(90,-90,0,50,0,0,-50);
	R_endP.SetArray(90-MovOutLen,-90-MovOutLen,0,50,0,0,-70);
	L_starP.SetArray(-90,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CostTime=2;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);

	//�k�⩹X�t����RelMovLen  ���⤣��1 
	R_starP.SetArray(90-MovOutLen,-90-MovOutLen,0,50,0,0,-70);
	R_endP.SetArray(90-MovOutLen-RelMovLen,-90-MovOutLen,0,50,0,0,-70);
	L_starP.SetArray(-90,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CostTime=5;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);

	//�k�⩹X��Y��MovOutLen  ���⤣��1 
	R_starP.SetArray(90-MovOutLen-RelMovLen,-90-MovOutLen,0,50,0,0,-70);
	R_endP.SetArray(90-RelMovLen,-90,0,50,0,0,-70);
	L_starP.SetArray(-90,90,0,-90,0,0,90);
	L_endP.SetArray(-90,90,0,-90,0,0,90);
	CostTime=2;
	LineMoveTo(L_starP,L_endP,R_starP,R_endP,CostTime);

#ifdef F446RE_GRIPPER_EN
	//�k�⧨ ���⤣��1
	F446RE_Gripper_Hold(DEF_RIGHT_HAND,true,800);
#endif

	
#if	defined(RECORD_JOINT_ANGLE) || defined(CHECK_CARTESIAN_PATH) 
	gfileR.close();
	gfileL.close();
#endif
	
#ifdef	CHECK_JOINT_PATH
	gfileR.close();
	gfileR.close();
#endif

}

//void TestSewingActionOld()
//{
//	const int SegSize=20;
//
//	//==�U�q��O�ɶ�==//
//	float SeqItv[SegSize]={0};//Sequence  invterval
//
//	SeqItv[S_INITIAL]=0;
//	SeqItv[S_FL_UP_1]=2;//�����}��1
//	SeqItv[S_R_HOLD_L_HOLD_1]=2;//�k�⧨ ���⧨
//	SeqItv[S_FL_DOWN_1]=2;//�����}��1
//	SeqItv[S_SP_ON_1]=0.05;//�D�b�}�l�_
//	SeqItv[S_R_FX_L_FX_1]=3;//�k�⩹��X SewingLenth ���⩹��X �_�u���� SewingLenth 
//	SeqItv[S_SP_OFF_1]=2;//�D�b����
//	SeqItv[S_R_KEEP_L_REL_1]=2;//�k�⤣�� ����}1
//	SeqItv[S_R_KEEP_L_FY_1]=3;//�k�⤣�� ���⩹��y���� 
//	SeqItv[S_R_KEEP_L_FX_1]=4;//�k�⤣�� ���⩹��X ����I���j����(Release move lenth)
//	SeqItv[S_R_KEEP_L_BY_1]=3;//�k�⤣�� ���⩹�ty����MovOutLen
//
//	SeqItv[S_R_KEEP_L_HOLD_1]=2;///�k�⤣�� ���⧨
//	SeqItv[S_FL_UP_2]=2;//�����}��2
//	
//	SeqItv[S_R_FCIRX_L_BCIRX_1]=3;//�k����੹��X ������੹�tX
//	SeqItv[S_FL_DOWN_2]=2;//�����}��2
//
//	SeqItv[S_R_REL_L_HOLD_1]=2;//�k��} ���⤣��
//	SeqItv[S_R_BXY_L_KEEP_1]=2;//�k�⩹X�tY�t���X1  ���⤣��1 
//	SeqItv[S_R_BX_L_KEEP_1]=5;//�k�⩹X�t  ���⤣��1
//	SeqItv[S_R_FXY_L_KEEP_1]=2;//�k�⩹X��Y��  ���⤣��1 
//	SeqItv[S_R_HOLD_L_KEEP_1]=2;//�k�⧨ ���⤣��1
//
//
//	//==����ɶ��Эp==//
//	float Seqt[SegSize]={0};
//	float CurT=0;
//
//	for(int i=0;i<SegSize;i++) 
//	{
//		CurT=CurT+SeqItv[i];
//		Seqt[i]=CurT;
//	}
//	float TotalTime=CurT;
//	
//	
//	float Needle_RobotF[3]={350,-300,30};//�w�I�b���u���Шt��m   
//	float Needle_ini_Plate[3]={30,-30,0};//�U�w�I�b�[�lplate�y�Шt�W����l�I
//	float TranFrameToRobot[3]={0};//�Q�Ψ�Ӫ��t�ȥh�����
//
//	for(int i=0;i<3;i++) 
//	{
//		TranFrameToRobot[i]=Needle_RobotF[i]-Needle_ini_Plate[i];
//	}
//
//	float MovOutLen=50;//���X����I������
//	float SewingLength=60;//�_����{
//	float RelMovLen=180;//�ج[����I���Z
//
//	//�k���P���|
//	//ini_rotate_p_R=[-90 -90 0]+[SewingLength 0 0]+TranFrameToRobot;//��x��V�e�i140����A�}�l������ �q�[�l���Шt�����u�y�Шt
//	float ini_rotate_p_R[3]={0};
//	ini_rotate_p_R[DEF_X]=-90+SewingLength+TranFrameToRobot[DEF_X];
//	ini_rotate_p_R[DEF_Y]=-90+TranFrameToRobot[DEF_Y];
//	ini_rotate_p_R[DEF_Z]=TranFrameToRobot[DEF_Z];
//
//	float rR=sqrt(pow((ini_rotate_p_R[DEF_X]-Needle_RobotF[DEF_X]),2)+pow((ini_rotate_p_R[DEF_Y]-Needle_RobotF[DEF_Y]),2));
//	float ini_rad_R=DEF_PI+atan((ini_rotate_p_R[DEF_Y]-Needle_RobotF[DEF_Y])/(ini_rotate_p_R[DEF_X]-Needle_RobotF[DEF_X]));//����ɪ��_�l���ਤ��
//
//	//�����P���|
//	//ini_rotate_p_L=[90 90 0]+[SewingLength 0 0]+TranFrameToRobot;%��x��V�e�i140
//	float ini_rotate_p_L[3]={0};
//	ini_rotate_p_L[DEF_X]=90+SewingLength+TranFrameToRobot[DEF_X];
//	ini_rotate_p_L[DEF_Y]=90+TranFrameToRobot[DEF_Y];
//	ini_rotate_p_L[DEF_Z]=TranFrameToRobot[DEF_Z];
//
//	float rL=sqrt(pow((ini_rotate_p_L[DEF_X]-Needle_RobotF[DEF_X]),2)+pow((ini_rotate_p_L[DEF_Y]-Needle_RobotF[DEF_Y]),2));
//	float ini_rad_L=atan((ini_rotate_p_L[DEF_Y]-Needle_RobotF[DEF_Y])/(ini_rotate_p_L[DEF_X]-Needle_RobotF[DEF_X]));
//
//	float HoldLen_L[3]={180,0,0};//�������I���Z �ѮبM�w
//	float HoldLen_R[3]={180,0,0};//�k�����I���Z �ѮبM�w
//	
//
//	//==���|�I
//	//float R_p[SegSize][7]={   
//	//	{210,	-360,	0,	50,	-90,	0,	-50},	//�_�l�I
//	//	{210,	-360,	0,	50,	-90,	0,	-50},	//�k�⧨ ���⧨
//	//	{350,	-360,	0,  50, -90,	0,	-50},	//�k�⩹��X 140 ���⩹��X 140 
//	//	{350,	-360,	0,  50, -90,	0,	-50},	//�k�⤣�� ����}1
//	//	{350,	-360,	0,  50, -90,	0,	-50},	//�k�⤣�� ���⩹��X 180
//	//	{350,	-360,	0 , 50 ,-90,	0,	-50},	//�k�⤣�� ���⧨1
//	//	{390,	-360,	0,  50, -90,	0,	-50},	//�k����੹��X ������੹�tX
//	//	{390,	-360,	0,  50, -90,	0,	-50},	//�k��} ���⤣��1
//	//	{210,	-360,	0,  50, -90,	0,	-50}};  //�k�⩹X�t  ���⤣��1 
//	float R_p_robotF[SegSize][8]={   
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_GRIP_REL},//�_�l�I
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_FL_UP},//�����}��1
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_GRIP_HOLD},	//�k�⧨ ���⧨
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_FL_DOWN},	//�����}��1
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_SP_ON},	//�D�b�Ұ�
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_LINE},	//�k�⩹��X SewingLenth ���⩹��X �_�u���� SewingLenth 
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_SP_OFF},//�D�b����
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_GRIP_NOACT},	//�k�⤣�� ����}1
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_LINE},	//�k�⤣�� ���⩹��y���� 
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_LINE},	//�k�⤣�� ���⩹��X ����I���j����(Release move lenth)
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_LINE},   //�k�⤣�� ���⩹�ty����MovOutLen
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_GRIP_NOACT},///�k�⤣�� ���⧨
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_FL_UP},//�����}��2
//	{90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_ARC},	//�k����੹��X ������੹�tX
//	{90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_FL_DOWN},//�����}��2
//	{90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-50, DEF_TYPE_GRIP_REL},	//�k��} ���⤣��1
//	{90-MovOutLen+TranFrameToRobot[DEF_X],	-90-MovOutLen+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-70, DEF_TYPE_LINE},//�k�⩹X�tY�t���X  ���⤣��1
//	{-90-MovOutLen+TranFrameToRobot[DEF_X],	-90-MovOutLen+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-70, DEF_TYPE_LINE},//�k�⩹X�t  ���⤣��1 
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-70, DEF_TYPE_LINE},//�k�⩹X��Y��  ���⤣��1
//	{-90+TranFrameToRobot[DEF_X],	-90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	50,	0, 0,-70, DEF_TYPE_GRIP_HOLD}}; //�k�⧨ ���⤣��1
//	
//	
//	//Mat V_r_end=(Mat_<float>(9,9) <<Pend[0]-base[0],//x 
//	//								Pend[1]-base[1],//y
//	//								Pend[2]-base[2]);//z
//		
//	//float L_p[SegSize][8]={  
//	//	{210,-180,0,-90,90,0,90},	//�_�l�I
//	//	{210,-180,0,-90,90,0,90},	//�k�⧨ ���⧨
//	//	{350,-180,0,-70,90,0,90},	//�k�⩹��X 140 ���⩹��X 140 
//	//	{350,-180,0,-70,90,0,90},	//�k�⤣�� ����}1
//	//	{530,-180,0,-50,90,0,90},	//�k�⤣�� ���⩹��X 180
//	//	{530,-180,0,-50,90,0,90},	//�k�⤣�� ���⧨1
//	//	{210,-180,0,-90,90,0,90},	//�k����੹��X ������੹�tX
//	//	{210,-180,0,-90,90,0,90},	//�k��} ���⤣��1
//	//	{210,-180,0,-90,90,0,90}};	//�k�⩹X�t  ���⤣��1 
//	
//	float L_p_robotF[SegSize][8]={   
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_GRIP_REL},//�_�l�I
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y], 0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_FL_UP},//�����}��1
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_GRIP_HOLD},//�k�⧨ ���⧨
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_FL_DOWN},//�����} ��
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_SP_ON},//�D�b�Ұ�
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_LINE},//�k�⩹��X SewingLenth ���⩹��X �_�u���� SewingLenth 
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_SP_OFF},//�D�b����
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -90, 0, 0,90, DEF_TYPE_GRIP_REL},//�k�⤣�� ����}1
//	{-90+SewingLength+TranFrameToRobot[DEF_X],	90+MovOutLen+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90, 0, 0,90, DEF_TYPE_LINE},//�k�⤣�� ���⩹��y���� 
//	{-90+SewingLength+RelMovLen+TranFrameToRobot[DEF_X],	90+MovOutLen+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-60,	0, 0,90, DEF_TYPE_LINE},	//�k�⤣�� ���⩹��X ����I���j����(Release move lenth)
//	{-90+SewingLength+RelMovLen+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -60, 0, 0,90, DEF_TYPE_LINE},   //�k�⤣�� ���⩹�ty����MovOutLen
//	{-90+SewingLength+RelMovLen+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -60, 0, 0,90, DEF_TYPE_GRIP_HOLD},///�k�⤣�� ���⧨
//	{-90+SewingLength+RelMovLen+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z], -60, 0, 0,90, DEF_TYPE_FL_UP},//�����} ��
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_ARC},	//�k����੹��X ������੹�tX
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_FL_DOWN},//�����} ��
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_GRIP_NOACT},	//�k��} ���⤣��1
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_LINE},//�k�⩹Y�t  ���⤣��1 
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_LINE},//�k�⩹X�t  ���⤣��1 
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_LINE},//�k�⩹Y��  ���⤣��1 
//	{-90+TranFrameToRobot[DEF_X],	90+TranFrameToRobot[DEF_Y],	0+TranFrameToRobot[DEF_Z],	-90,	0, 0,90, DEF_TYPE_GRIP_NOACT}};//�k�⧨ ���⤣��1
//
//
//	
//
//
//	//==�y�{�ݥ��ܼ�
//	//float CycleT=0.1f;
//#ifndef RECORD_JOINT_ANGLE
//	float CycleT=0.01f; //��Ūfeedback�� ��matrix �j��22m  opencv mat �j��2.5ms �]����10ms
//	//float CycleT=0.5f;
//#else
//	float CycleT=0.1f; //��Ūfeedback�ɻݭn 22+17+17=56ms 19+16+16=51ms
//#endif
//
//	//float CycleT=0.040f; //�氦��S������Ū��
//	//float CycleT=0.03f; //��Ūfeedback�� �j��22m ���Lcycle time�]�j�@�I���|���b���Iover load���p�Acycltime���ڤj�A��ܷ|�b��F�e�U�s�R�O
//
//	float Itv=0; //interval of the segment
//	float t=0; //t of each segment (0~interval)
//	
//	
//
//	//==variable for reocrd file==//
//	//open file
//#ifdef	RECORD_JOINT_ANGLE
//	fstream fileR;
//	fstream fileL;
//	fileR.open("D://GetDrinkJointAngle_R.csv",ios::out|ios::trunc);
//	fileL.open("D://GetDrinkJointAngle_L.csv",ios::out|ios::trunc);
//#endif
//
//#ifdef CHECK_CARTESIAN_PATH
//	fstream fileR;
//	fstream fileL;
//	fileR.open("D://GetSewCartesian_R.csv",ios::out|ios::trunc);
//	fileL.open("D://GetSewCartesian_L.csv",ios::out|ios::trunc);
//
//#endif
//
//#ifdef	CHECK_JOINT_PATH
//	fileR.open("C://stanley//SewJoint_CMD_R.csv",ios::out|ios::trunc);
//	fileR.open("C://stanley//SewJoint_CMD_L.csv",ios::out|ios::trunc);
//#endif
//	
//	
//	//==record para==//
//	float pos_deg_R[MAX_AXIS_NUM]={0};
//	float pos_deg_L[MAX_AXIS_NUM]={0};
//	float pos_deg_last_ok_R[MAX_AXIS_NUM]={0};
//	float pos_deg_last_ok_L[MAX_AXIS_NUM]={0};
//	int n=0;
//	int rt=0;
//	
//	//==Robotic arm pose==//
//	const int POINT_DIMENTION=7; //[x,y,z,alpha,beta,gamma,redant_alppha]
//	float PathPlanPoint_R[POINT_DIMENTION]={0};
//	float PathPlanPoint_L[POINT_DIMENTION]={0};
//	float vel_deg_R=20;
//	float vel_deg_L=20;
//
//	//=========================//
//	//==move to initial point==//
//	//=========================//
//#ifdef	MOVE_TO_INITIAL_POINT
//	
//	//MoveToPoint_Dual(R_p[0],pose_deg_R,Rednt_alpha_R,vel_deg_R,L_p[0],pose_deg_L,Rednt_alpha_L,vel_deg_L);  //20ms
//	MoveToPoint(DEF_RIGHT_HAND,R_p_robotF[0],vel_deg_R);
//	MoveToPoint(DEF_LEFT_HAND,L_p_robotF[0],vel_deg_L);
//	printf("move to p0..\n");
//	WaitMotionDoneDual();
//#endif
//
//#ifdef	DEF_WAIT_ENTER
//	printf("In initial point. press any key to continue...\n");
//	getchar();
//#endif
//	//=========================//
//	//==Sewing process start ==//
//	//=========================//
//	LARGE_INTEGER nFreq;
//	LARGE_INTEGER nBeginTime;
//	LARGE_INTEGER nEndTime;
//
//	QueryPerformanceFrequency(&nFreq);
//	
//	float abst=0.0;
//	int index=1;
//	int GripperAlreadyAct=0;// already send command to gripper 
//	int IOAlreadyAct=0;
//
//	for(abst=0.0;abst<=TotalTime;abst+=CycleT)
//	{
//		QueryPerformanceCounter(&nBeginTime); //Record cycle start time
//
//		if(abst>Seqt[index] && (index<SegSize))
//		{
//			index=index+1;
//			GripperAlreadyAct=0;//reset �W�@�q�������X�� 
//			IOAlreadyAct=0;
//		}
//		//if(abst>TotalTime)
//		//	abst=TotalTime;
//
//		Itv=SeqItv[index];
//		t=abst-Seqt[index-1];
//
//		int Rtype=(int)R_p_robotF[index][DEF_ACT_TYPE];
//		int Ltype=(int)L_p_robotF[index][DEF_ACT_TYPE];
//		
//		if(Rtype==DEF_TYPE_LINE)
//		{
//			for(int f=0;f<POINT_DIMENTION;f++) 
//			{
//				PathPlanPoint_R[f]=R_p_robotF[index-1][f]+(R_p_robotF[index][f]-R_p_robotF[index-1][f])*t/Itv;//�W�@�����_�l�I�}�l��
//				PathPlanPoint_L[f]=L_p_robotF[index-1][f]+(L_p_robotF[index][f]-L_p_robotF[index-1][f])*t/Itv;    
//			}
//		}
//		else if(Rtype==DEF_TYPE_ARC)
//		{
//			for(int f=0;f<POINT_DIMENTION;f++) 
//			{			
//				PathPlanPoint_R[DEF_X]=Needle_RobotF[DEF_X]+(float)(rR*(cos(0.5*DEF_PI*t/Itv + ini_rad_R))); 
//				PathPlanPoint_R[DEF_Y]=Needle_RobotF[DEF_Y]+(float)(rR*(sin(0.5*DEF_PI*t/Itv + ini_rad_R))); 
//				PathPlanPoint_R[DEF_Z]=Needle_RobotF[DEF_Z];
//				PathPlanPoint_R[DEF_ALPHA]=R_p_robotF[index-1][DEF_ALPHA]+(R_p_robotF[index][DEF_ALPHA]-R_p_robotF[index-1][DEF_ALPHA])*t/Itv; 
//				PathPlanPoint_R[DEF_BETA]=R_p_robotF[index-1][DEF_BETA]+(R_p_robotF[index][DEF_BETA]-R_p_robotF[index-1][DEF_BETA])*t/Itv;
//				PathPlanPoint_R[DEF_GAMMA]=R_p_robotF[index-1][DEF_GAMMA]+(R_p_robotF[index][DEF_GAMMA]-R_p_robotF[index-1][DEF_GAMMA])*t/Itv;
//
//				PathPlanPoint_L[DEF_X]=Needle_RobotF[DEF_X]+(float)(rL*(cos(0.5*DEF_PI*t/Itv + ini_rad_L))); 
//				PathPlanPoint_L[DEF_Y]=Needle_RobotF[DEF_Y]+(float)(rL*(sin(0.5*DEF_PI*t/Itv + ini_rad_L))); 
//				PathPlanPoint_L[DEF_ALPHA]=L_p_robotF[index-1][DEF_ALPHA]+(L_p_robotF[index][DEF_ALPHA]-L_p_robotF[index-1][DEF_ALPHA])*t/Itv; 
//				PathPlanPoint_L[DEF_BETA]=L_p_robotF[index-1][DEF_BETA]+(L_p_robotF[index][DEF_BETA]-L_p_robotF[index-1][DEF_BETA])*t/Itv;
//				PathPlanPoint_L[DEF_GAMMA]=L_p_robotF[index-1][DEF_GAMMA]+(L_p_robotF[index][DEF_GAMMA]-L_p_robotF[index-1][DEF_GAMMA])*t/Itv;
//			}
//		}
//        else if((Rtype==DEF_TYPE_GRIP_HOLD) || (Ltype==DEF_TYPE_GRIP_HOLD) || (Rtype==DEF_TYPE_GRIP_REL) || (Ltype==DEF_TYPE_GRIP_REL))
//		{
//			for(int f=0;f<POINT_DIMENTION;f++) 
//			{
//				PathPlanPoint_R[f]=R_p_robotF[index][f]; 
//				PathPlanPoint_L[f]=L_p_robotF[index][f];
//			}
//
//			if(GripperAlreadyAct==0)
//			{
//				if(Ltype==DEF_TYPE_GRIP_HOLD)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Gripper_Hold(DEF_LEFT_HAND,true,1000);
//					Sleep(1500);//�s�򱵵ۤU�ɻݭndelay 
//#endif				
//					printf("left hold\n");
//				}
//				if(Ltype==DEF_TYPE_GRIP_REL)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Gripper_Hold(DEF_LEFT_HAND,false,1000);
//					Sleep(1500);//ms 
//#endif
//					printf("left release\n");
//				}
//				if(Rtype==DEF_TYPE_GRIP_HOLD)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Gripper_Hold(DEF_RIGHT_HAND,true,1000);
//					Sleep(1500);
//#endif				
//					printf("right hold\n");
//				}
//				if(Rtype==DEF_TYPE_GRIP_REL)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Gripper_Hold(DEF_RIGHT_HAND,false,1000);
//					Sleep(1500);//ms
//#endif
//					printf("right release\n");
//				}
//				GripperAlreadyAct=1;
//			}
//		}
//		else if((Rtype==DEF_TYPE_SP_ON) || (Rtype==DEF_TYPE_SP_OFF) || (Rtype==DEF_TYPE_FL_UP) || (Rtype==DEF_TYPE_FL_DOWN) || (Rtype==DEF_TYPE_TRIM_ON) || (Rtype==DEF_TYPE_TRIM_OFF))
//		{
//			for(int f=0;f<POINT_DIMENTION;f++) 
//			{
//				PathPlanPoint_R[f]=R_p_robotF[index][f]; 
//				PathPlanPoint_L[f]=L_p_robotF[index][f];
//			}
//
//			if(IOAlreadyAct==0)
//			{
//				if(Rtype==DEF_TYPE_SP_ON)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Spindle(true);
//#endif
//					printf("spindle on\n");
//				}
//				if(Rtype==DEF_TYPE_SP_OFF)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Spindle(false);
//#endif
//					printf("spindle off\n");
//				}
//				if(Rtype==DEF_TYPE_FL_UP)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_FootLifter(true);
//#endif
//					printf("footlifter up\n");
//				}
//				if(Rtype==DEF_TYPE_FL_DOWN)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_FootLifter(false);
//#endif
//					printf("footlifter down\n");
//				}
//				if(Rtype==DEF_TYPE_TRIM_ON)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Trimmer(true);
//#endif
//					printf("trimmer on\n");
//				}            
//				if(Rtype==DEF_TYPE_TRIM_OFF)
//				{
//#ifdef F446RE_GRIPPER_EN
//					F446RE_Trimmer(false);
//#endif
//					printf("trimmer off\n");
//				}
//			}
//			IOAlreadyAct=1;
//		}
//		vel_deg_R=30;
//		vel_deg_L=30;
//		//vel_deg_R=0;
//		//vel_deg_L=0;
//		
//
//#ifdef MOVETOPOINT_DUAL
//		//PathPlanPoint_R[0]=410;//test
//		//PathPlanPoint_R[1]=-360;
//		//PathPlanPoint_R[2]=15;
//		//PathPlanPoint_R[3]=50;
//		//PathPlanPoint_R[4]=0;
//		//PathPlanPoint_R[5]=0;
//		//PathPlanPoint_R[6]=-50;
//		MoveToPoint_Dual(PathPlanPoint_R,vel_deg_R,PathPlanPoint_L,vel_deg_L);  //�ϥέ쥻matrix�j��20ms    �אּopencv matri�ᥭ��2.5ms �]��cycle time�Q��10ms  
//#endif
//		printf("Pend_R=[%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f],Pend_L=[%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f]\n",PathPlanPoint_R[DEF_X],PathPlanPoint_R[DEF_Y],PathPlanPoint_R[DEF_Z],PathPlanPoint_R[DEF_ALPHA],PathPlanPoint_R[DEF_BETA],PathPlanPoint_R[DEF_GAMMA],PathPlanPoint_R.at(DEF_REDNT_ALPHA),PathPlanPoint_L[DEF_X],PathPlanPoint_L[DEF_Y],PathPlanPoint_L[DEF_Z],PathPlanPoint_L[DEF_ALPHA],PathPlanPoint_L[DEF_BETA],PathPlanPoint_L[DEF_GAMMA],PathPlanPoint_L.at(DEF_REDNT_ALPHA));
//
//		
//		//==�T�{�y���I==//
//#ifdef CHECK_CARTESIAN_PATH
//		char buffer[100];
//		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",abst,PathPlanPoint_R[DEF_X],PathPlanPoint_R[DEF_Y],PathPlanPoint_R[DEF_Z]);
//		fileR.write(buffer,n);
//
//		n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f\n",abst,PathPlanPoint_L[DEF_X],PathPlanPoint_L[DEF_Y],PathPlanPoint_L[DEF_Z]);
//		fileL.write(buffer,n);
//#endif 
//
//#ifdef RECORD_JOINT_ANGLE
//		////==Read right hand
//		rt=Read_pos(DEF_RIGHT_HAND,pos_deg_R,DEF_UNIT_DEG);
//
//		if(rt==0)
//		{
//			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
//			//{
//			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_R[i]);
//			//}
//			printf("\n");
//
//			
//			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_R[Index_AXIS1],pos_deg_R[Index_AXIS2],pos_deg_R[Index_AXIS3],pos_deg_R[Index_AXIS4],pos_deg_R[Index_AXIS5],pos_deg_R[Index_AXIS6],pos_deg_R[Index_AXIS7]);
//			fileR.write(buffer,n);
//			
//			memcpy(pos_deg_last_ok_R,pos_deg_R,sizeof(pos_deg_last_ok_R));
//		}
//		else //Ū�����ѮɡA���e�@���Ӹ�
//		{
//			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
//			//{
//			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_R[i]);
//			//}
//			printf("\n");
//
//			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_last_ok_R[Index_AXIS1],pos_deg_last_ok_R[Index_AXIS2],pos_deg_last_ok_R[Index_AXIS3],pos_deg_last_ok_R[Index_AXIS4],pos_deg_last_ok_R[Index_AXIS5],pos_deg_last_ok_R[Index_AXIS6],pos_deg_last_ok_R[Index_AXIS7]);
//			fileR.write(buffer,n);
//		}
//
//		//==Read left hand
//		rt=Read_pos(DEF_LEFT_HAND,pos_deg_L,DEF_UNIT_DEG);
//
//		if(rt==0)
//		{
//			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
//			//{
//			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_L[i]);
//			//}
//			printf("\n");
//
//			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_L[Index_AXIS1],pos_deg_L[Index_AXIS2],pos_deg_L[Index_AXIS3],pos_deg_L[Index_AXIS4],pos_deg_L[Index_AXIS5],pos_deg_L[Index_AXIS6],pos_deg_L[Index_AXIS7]);
//			fileL.write(buffer,n);
//			
//			memcpy(pos_deg_last_ok_L,pos_deg_L,sizeof(pos_deg_last_ok_L));
//		}
//		else //Ū�����ѮɡA���e�@���Ӹ�
//		{
//			//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
//			//{
//			//	printf("f%d:%3.0f, ",gMapAxisNO[i],pos_deg_last_ok_L[i]);
//			//}
//			printf("\n");
//
//			n=sprintf_s(buffer,sizeof(buffer),"%4.3f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",abst,pos_deg_last_ok_L[Index_AXIS1],pos_deg_last_ok_L[Index_AXIS2],pos_deg_last_ok_L[Index_AXIS3],pos_deg_last_ok_L[Index_AXIS4],pos_deg_last_ok_L[Index_AXIS5],pos_deg_last_ok_L[Index_AXIS6],pos_deg_last_ok_L[Index_AXIS7]);
//			fileL.write(buffer,n);
//		}
//#endif
//
//
//		do
//		{
//			Sleep(0);
//			QueryPerformanceCounter(&nEndTime);
//			//printf("%f\n",(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart);
//		}
//		while((double)(nEndTime.QuadPart-nBeginTime.QuadPart)/(double)nFreq.QuadPart < CycleT);
//	}	
//
//#if	defined(RECORD_JOINT_ANGLE) || defined(CHECK_CARTESIAN_PATH) 
//	fileR.close();
//	fileL.close();
//#endif
//	
//#ifdef	CHECK_JOINT_PATH
//	fileR.close();
//	fileR.close();
//#endif
//}

int TestMoveToSewingHome_Dual()
{
	//float theta_R[7]={0.02,-1.02,-0.06,1.48,0.19,0.34,1.18};
	float theta_R[7]={0.08,-0.72,0.36,1.48,0.78,0.05,1.22};
	float theta_L[7]={-0.97,-0.16,1.39,1.07,0.22,-0.35,-0.91};

	//output to motor
	unsigned short int velocity_R[MAX_AXIS_NUM]={6,3,5,2,4,4,4};
	unsigned short int velocity_L[MAX_AXIS_NUM]={4,4,4,4,4,4,4};
	
	Output_to_Dynamixel_Dual(theta_R,velocity_R,theta_L,velocity_L); 

	WaitMotionDoneDual();

	Torque_Disable();
	printf("Torque_Disable\n");	
	return 0;

}
int Torque_Disable()
{
		//================================//
	//==output to motor by syncpage===//
	//===============================//
	unsigned short int SyncPage[28]=
	{ 
		ID_RAXIS1,(unsigned short int)0, //ID,torque enable
		ID_RAXIS2,(unsigned short int)0, 
		ID_RAXIS3,(unsigned short int)0, 
		ID_RAXIS4,(unsigned short int)0, 
		ID_RAXIS5,(unsigned short int)0, 
		ID_RAXIS6,(unsigned short int)0, 
		ID_RAXIS7,(unsigned short int)0, 

		ID_LAXIS1,(unsigned short int)0, 
		ID_LAXIS2,(unsigned short int)0, 
		ID_LAXIS3,(unsigned short int)0, 
		ID_LAXIS4,(unsigned short int)0, 
		ID_LAXIS5,(unsigned short int)0, 
		ID_LAXIS6,(unsigned short int)0, 
		ID_LAXIS7,(unsigned short int)0, 
	};
	
#if (DEBUG)
	//for(i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	//	printf("syncwrite AXIS%d pos=%d velocity=%d\n",gMapAxisNO[i],Ang_pulse[i],velocity[i]);
#endif
	
	syncWrite_x86(TORQUE_ENABLE,1,SyncPage,28);//byte syncWrite(byte start_addr, byte num_of_data, int *param, int array_length);

	return 0;
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
//Matrix R_z1x2y3(float alpha,float beta,float gamma)
//{
//	Matrix ans(3,3);
//	ans << cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma)	<< -cos(beta)*sin(alpha)	<< cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta)
//        << cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)	<< cos(alpha)*cos(beta)		<< sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta)
//        << -cos(beta)*sin(gamma)									<< sin(beta)				<< cos(beta)*cos(gamma);
//
//	return ans;
//
//}

//�ϥ�CV Mat
Mat R_z1x2y3(float alpha,float beta,float gamma)
{
	Mat ans=(Mat_<float>(3,3) <<	cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma),	-cos(beta)*sin(alpha)	,	cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta),  //0.06ms
									cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma),	cos(alpha)*cos(beta)	,	sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta),
									-cos(beta)*sin(gamma),sin(beta),cos(beta)*cos(gamma));
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
//opencv matrix
Mat Rogridues(float theta,Mat V_A)
{
	float cs = cos( theta );
    float sn = sin( theta );

	float* pV_A=V_A.ptr<float>(0);
	


	Mat R_a=(Mat_<float>(4,4) <<	cs + pow(pV_A[0],2)*(1-cs)											,	pV_A[0]*pV_A[1]*(1-cs)-pV_A[2]*sn						,	pV_A[0]*pV_A[2]*(1-cs)+pV_A[1]*sn				,	0,
									pV_A[0]*pV_A[1]*(1-cs)+pV_A[2]*sn									,	cos(theta)+pow(pV_A[1],2)*(1-cs)						,	pV_A[1]*pV_A[2]*(1-cs)-pV_A[0]*sn				,	0,		
									pV_A[0]*pV_A[2]*(1-cs)-pV_A[1]*sn									,	pV_A[1]*pV_A[2]*(1-cs)+pV_A[0]*sn						,	cs+pow(pV_A[2],2)*(1-cs)						,	0,
									0																	,	0														,	0												,	1);

	return R_a;
}


Mat RotX( float radians )
{
    float cs = cos( radians );
    float sn = sin( radians );

    Mat rotate=(Mat_<float>(4,4) << 1 , 0  ,  0  , 0,
									0 , cs , -sn , 0,
									0 , sn ,  cs , 0,
									0 , 0  ,  0  , 1);

    return rotate;

}

Mat RotY( float radians )
{
    float cs = cos( radians );
    float sn = sin( radians );

    Mat rotate=(Mat_<float>(4,4) <<	cs		, 0   ,  sn  , 0,
									0		, 1   ,  0   , 0,
									-sn		, 0   ,  cs  , 0,
									0		, 0   ,  0   , 1);

    return rotate;
}


//int IK_7DOF_nonFB(const float l1,const float l2,const float l3,const float x_base,const float y_base,const float z_base,const float x_end,const float y_end,const float z_end,const float alpha,const float beta,const float gamma,const float Rednt_alpha,float* theta)
//{
//	int i=0;
//	
//	//Out put initial to zero
//	for(i=Index_AXIS1;i<=Index_AXIS7;i++)
//	{
//		theta[i]=0;
//	}
//
//	Matrix R(3,3);
//	R=R_z1x2y3(alpha,beta,gamma);
//
//	Matrix V_H_hat_x(3,1);
//	V_H_hat_x=Matrix::ExportCol(R,1);//���X�کԨ��ഫ������x�}�A���X��1�欰X�b�����V�q
//	V_H_hat_x*=1/norm(V_H_hat_x);
//	
//	Matrix V_H_hat_y(3,1);
//	V_H_hat_y=Matrix::ExportCol(R,2);//���X�کԨ��ഫ������x�}�A���X��2�欰Y�b�����V�q
//	V_H_hat_y*=1/norm(V_H_hat_y);
//	
//
//	Matrix V_r_end(3,1);
//	V_r_end	<<x_end-x_base
//			<<y_end-y_base
//			<<z_end-z_base;
//
//
//	Matrix V_r_h(3,1);
//	V_r_h=V_H_hat_x*L3;
//
//	Matrix V_r_wst(3,1);
//	V_r_wst=V_r_end-V_r_h;	
//
//	//theat 4
//	theta[Index_AXIS4]=-(float)(M_PI-acos((pow(l1,2)+pow(l2,2)-pow(norm(V_r_wst),2))/(2*l1*l2)));
//
//
//	Matrix V_r_m(3,1);
//	V_r_m=(pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;
//
//
//
//	//Redundant circle �b�|R
//	float Rednt_cir_R = pow(l1,2)- pow( (pow(l1,2)-pow(l2,2)+pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2);
//	Rednt_cir_R=sqrt(Rednt_cir_R);
//
//
//	//�ꤤ���I��Elbow�V�q V_r_u
//	Matrix V_shz(3,1);
//	V_shz	<<0
//			<<0
//			<<1;
//
//	Matrix V_alpha_hat(3,1);//V_alpha_hat=cross(V_r_wst,V_shz)/norm(cross(V_r_wst,V_shz));
//	Matrix temp_cross(3,1);
//	temp_cross=MatrixMath::cross(V_r_wst,V_shz); //���~
//	V_alpha_hat=temp_cross*(1/norm(temp_cross));
//
//	Matrix V_beta_hat(3,1);//V_beta_hat=cross(V_r_wst,V_alpha_hat)/norm(cross(V_r_wst,V_alpha_hat));
//	temp_cross=MatrixMath::cross(V_r_wst,V_alpha_hat);
//	V_beta_hat=temp_cross*(1/norm(temp_cross));
//
//
//
//	Matrix temp(4,4);//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha����V�M�פ�W����V�ʬۤ�
//	Matrix V_r_wst_unit =V_r_wst*(1/norm(V_r_wst));
//	Matrix V_temp3x1(3,1);//�ݭn�g�@�ӥi�H��1�����
//	Matrix V_temp4x1(4,1);
//	V_temp3x1=V_beta_hat*Rednt_cir_R;
//	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1
//
//	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;
//
//
//	Matrix V_R_u(3,1);
//	V_R_u.Vec_export_3_row(temp);
//	
//
//	Matrix V_r_u(3,1);
//	V_r_u=V_r_m+V_R_u;
//
//	theta[Index_AXIS1]=atan2(-V_r_u(1,1),-V_r_u(3,1));//theta(1)=atan2(-V_r_u(1),-V_r_u(3));
//
//
//	if (theta[Index_AXIS1] !=0) 
//		theta[Index_AXIS2]=atan2(V_r_u(2,1),-V_r_u(1,1)/sin(theta[Index_AXIS1]));
//	else
//		theta[Index_AXIS2]=atan2(-V_r_u(2,1),V_r_u(3,1));
//	
//
//
//	//theat 3
//	//theta(3)=atan2( sin(theta(2))*sin(theta(1))*V_r_wst(1)+cos(theta(2))*V_r_wst(2)+sin(theta(2))*cos(theta(1))*V_r_wst(3),cos(theta(1))*V_r_wst(1)-sin(theta(1))*V_r_wst(3));
//	theta[Index_AXIS3]=atan2( sin(theta[Index_AXIS2])*sin(theta[Index_AXIS1])*V_r_wst(1,1)+cos(theta[Index_AXIS2])*V_r_wst(2,1)+sin(theta[Index_AXIS2])*cos(theta[Index_AXIS1])*V_r_wst(3,1),cos(theta[Index_AXIS1])*V_r_wst(1,1)-sin(theta[Index_AXIS1])*V_r_wst(3,1));
//
//
//
//	//theat 5
//	Matrix V_r_f(3,1);
//	V_r_f=V_r_wst-V_r_u;
//
//	Matrix V_Axis6(3,1);
//	V_Axis6=MatrixMath::cross(V_H_hat_y,-V_r_f)*(1/norm(MatrixMath::cross(V_H_hat_y,-V_r_f)));//V_Axis6=cross(V_H_hat_y,-V_r_f)/norm(cross(V_H_hat_y,-V_r_f));
//
//	Matrix V_r_wst_u(3,1);
//	V_r_wst_u=V_r_wst+V_Axis6;
//
//	Matrix A1_4(4,4);
//	A1_4=MatrixMath::RotY(theta[Index_AXIS1])*MatrixMath::RotX(theta[Index_AXIS2])*MatrixMath::RotZ(theta[Index_AXIS3])*MatrixMath::Tz(-l1)*MatrixMath::RotY(theta[Index_AXIS4]);//A1_4=Ry(theta(1))*Rx(theta(2))*Rz(theta(3))*Tz(-L1)*Ry(theta(4));
//	
//
//	Matrix V_temp_f(4,1);
//	Matrix V_r_wst_u_4x1(4,1);
//	V_r_wst_u_4x1.Vec_ext_1_row(V_r_wst_u,1);
//	
//
//	V_temp_f=MatrixMath::Inv(A1_4)*V_r_wst_u_4x1;//V_temp_f=inv(A1_4)*[V_r_wst_u;1]; //(3.31) �o�ӬO�ɤ@�C1�W�h���N��,need fix
//	theta[Index_AXIS5]=atan2(V_temp_f(2,1),V_temp_f(1,1));//theta(5)=atan2(V_temp_f(2),V_temp_f(1));
//
//	
//	//theat 6
//	Matrix V_r_wst_r(3,1);
//	V_r_wst_r=V_r_wst+V_H_hat_y;
//
//	Matrix A1_5(4,4);
//	A1_5=A1_4*MatrixMath::RotZ(theta[Index_AXIS5])*MatrixMath::Tz(-l2);//A1_5=A1_4*Rz(theta(5))*Tz(-L2);
//	
//	Matrix V_temp_g(4,4);
//	Matrix V_r_wst_r_4x1(4,1);
//	V_r_wst_r_4x1.Vec_ext_1_row(V_r_wst_r,1);
//	
//	V_temp_g=MatrixMath::Inv(A1_5)*V_r_wst_r_4x1; //V_temp_g=inv(A1_5)*[V_r_wst_r;1]; //(3.38)  �o�ӬO�ɤ@�C1�W�h���N��,need fix
//	
//	theta[Index_AXIS6]=atan2(V_temp_g(3,1),V_temp_g(2,1));
//
//
//	//theat 7
//	Matrix V_r_wst_f(3,1);
//	V_r_wst_f=V_r_wst+V_H_hat_x;
//
//	Matrix A1_6(4,4);
//	A1_6=A1_5*MatrixMath::RotX(theta[Index_AXIS6]);
//
//	Matrix V_temp_h(3,1);
//	Matrix V_r_wst_f_4x1(4,1);
//	V_r_wst_f_4x1.Vec_ext_1_row(V_r_wst_f,1);
//	
//	V_temp_h=MatrixMath::Inv(A1_6)*V_r_wst_f_4x1; //V_temp_h=inv(A1_6)*[V_r_wst_f;1]; 
//	
//	theta[Index_AXIS7]=atan2(-V_temp_h(1,1),-V_temp_h(3,1));//theta(7)=atan2(-V_temp_h(1),-V_temp_h(3));
//
//
//	return 0;
//}


//�ĤC�b��roll�b
//�ثe���կx�θ��|�����j���ݭn10.8ms
/*
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

	LARGE_INTEGER nFreq;

	LARGE_INTEGER nBeginTime;

	LARGE_INTEGER nEndTime;

	double time;
	double delaytime=1;
	QueryPerformanceFrequency(&nFreq);
	QueryPerformanceCounter(&nBeginTime); 
	V_temp4x1.Vec_ext_1_row(V_temp3x1,1); //3x1 extend to 4x1  0.23MS

	QueryPerformanceCounter(&nEndTime);
	time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart;
	


	
	temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1; //7.4ms

	

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
	
	//����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}

	//Vn_u_f �M V_n_yrot12���k�V�q   �P V_ru_l1�P��V theta(3)�ݭn�[�t��
	if ( norm(Vn_nuf_nyrot12 - V_ru_l1*(1/norm(V_ru_l1))) < DEF_NORM_VERY_SMALL )
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
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL) 
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
	if (norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf) < DEF_NORM_VERY_SMALL)
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

	if (norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_NORM_VERY_SMALL)
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
	tempfloat=MatrixMath::dot(V_x_rot1to6,V_H_hat_z)*(1/norm(V_x_rot1to6))*(1/norm(V_H_hat_z));// theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}

	if (norm(Vn_xrot1to6_VHhatz - V_H_hat_x) <  DEF_NORM_VERY_SMALL)
        theta[Index_AXIS7]=acos(tempfloat);
    else
        theta[Index_AXIS7]=-acos(tempfloat);  
	
	
	// ==���k���1�b��V�ۤ�== //
    if (RLHand == DEF_LEFT_HAND) //����M�k��Ĥ@�b��V�ۤ�
        theta[Index_AXIS1]=-theta[Index_AXIS1];
    
	//==output degree==//
	memcpy(out_theta,theta,sizeof(theta));

	return 0;
}
*/
//#define SHOW_MATRIX
//�ϥ�opencv matrix�p��  �Ĥ@������غcMat �ݭn50ms �U�������ɶ��i�Y��1.5ms
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
	

    //== �D�XH_hat_x ==//
    Mat R=R_z1x2y3(PoseAngle[0],PoseAngle[1],PoseAngle[2]); //alpha,beta,gamma
#ifdef SHOW_MATRIX
	cout<< "R="<<endl<<""<<R<<endl<<endl;
#endif
  
	Mat V_H_hat_x=R.col(0);  //���X�کԨ��ഫ������x�}�A���X��1�欰X�b�����V�q//0.016ms

	
	V_H_hat_x = V_H_hat_x/norm(V_H_hat_x,NORM_L2);


#ifdef SHOW_MATRIX
	cout<< "V_H_hat_x="<<endl<<""<<V_H_hat_x<<endl<<endl;
#endif

    //==H_hat_x==//
	Mat V_H_hat_z=R.col(2); //���X�کԨ��ഫ������x�}�A���X��3�欰Z�b�����V�q
	V_H_hat_z = V_H_hat_z/norm(V_H_hat_z,NORM_L2);

	Mat V_r_end=(Mat_<float>(3,1) <<Pend[0]-base[0],//x 
									Pend[1]-base[1],//y
									Pend[2]-base[2]);//z
	
   
	Mat V_r_h=V_H_hat_x*linkL[5];//L5


	Mat V_r_wst=V_r_end-V_r_h;
  
	// ==Axis4== //
    float ru_norm=sqrt(pow(linkL[1],2)+pow(linkL[2],2)); //L�����������
    float rf_norm=sqrt(pow(linkL[3],2)+pow(linkL[4],2));
	
    float theta_tmp=(float)acos((pow(ru_norm,2) + pow(rf_norm,2)- pow(norm(V_r_wst),2)) / (2*ru_norm*rf_norm));
    theta[Index_AXIS4]=(float)(2*M_PI)-atan2(linkL[1],linkL[2])-atan2(linkL[4],linkL[3])-theta_tmp;

    // ==AXIS1 2== //
	Mat V_r_m=(pow(ru_norm,2)-pow(rf_norm,2)+pow(norm(V_r_wst),2))/(2*pow(norm(V_r_wst),2))*V_r_wst;

	//Redundant circle �b�|R
	float Rednt_cir_R =(float)(pow(ru_norm,2)- pow((pow(ru_norm,2)-pow(rf_norm,2) + pow(norm(V_r_wst),2))/(2*norm(V_r_wst)) , 2));
	Rednt_cir_R=sqrt(Rednt_cir_R);

   
	//�ꤤ���I��Elbow�V�q V_r_u
	Mat V_shx=(Mat_<float>(3,1) <<	1,  
									0,
									0);

	Mat V_shy=(Mat_<float>(3,1) <<	0,  
									1,
									0);

	Mat V_shz=(Mat_<float>(3,1) <<	0,  
									0,
									1);


	Mat temp_cross=V_r_wst.cross(V_shz);
	Mat V_alpha_hat=temp_cross/norm(temp_cross,NORM_L2);

	
	temp_cross=V_r_wst.cross(V_alpha_hat);
	Mat V_beta_hat=temp_cross/norm(temp_cross,NORM_L2);

	Mat V_r_wst_unit=V_r_wst/norm(V_r_wst,NORM_L2);
	Mat V_temp3x1=V_beta_hat*Rednt_cir_R;

	float *pV_temp3x1=V_temp3x1.ptr<float>(0);
	Mat V_temp4x1=(Mat_<float>(4,1)<<	pV_temp3x1[0],
										pV_temp3x1[1],
										pV_temp3x1[2],
										1);

	Mat temp=Rogridues(Rednt_alpha,V_r_wst_unit)*V_temp4x1;//temp=Rogridues(Rednt_alpha,V_r_wst/norm(V_r_wst)) *[Rednt_cir_R*V_beta_hat;1];  //Rednt_alpha����V�M�פ�W����V�ʬۤ� //0.04ms
	
	float* ptemp=temp.ptr<float>(0);

	Mat V_R_u=(Mat_<float>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);					

	Mat V_r_u=V_r_m+V_R_u;
	Mat V_r_f=V_r_wst-V_r_u;

	temp_cross=V_r_u.cross(V_r_f);
	Mat Vn_u_f=temp_cross/norm(temp_cross,NORM_L2);//Vn_u_f=cross(V_r_u,V_r_f)/norm(cross(V_r_u,V_r_f)); //ru �� rf��z�k�V�q
	
	float theta_upoff=atan(linkL[2]/linkL[1]);
	float* pV_r_u=V_r_u.ptr<float>(0);
	V_temp4x1=(Mat_<float>(4,1)<<	pV_r_u[0],
									pV_r_u[1],
									pV_r_u[2],
									1);				

	temp=Rogridues(-theta_upoff,Vn_u_f)*V_temp4x1;//temp=Rogridues(-theta_upoff,Vn_u_f)*[V_r_u;1];  //���� V_r_u  ��V_ru_l1
	Mat V_ru_l1=(Mat_<float>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);		
	V_ru_l1=linkL[1]*V_ru_l1/norm(V_ru_l1,NORM_L2); //�վ㦨L1����
	
	float* pV_ru_l1=V_ru_l1.ptr<float>(0);
	theta[Index_AXIS1]=atan2(pV_ru_l1[0],-pV_ru_l1[2]);//theta(1)=atan2(V_ru_l1(1),-V_ru_l1(3));

	if (theta[Index_AXIS1] !=0) 
		theta[Index_AXIS2]=atan2(pV_ru_l1[1],pV_ru_l1[0]/sin(theta[Index_AXIS1]));
	else
		theta[Index_AXIS2]=atan2(pV_ru_l1[1],-pV_ru_l1[2]);

	
	// ==AXIS3== //
	Mat nV_shy=V_shy*(-1);
	float* pnV_shy=nV_shy.ptr<float>(0);
	V_temp4x1=(Mat_<float>(4,1)<<	pnV_shy[0],
									pnV_shy[1],
									pnV_shy[2],
									1);			
	temp=RotY(-theta[Index_AXIS1])*RotX(theta[Index_AXIS2])*V_temp4x1;// V_n_yrot12=Ry(-theta(1))*Rx(theta(2))*[-V_shy;1];  //�Ĥ@�b�M�j�aY�y�Ф�V�ۤ�

	Mat V_n_yrot12=(Mat_<float>(3,1)<<	ptemp[0],
										ptemp[1],
										ptemp[2]);		

	Mat Vn_nuf_nyrot12=Vn_u_f.cross(V_n_yrot12);
	Vn_nuf_nyrot12=Vn_nuf_nyrot12/norm(Vn_nuf_nyrot12,NORM_L2);
	tempfloat=(float)(V_n_yrot12.dot(Vn_u_f)/norm(V_n_yrot12,NORM_L2)/norm(Vn_u_f,NORM_L2));// temp=V_n_yrot12'*Vn_u_f/norm(V_n_yrot12)/norm(Vn_u_f); 
	

	//����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}

	//Vn_u_f �M V_n_yrot12���k�V�q   �P V_ru_l1�P��V theta(3)�ݭn�[�t��
	if ( norm(Vn_nuf_nyrot12 - V_ru_l1/norm(V_ru_l1,NORM_L2),NORM_L2) < DEF_NORM_VERY_SMALL )
		theta[Index_AXIS3]=-acos(tempfloat);
	else
		theta[Index_AXIS3]=acos(tempfloat);

	
    // ==Axis5== //
	float theat_lowoff=atan(linkL[3]/linkL[4]); //temp=Rogridues(theat_lowoff,Vn_u_f)*[V_r_f;1];  //���� V_r_f  V_rf_l4
	float *pV_r_f=V_r_f.ptr<float>(0);
	V_temp4x1=(Mat_<float>(4,1)<<	pV_r_f[0],
									pV_r_f[1],
									pV_r_f[2],
									1);			
	temp=Rogridues(theat_lowoff,Vn_u_f)*V_temp4x1; 
	Mat V_rf_l4=(Mat_<float>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);		
	V_rf_l4=V_rf_l4*linkL[4]/norm(V_rf_l4,NORM_L2); //�վ㦨L4����

	//V_n_rfl4 ��V_n_rf�Φ������� ���k�V�q
	Mat Vn_rfl4_nuf=V_rf_l4.cross(Vn_u_f)/norm(V_rf_l4.cross(Vn_u_f));//Vn_rfl4_nuf=cross(V_rf_l4,Vn_u_f)/norm(cross(V_rf_l4,Vn_u_f));
	float t_rfl4_nuf=(float)((Vn_rfl4_nuf.dot(V_r_wst)-Vn_rfl4_nuf.dot(V_r_end))/pow(norm(Vn_rfl4_nuf,NORM_L2),2));//  t_rfl4_nuf=(Vn_rfl4_nuf'*V_r_wst-Vn_rfl4_nuf'*V_r_end)/(norm(Vn_rfl4_nuf)^2); //V_n_rf,V_n_rfl4�����W�A�B�g�LV_r_end�I�����u�ѼƦ���t ��rfl4_nuf
	Mat Vproj_end_rfl4_nuf =V_r_end+t_rfl4_nuf*Vn_rfl4_nuf;//V_r_end �u��V_n_rfl4,V_n_rf�����k�V�q��v�b�����W���I
	Mat V_wst_to_projend_rfl4_nuf=Vproj_end_rfl4_nuf-V_r_wst;

	//V_n_rfl4 ��V_n_rf�Φ������� ���k�V�q
	//����bacos(1.000000.....)���ɭԷ|�X�{�곡�����p
	tempfloat=(float)(V_rf_l4.dot(V_wst_to_projend_rfl4_nuf)/norm(V_rf_l4,NORM_L2)/norm(V_wst_to_projend_rfl4_nuf,NORM_L2));//temp=V_rf_l4'*V_wst_to_projend_rfl4_nuf/norm(V_rf_l4)/norm(V_wst_to_projend_rfl4_nuf);

	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL) 
	{
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}	

	Mat V_rf_l4_unit=V_rf_l4/norm(V_rf_l4,NORM_L2);
	Mat V_wst_to_projend_rfl4_nuf_unit=V_wst_to_projend_rfl4_nuf/norm(V_wst_to_projend_rfl4_nuf,NORM_L2);
	Mat Vn_rfl4_WstToProjEndRfl4Nuf=V_rf_l4_unit.cross(V_wst_to_projend_rfl4_nuf_unit);
	Vn_rfl4_WstToProjEndRfl4Nuf=Vn_rfl4_WstToProjEndRfl4Nuf/norm(Vn_rfl4_WstToProjEndRfl4Nuf,NORM_L2);

	//�����k�V�q �M Vn_rfl4_nuf  �P��n�[�t��  �P�_theta5�n���W�Ω��U��
	if (norm(Vn_rfl4_WstToProjEndRfl4Nuf - Vn_rfl4_nuf,NORM_L2) < DEF_NORM_VERY_SMALL)
        theta[Index_AXIS5]=-acos(tempfloat); 
    else
        theta[Index_AXIS5]=acos(tempfloat); 

	// ==Axis6== //
	float *pVn_u_f=Vn_u_f.ptr<float>(0);
	V_temp4x1=(Mat_<float>(4,1)<<	pVn_u_f[0],
									pVn_u_f[1],
									pVn_u_f[2],
									1);			
	temp=Rogridues(-theta[Index_AXIS5],Vn_rfl4_nuf)*V_temp4x1; //nuf �u�� Vn_rfl4_nuf �����5�b���ױo���v�I�P�ؼ��I�������k�V�q
	Mat Vn_nuf_rotx5_along_NRfl4Nuf=(Mat_<float>(3,1)<<	ptemp[0],
														ptemp[1],
														ptemp[2]);		
	Vn_nuf_rotx5_along_NRfl4Nuf=Vn_nuf_rotx5_along_NRfl4Nuf/norm(Vn_nuf_rotx5_along_NRfl4Nuf,NORM_L2);

	Mat V_wst_to_end=V_r_end-V_r_wst;
	    
	Mat Vn_WstToEnd_WstToProjEndRfl4Nuf=V_wst_to_end.cross(V_wst_to_projend_rfl4_nuf);//V_wst_to_projend �M V_wst_to_end���k�V�q
	Vn_WstToEnd_WstToProjEndRfl4Nuf=Vn_WstToEnd_WstToProjEndRfl4Nuf/norm(Vn_WstToEnd_WstToProjEndRfl4Nuf,NORM_L2);

	//�Q�Ϊk�V�q��V �P�_theta6�����V
	tempfloat=(float)(V_wst_to_projend_rfl4_nuf.dot(V_wst_to_end)/norm(V_wst_to_projend_rfl4_nuf,NORM_L2)/norm(V_wst_to_end,NORM_L2));//temp=V_wst_to_projend_rfl4_nuf'*V_wst_to_end/norm(V_wst_to_projend_rfl4_nuf)/norm(V_wst_to_end);
	if (norm(Vn_WstToEnd_WstToProjEndRfl4Nuf - Vn_nuf_rotx5_along_NRfl4Nuf) < DEF_NORM_VERY_SMALL)
        theta[Index_AXIS6]=-acos(tempfloat); 
    else
        theta[Index_AXIS6]=acos(tempfloat); 
	
	// ==Axis7== //	
	float *pV_shx=V_shx.ptr<float>(0);
	V_temp4x1=(Mat_<float>(4,1)<<	pV_shx[0],
									pV_shx[1],
									pV_shx[2],
									1);		
	Mat V_x_rot1to6=RotY(-theta[Index_AXIS1])*RotX(theta[Index_AXIS2])*V_temp4x1;//V_x_rot1to6=Ry(-theta(1))*Rx(theta(2))*[V_shx;1];  //�Ĥ@�b�M�j�aZ�y�Ф�V�ۤ�

	//V_shx�g�L1to�b����������ӭn�P���I�y�Шt��Z�b�K��
	temp=Rogridues(theta[Index_AXIS3],V_ru_l1/norm(V_ru_l1,NORM_L2))*V_x_rot1to6;		//temp=Rogridues(theta(3),V_ru_l1/norm(V_ru_l1))*V_x_rot1to6;  
	temp=Rogridues(theta[Index_AXIS4],Vn_u_f*(1/norm(Vn_u_f)))*temp;				//temp=Rogridues(theta(4),Vn_u_f/norm(Vn_u_f))*temp;  
	temp=Rogridues(theta[Index_AXIS5],Vn_rfl4_nuf*(1/norm(Vn_rfl4_nuf)))*temp;		//temp=Rogridues(theta(5),Vn_rfl4_nuf/norm(Vn_rfl4_nuf))*temp; 
	temp=Rogridues(theta[Index_AXIS6],Vn_nuf_rotx5_along_NRfl4Nuf)*temp;			//temp=Rogridues(theta(6),Vn_nuf_rotx5_along_NRfl4Nuf)*temp; 
	
	V_x_rot1to6=(Mat_<float>(3,1)<<	ptemp[0],
									ptemp[1],
									ptemp[2]);	
	V_x_rot1to6=V_x_rot1to6/norm(V_x_rot1to6,NORM_L2);

	//xrot1to6 �M V_H_hat_z ���k�V�q�ӧP�_��7�b�����V
	Mat Vn_xrot1to6_VHhatz=V_x_rot1to6.cross(V_H_hat_z);
	Vn_xrot1to6_VHhatz=Vn_xrot1to6_VHhatz/norm(Vn_xrot1to6_VHhatz,NORM_L2);

	//V_shx�g�L123456�b�����M���I�y�Шt��Z�b�ٮt�X��
	tempfloat=(float)(V_x_rot1to6.dot(V_H_hat_z)/norm(V_x_rot1to6,NORM_L2)/norm(V_H_hat_z,NORM_L2));// theta(7)=acos(V_x_rot1to6'*V_H_hat_z/norm(V_x_rot1to6)/norm(V_H_hat_z));
	if (abs(tempfloat-1) < DEF_COSVAL_VERY_SMALL)
    {   
		if (tempfloat >0)
			tempfloat=1;
		else
			tempfloat=-1;
	}
	if (norm(Vn_xrot1to6_VHhatz - V_H_hat_x,NORM_L2) <  DEF_NORM_VERY_SMALL)
        theta[Index_AXIS7]=acos(tempfloat);
    else
        theta[Index_AXIS7]=-acos(tempfloat);  
	
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


int MoveToPoint(int RLHand,float Point[7],float vel_deg)  //point[x,y,z,alpha,beta,gamma,redant_alpha]
{
	const float linkL[6]={L0,L1,L2,L3,L4,L5};
	float base[3]={0.0};
	float Pend[3]={Point[DEF_X],Point[DEF_Y],Point[DEF_Z]};
	float Pose_rad[3]={Point[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Point[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Point[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};
	float Rednt_alpha=Point[DEF_REDNT_ALPHA]*DEF_RATIO_DEG_TO_RAD;
	float theta[7]={0};
	int vel_pus=(int)(vel_deg*DEF_RATIO_VEL_DEG_TO_PUS);
	int rt=0;
	
	//inverse kinematics
	if(RLHand==DEF_RIGHT_HAND)
		base[DEF_Y]=-L0;
	else if(RLHand==DEF_LEFT_HAND)
		base[DEF_Y]=L0;

	//LARGE_INTEGER nFreq;
	//LARGE_INTEGER nBeginTime;

	//LARGE_INTEGER nEndTime;

	//double time;
	//double delaytime=1;

	//QueryPerformanceFrequency(&nFreq);
	//QueryPerformanceCounter(&nBeginTime); 



	rt= IK_7DOF_FB7roll(RLHand,linkL,base,Pend,Pose_rad,Rednt_alpha,theta);

	//QueryPerformanceCounter(&nEndTime);
	//time=(double)(nEndTime.QuadPart-nBeginTime.QuadPart)*1000/(double)nFreq.QuadPart;
	//printf("%fms\n",time);


	if(RLHand==DEF_RIGHT_HAND)
	{
		for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		{
			DBGMSG(("R%d:%3.0f, ",gMapAxisNO[i],theta[i]*DEF_RATIO_RAD_TO_DEG))
		}
		DBGMSG(("\n"))
	}
	else if(RLHand==DEF_LEFT_HAND)
	{
		for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
		{
			DBGMSG(("L%d:%3.0f, ",gMapAxisNO[i],theta[i]*DEF_RATIO_RAD_TO_DEG))
		}
		DBGMSG(("\n"))
	}
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
	unsigned short int velocity[MAX_AXIS_NUM]={vel_pus,vel_pus,vel_pus,vel_pus,vel_pus,vel_pus,vel_pus};
	
	rt=Output_to_Dynamixel(RLHand,theta,velocity); 

	return 0;
}

//#define CHECK_JOINT_PATH
#ifdef	CHECK_JOINT_PATH
#include<fstream>
extern fstream fileR;
extern fstream fileR;
#endif
int MoveToPoint_Dual(float Point_R[7],float vel_deg_R,float Point_L[7],float vel_deg_L)
{
	const float linkL[6]={L0,L1,L2,L3,L4,L5};
	float base_R[3]={0,-L0,0};
	float base_L[3]={0,L0,0};

	float Pend_R[3]={Point_R[DEF_X],Point_R[DEF_Y],Point_R[DEF_Z]};
	float Pend_L[3]={Point_L[DEF_X],Point_L[DEF_Y],Point_L[DEF_Z]};
	float Pose_rad_R[3]={Point_R[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Point_R[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Point_R[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};
	float Pose_rad_L[3]={Point_L[DEF_ALPHA]*DEF_RATIO_DEG_TO_RAD,Point_L[DEF_BETA]*DEF_RATIO_DEG_TO_RAD,Point_L[DEF_GAMMA]*DEF_RATIO_DEG_TO_RAD};

	float Rednt_alpha_rad_R=Point_R[DEF_REDNT_ALPHA]*DEF_RATIO_DEG_TO_RAD;
	float Rednt_alpha_rad_L=Point_L[DEF_REDNT_ALPHA]*DEF_RATIO_DEG_TO_RAD;

	float theta_R[7]={0};
	float theta_L[7]={0};
	int vel_pus_R=(int)(vel_deg_R*DEF_RATIO_VEL_DEG_TO_PUS);
	int vel_pus_L=(int)(vel_deg_R*DEF_RATIO_VEL_DEG_TO_PUS);
	int rt=0;
	int over_index=0;
	bool bOver=false;

	//inverse kinematics right hand
	rt= IK_7DOF_FB7roll(DEF_RIGHT_HAND,linkL,base_R,Pend_R,Pose_rad_R,Rednt_alpha_rad_R,theta_R);


	//�T�{joint �ϥ�
#ifdef CHECK_JOINT_PATH
	char buffer[100];
	int n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",theta_R[Index_AXIS1]*DEF_RATIO_RAD_TO_DEG,theta_R[Index_AXIS2]*DEF_RATIO_RAD_TO_DEG,theta_R[Index_AXIS3]*DEF_RATIO_RAD_TO_DEG,theta_R[Index_AXIS4]*DEF_RATIO_RAD_TO_DEG,theta_R[Index_AXIS5]*DEF_RATIO_RAD_TO_DEG,theta_R[Index_AXIS6]*DEF_RATIO_RAD_TO_DEG,theta_R[Index_AXIS7]*DEF_RATIO_RAD_TO_DEG);
	fileR.write(buffer,n);
#endif
	//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//{
	//	DBGMSG(("R%d:%3.0f, ",gMapAxisNO[i],theta_R[i]*DEF_RATIO_RAD_TO_DEG))
	//}
	//DBGMSG(("\n"))

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
	
	//�T�{joint �ϥ�
#ifdef CHECK_JOINT_PATH
	 buffer[100];
	 n=sprintf_s(buffer,sizeof(buffer),"%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f,%4.1f\n",theta_L[Index_AXIS1]*DEF_RATIO_RAD_TO_DEG,theta_L[Index_AXIS2]*DEF_RATIO_RAD_TO_DEG,theta_L[Index_AXIS3]*DEF_RATIO_RAD_TO_DEG,theta_L[Index_AXIS4]*DEF_RATIO_RAD_TO_DEG,theta_L[Index_AXIS5]*DEF_RATIO_RAD_TO_DEG,theta_L[Index_AXIS6]*DEF_RATIO_RAD_TO_DEG,theta_L[Index_AXIS7]*DEF_RATIO_RAD_TO_DEG);
	 fileR.write(buffer,n);
#endif

	//for(int i=Index_AXIS1;i<=Index_AXIS7;i++)
	//{
	//	DBGMSG(("L%d:%3.0f, ",gMapAxisNO[i],theta_L[i]*DEF_RATIO_RAD_TO_DEG))
	//}
	//DBGMSG(("\n"))

	
	//==prevent angle over constrain left hand 
	over_index=0;
	bOver=AngleOverConstrain(DEF_LEFT_HAND,theta_L,&over_index);
	if(bOver)
	{
		DBGMSG(("axis%d=%f,over constrain, %f< axis%d < %f\n",gMapAxisNO[over_index],theta_L[over_index]*DEF_RATIO_RAD_TO_DEG,grobot_lim_rad_L_Low[over_index]*DEF_RATIO_RAD_TO_DEG,gMapAxisNO[over_index],grobot_lim_rad_L_High[over_index]*DEF_RATIO_RAD_TO_DEG))
		return 1;
	}


	//output to motor
	unsigned short int velocity_R[MAX_AXIS_NUM]={vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R,vel_pus_R};
	unsigned short int velocity_L[MAX_AXIS_NUM]={vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L,vel_pus_L};
	
	rt=Output_to_Dynamixel_Dual(theta_R,velocity_R,theta_L,velocity_L); 

	return 0;
}

int IsMoving(int RLHand,bool *stillmoving)	
{
	int rt=0;
	int moving=0;

	for(int i=Index_AXIS1;i<MAX_AXIS_NUM;i++)
	{
		//read pulse
		if(RLHand==DEF_RIGHT_HAND)
			moving = dxl_read_byte(gMapRAxisID[i], STILL_MOVING);
		else if(RLHand==DEF_LEFT_HAND)
			moving = dxl_read_byte(gMapLAxisID[i], STILL_MOVING);

		//If communication ok 
		if(dxl_get_result()==COMM_RXSUCCESS)
		{
			if(moving==1)
				break;	
		}	
		else
		{
			rt=1;//communication error
			break; 
		}
	}


	(*stillmoving)=(moving==1)?true:false;
	return rt;
		
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
	//const int default_portnum=6;//latte_panda
	//const int default_portnum=5;//my pc
	const int default_portnum=5;//vaio plate
	const int default_baudnum=1;

	printf("DXL_port=%d\n",default_portnum);
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
#ifdef MODBUS_GRIPPER
#pragma comment(lib,"modbus.lib")

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

#endif

//=========================================
//==LattePanda Arduino Leonardo for Gripper
//==using C# "LattePanda Firmata
//=========================================
using namespace  LattePandaFirmata;
#using "LattePandaFirmata.dll"
#include <windows.h> //sleep�ϥ�
#pragma warning (disable: 4538)

#define DEF_LATTE_D1_GRIPPER_R1		1
#define DEF_LATTE_D2_GRIPPER_R2		2
#define DEF_LATTE_D3_GRIPPER_RPWM	3

#define DEF_LATTE_D4_GRIPPER_L1		4
#define DEF_LATTE_D5_GRIPPER_L2		5
#define DEF_LATTE_D6_GRIPPER_LPWM	6

#define DEF_LATTE_D13_LED		13
ref class GlobalObjects
{
public:
	static Arduino ^arduino=nullptr;

	static int Initial()
	{
		arduino = gcnew Arduino();
		arduino->pinMode(DEF_LATTE_D1_GRIPPER_R1, arduino->OUTPUT);//Set the digital pin 1 as output      
		arduino->pinMode(DEF_LATTE_D2_GRIPPER_R2, arduino->OUTPUT);//Set the digital pin 2 as output
		arduino->pinMode(DEF_LATTE_D3_GRIPPER_RPWM, arduino->PWM);//Set the digital pin 3 as pwm

		arduino->pinMode(DEF_LATTE_D4_GRIPPER_L1, arduino->OUTPUT);//Set the digital pin 4 as output      
		arduino->pinMode(DEF_LATTE_D5_GRIPPER_L2, arduino->OUTPUT);//Set the digital pin 5 as output
		arduino->pinMode(DEF_LATTE_D6_GRIPPER_LPWM, arduino->PWM);//Set the digital pin 6 as pwm

		arduino->pinMode(DEF_LATTE_D13_LED, arduino->OUTPUT);//Set the digital pin 13 as output just test


		arduino->analogWrite(DEF_LATTE_D3_GRIPPER_RPWM, 185);//0~255  255:4.5V  185:3.3V
		arduino->analogWrite(DEF_LATTE_D6_GRIPPER_LPWM, 185);//0~255
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

int Gripper_LattePanda_Hold(int RLHand,bool Hold,int delay_ms)
{
	//=========
	//=Test LED
	//=========
    //for(int i=0;i<5;i++)
    //{
    //    // ==== set the led on or off  
    //    GlobalObjects::arduino->digitalWrite(DEF_LATTE_D13_LED, GlobalObjects::arduino->HIGH);//set the LED�@on  
    //    Sleep(500);//delay a seconds  
    //    GlobalObjects::arduino->digitalWrite(DEF_LATTE_D13_LED, GlobalObjects::arduino->LOW);//set the LED�@off  
    //    Sleep(500);//delay a seconds  
    //}

	if(RLHand==DEF_RIGHT_HAND)
	{
		if(Hold)
		{
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D1_GRIPPER_R1, GlobalObjects::arduino->HIGH);
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R2, GlobalObjects::arduino->LOW);
		}
		else
		{
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D1_GRIPPER_R1, GlobalObjects::arduino->LOW);
			GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R2, GlobalObjects::arduino->HIGH);
		}

		QPDelay_ms(delay_ms);
	
		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D1_GRIPPER_R1, GlobalObjects::arduino->LOW);
		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D2_GRIPPER_R2, GlobalObjects::arduino->LOW);

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

		QPDelay_ms(delay_ms);

		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D4_GRIPPER_L1, GlobalObjects::arduino->LOW);
		GlobalObjects::arduino->digitalWrite(DEF_LATTE_D5_GRIPPER_L2, GlobalObjects::arduino->LOW);
	}



	return 0;
}


//=========================================
//==RoboticIO
//=========================================
using namespace RoboticArmIO;
#using "RoboticArmIO.dll"
#pragma warning (disable: 4538)

ref class F446REObj
{
public:
	static RobotIO ^robotio=nullptr;

	static int Initial()
	{
		robotio = gcnew RobotIO();
		robotio->initial();
		return 0;
	}
	static int Close()
	{
		robotio->close();
		return 0;
	}
	static bool RotateMotor(bool dir,int deg) 
	{
		return robotio->RotateMotor(dir,deg);
	}
	static void Gripper_Hold(int RLHand,bool Hold,int delay_ms)
	{
		robotio->Gripper_Hold(RLHand,Hold,delay_ms);
	}

	static void FootLifter(bool sw)
    {
       robotio->FootLifter(sw);
    }

    static void Spindle(bool sw)
    {
         robotio->Spindle(sw);
    }

	 static void Trimmer(bool sw)
    {
         robotio->Trimmer(sw);
    }
};


int F446RE_Initial()
{
	 F446REObj::Initial();
	 return 0;
}

void F446RE_Close()
{
	 F446REObj::Close();
}

void F446RE_RotateMotor(bool dir,int deg)
{
	 F446REObj::RotateMotor(dir,deg);
}

void F446RE_Gripper_Hold(int RLHand,bool Hold,int delay_ms)
{
	 F446REObj::Gripper_Hold(RLHand,Hold,delay_ms);
}

void F446RE_FootLifter(bool sw)
{
	F446REObj::FootLifter(sw);
}

void F446RE_Spindle(bool sw)
{
	F446REObj::Spindle(sw);
}

void F446RE_Trimmer(bool sw)
{
	F446REObj::Trimmer(sw);
}