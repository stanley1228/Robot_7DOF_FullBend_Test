// Robot_7DOF_FullBendTest.cpp : 定義主控台應用程式的進入點。
//

#include "stdafx.h"
#include "Robot_7DOF_FB.h"

#define _USE_MATH_DEFINES // for C++
#include <math.h>
using namespace std;

int _tmain(int argc, _TCHAR* argv[])
{

	float theta[7]={0};
	int rt = IK_7DOF_stanley(L1,L2,L3,0,0,0,60,0,0,0,0,0,(float)-M_PI*0.5,theta);

	return 0;
}

