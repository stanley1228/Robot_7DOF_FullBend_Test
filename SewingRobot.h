
// SewingRobot.h : PROJECT_NAME ���ε{�����D�n���Y��
//

#pragma once

#ifndef __AFXWIN_H__
	#error "�� PCH �]�t���ɮ׫e���]�t 'stdafx.h'"
#endif

#include "resource.h"		// �D�n�Ÿ�


// CSewingRobotApp:
// �аѾ\��@�����O�� SewingRobot.cpp
//

class CSewingRobotApp : public CWinApp
{
public:
	CSewingRobotApp();

// �мg
public:
	virtual BOOL InitInstance();

// �{���X��@

	DECLARE_MESSAGE_MAP()
};

extern CSewingRobotApp theApp;