
// SewingRobotDlg.cpp : ��@��
//

#include "stdafx.h"
#include "SewingRobot.h"
#include "SewingRobotDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// �� App About �ϥ� CAboutDlg ��ܤ��

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// ��ܤ�����
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV �䴩

// �{���X��@
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(CAboutDlg::IDD)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CSewingRobotDlg ��ܤ��



CSewingRobotDlg::CSewingRobotDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(CSewingRobotDlg::IDD, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CSewingRobotDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CSewingRobotDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_BTN_INI_DXL, &CSewingRobotDlg::OnBnClickedBtnIniDxl)
	ON_BN_CLICKED(IDC_BTN_RIGHT_REL, &CSewingRobotDlg::OnBnClickedBtnRightRel)
	ON_BN_CLICKED(IDC_BTN_SewProcess, &CSewingRobotDlg::OnBnClickedBtnSewprocess)
	ON_BN_CLICKED(IDC_BTN_LEFT_HOLD, &CSewingRobotDlg::OnBnClickedBtnLeftHold)
	ON_BN_CLICKED(IDC_BTN_LEFT_REL, &CSewingRobotDlg::OnBnClickedBtnLeftRel)
	ON_BN_CLICKED(IDC_BTN_RIGHT_HOLD, &CSewingRobotDlg::OnBnClickedBtnRightHold)
	ON_BN_CLICKED(IDC_BTN_INI_F446, &CSewingRobotDlg::OnBnClickedBtnIniF446)
	ON_BN_CLICKED(IDOK, &CSewingRobotDlg::OnBnClickedOk)
	ON_BN_CLICKED(IDCANCEL, &CSewingRobotDlg::OnBnClickedCancel)
	ON_WM_CLOSE()
	ON_BN_CLICKED(IDC_CHK_SPINDLE, &CSewingRobotDlg::OnBnClickedChkSpindle)
	ON_BN_CLICKED(IDC_CHK_FOOTLIFTER, &CSewingRobotDlg::OnBnClickedChkFootlifter)
	ON_BN_CLICKED(IDC_BTN_MOVETOHOME, &CSewingRobotDlg::OnBnClickedBtnMovetohome)
	ON_BN_CLICKED(IDC_BTN_TORQUE_DISABLE, &CSewingRobotDlg::OnBnClickedBtnTorqueDisable)
	ON_BN_CLICKED(IDC_BTN_MOVETO_INIT, &CSewingRobotDlg::OnBnClickedBtnMovetoInit)
END_MESSAGE_MAP()


#include <io.h>  
#include <fcntl.h>
void InitConsole()  
{  
    int nRet= 0;  
    FILE* fp;  
    AllocConsole();  
    nRet= _open_osfhandle((long)GetStdHandle(STD_OUTPUT_HANDLE), _O_TEXT);  
    fp = _fdopen(nRet, "w");  
    *stdout = *fp;  
    setvbuf(stdout, NULL, _IONBF, 0);  
}  

// CSewingRobotDlg �T���B�z�`��

BOOL CSewingRobotDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// �N [����...] �\���[�J�t�Υ\���C

	// IDM_ABOUTBOX �����b�t�ΩR�O�d�򤧤��C
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// �]�w����ܤ�����ϥܡC�����ε{�����D�������O��ܤ���ɡA
	// �ج[�|�۰ʱq�Ʀ��@�~
	SetIcon(m_hIcon, TRUE);			// �]�w�j�ϥ�
	SetIcon(m_hIcon, FALSE);		// �]�w�p�ϥ�

	// TODO: �b���[�J�B�~����l�]�w
	//CString xx;
	//xx.Format('%d',500);
	SetDlgItemText(IDC_EDIT_REL_TIME,"500");
	SetDlgItemText(IDC_EDIT_HOLD_TIME,"500");
	InitConsole();

	//(CEdit*)GetDlgItem(IDC_EDIT_REL_TIME)->SetWindowText(xx);


	return TRUE;  // �Ǧ^ TRUE�A���D�z�ﱱ��]�w�J�I
}

void CSewingRobotDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// �p�G�N�̤p�ƫ��s�[�J�z����ܤ���A�z�ݭn�U�C���{���X�A
// �H�Kø�s�ϥܡC���ϥΤ��/�˵��Ҧ��� MFC ���ε{���A
// �ج[�|�۰ʧ������@�~�C

void CSewingRobotDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // ø�s���˸m���e

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// �N�ϥܸm����Τ�ݯx��
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// �yø�ϥ�
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// ��ϥΪ̩즲�̤p�Ƶ����ɡA
// �t�ΩI�s�o�ӥ\����o�����ܡC
HCURSOR CSewingRobotDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


#include "../Robot_7DOF_FullBendTest/Robot_7DOF_FB.h"
cF446RE* gpF446RE;


void CSewingRobotDlg::OnBnClickedBtnIniDxl()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	int rt=DXL_Initial_x86();
	if(rt==0)
	{
		printf("DXL_Initial_x86 failed\n");
		//getchar();
		//return 0;
	}
}

void CSewingRobotDlg::OnBnClickedBtnSewprocess()
{
	PID_Setting_Dual();
	SetAllAccTo(20); //20 deg/acc^
	TestSewingAction();
}


void CSewingRobotDlg::OnBnClickedBtnLeftRel()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	int releasetime = GetDlgItemInt(IDC_EDIT_REL_TIME);
	//F446RE_Gripper_Hold(DEF_LEFT_HAND,false,releasetime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, false, releasetime);
}
void CSewingRobotDlg::OnBnClickedBtnRightRel()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	int releasetime = GetDlgItemInt(IDC_EDIT_REL_TIME);
	//F446RE_Gripper_Hold(DEF_RIGHT_HAND,false,releasetime);
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, false, releasetime);
}

void CSewingRobotDlg::OnBnClickedBtnLeftHold()
{
	
	// TODO: �b���[�J����i���B�z�`���{���X
	int releasetime = GetDlgItemInt(IDC_EDIT_HOLD_TIME);
	//F446RE_Gripper_Hold(DEF_LEFT_HAND,true,releasetime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, true, releasetime);
}


void CSewingRobotDlg::OnBnClickedBtnRightHold()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	int releasetime = GetDlgItemInt(IDC_EDIT_HOLD_TIME);
	//F446RE_Gripper_Hold(DEF_RIGHT_HAND,true,releasetime);
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, true, releasetime);
}


void CSewingRobotDlg::OnBnClickedBtnIniF446()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	//F446RE_Initial();
	gpF446RE = new cF446RE(3, 9600);
	printf("F446RE_Initial\n");
}


void CSewingRobotDlg::OnBnClickedOk()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	CDialogEx::OnOK();
}


void CSewingRobotDlg::OnBnClickedCancel()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	CDialogEx::OnCancel();
}


void CSewingRobotDlg::OnClose()
{
	// TODO: �b���[�J�z���T���B�z�`���{���X�M (��) �I�s�w�]��

	DXL_Terminate_x86();
	//F446RE_Close();


	CDialogEx::OnClose();
}


void CSewingRobotDlg::OnBnClickedChkSpindle()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	bool sw=((CButton*)GetDlgItem(IDC_CHK_SPINDLE))->GetCheck();
	//F446RE_Spindle(sw);
	gpF446RE->Spindle(sw);
}


void CSewingRobotDlg::OnBnClickedChkFootlifter()
{
	bool sw=((CButton*)GetDlgItem(IDC_CHK_FOOTLIFTER))->GetCheck();
	//F446RE_FootLifter(sw);
	gpF446RE->FootLifter(sw);
}


void CSewingRobotDlg::OnBnClickedBtnMovetohome()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	TestMoveToSewingHome_Dual();
}


void CSewingRobotDlg::OnBnClickedBtnTorqueDisable()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	Torque_Disable();
}


void CSewingRobotDlg::OnBnClickedBtnMovetoInit()
{
	// TODO: �b���[�J����i���B�z�`���{���X
	CStaArray R_IniP(-90,-90,0,50,0,0,-50);
	CStaArray L_IniP(-90,90,0,-90,0,0,90);
	MoveToInitailPoint(R_IniP,L_IniP);
}
