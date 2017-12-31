
// SewingRobotDlg.cpp : 實作檔
//

#include "stdafx.h"
#include "SewingRobot.h"
#include "SewingRobotDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 對 App About 使用 CAboutDlg 對話方塊

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 對話方塊資料
	enum { IDD = IDD_ABOUTBOX };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支援

// 程式碼實作
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


// CSewingRobotDlg 對話方塊



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

// CSewingRobotDlg 訊息處理常式

BOOL CSewingRobotDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 將 [關於...] 功能表加入系統功能表。

	// IDM_ABOUTBOX 必須在系統命令範圍之中。
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

	// 設定此對話方塊的圖示。當應用程式的主視窗不是對話方塊時，
	// 框架會自動從事此作業
	SetIcon(m_hIcon, TRUE);			// 設定大圖示
	SetIcon(m_hIcon, FALSE);		// 設定小圖示

	// TODO: 在此加入額外的初始設定
	//CString xx;
	//xx.Format('%d',500);
	SetDlgItemText(IDC_EDIT_REL_TIME,"500");
	SetDlgItemText(IDC_EDIT_HOLD_TIME,"500");
	InitConsole();

	//(CEdit*)GetDlgItem(IDC_EDIT_REL_TIME)->SetWindowText(xx);


	return TRUE;  // 傳回 TRUE，除非您對控制項設定焦點
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

// 如果將最小化按鈕加入您的對話方塊，您需要下列的程式碼，
// 以便繪製圖示。對於使用文件/檢視模式的 MFC 應用程式，
// 框架會自動完成此作業。

void CSewingRobotDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 繪製的裝置內容

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 將圖示置中於用戶端矩形
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 描繪圖示
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

// 當使用者拖曳最小化視窗時，
// 系統呼叫這個功能取得游標顯示。
HCURSOR CSewingRobotDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


#include "../Robot_7DOF_FullBendTest/Robot_7DOF_FB.h"
cF446RE* gpF446RE;


void CSewingRobotDlg::OnBnClickedBtnIniDxl()
{
	// TODO: 在此加入控制項告知處理常式程式碼
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
	// TODO: 在此加入控制項告知處理常式程式碼
	int releasetime = GetDlgItemInt(IDC_EDIT_REL_TIME);
	//F446RE_Gripper_Hold(DEF_LEFT_HAND,false,releasetime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, false, releasetime);
}
void CSewingRobotDlg::OnBnClickedBtnRightRel()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	int releasetime = GetDlgItemInt(IDC_EDIT_REL_TIME);
	//F446RE_Gripper_Hold(DEF_RIGHT_HAND,false,releasetime);
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, false, releasetime);
}

void CSewingRobotDlg::OnBnClickedBtnLeftHold()
{
	
	// TODO: 在此加入控制項告知處理常式程式碼
	int releasetime = GetDlgItemInt(IDC_EDIT_HOLD_TIME);
	//F446RE_Gripper_Hold(DEF_LEFT_HAND,true,releasetime);
	gpF446RE->Gripper_Hold(DEF_LEFT_HAND, true, releasetime);
}


void CSewingRobotDlg::OnBnClickedBtnRightHold()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	int releasetime = GetDlgItemInt(IDC_EDIT_HOLD_TIME);
	//F446RE_Gripper_Hold(DEF_RIGHT_HAND,true,releasetime);
	gpF446RE->Gripper_Hold(DEF_RIGHT_HAND, true, releasetime);
}


void CSewingRobotDlg::OnBnClickedBtnIniF446()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	//F446RE_Initial();
	gpF446RE = new cF446RE(3, 9600);
	printf("F446RE_Initial\n");
}


void CSewingRobotDlg::OnBnClickedOk()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	CDialogEx::OnOK();
}


void CSewingRobotDlg::OnBnClickedCancel()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	CDialogEx::OnCancel();
}


void CSewingRobotDlg::OnClose()
{
	// TODO: 在此加入您的訊息處理常式程式碼和 (或) 呼叫預設值

	DXL_Terminate_x86();
	//F446RE_Close();


	CDialogEx::OnClose();
}


void CSewingRobotDlg::OnBnClickedChkSpindle()
{
	// TODO: 在此加入控制項告知處理常式程式碼
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
	// TODO: 在此加入控制項告知處理常式程式碼
	TestMoveToSewingHome_Dual();
}


void CSewingRobotDlg::OnBnClickedBtnTorqueDisable()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	Torque_Disable();
}


void CSewingRobotDlg::OnBnClickedBtnMovetoInit()
{
	// TODO: 在此加入控制項告知處理常式程式碼
	CStaArray R_IniP(-90,-90,0,50,0,0,-50);
	CStaArray L_IniP(-90,90,0,-90,0,0,90);
	MoveToInitailPoint(R_IniP,L_IniP);
}
