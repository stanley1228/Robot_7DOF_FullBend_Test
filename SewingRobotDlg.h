
// SewingRobotDlg.h : ���Y��
//

#pragma once


// CSewingRobotDlg ��ܤ��
class CSewingRobotDlg : public CDialogEx
{
// �غc
public:
	CSewingRobotDlg(CWnd* pParent = NULL);	// �зǫغc�禡

// ��ܤ�����
	enum { IDD = IDD_SEWINGROBOT_DIALOG };

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV �䴩


// �{���X��@
protected:
	HICON m_hIcon;

	// ���ͪ��T�������禡
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedBtnIniDxl();
	afx_msg void OnBnClickedBtnRightRel();
	afx_msg void OnBnClickedBtnSewprocess();
	afx_msg void OnBnClickedBtnLeftHold();
	afx_msg void OnBnClickedBtnLeftRel();
	afx_msg void OnBnClickedBtnRightHold();
	afx_msg void OnBnClickedBtnIniF446();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedCancel();
	afx_msg void OnClose();
	afx_msg void OnBnClickedChkSpindle();
	afx_msg void OnBnClickedChkFootlifter();
	afx_msg void OnBnClickedBtnMovetohome();
	afx_msg void OnBnClickedBtnTorqueDisable();
	afx_msg void OnBnClickedBtnMovetoInit();
};
