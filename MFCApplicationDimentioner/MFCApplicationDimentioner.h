
// MFCApplicationDimentioner.h : main header file for the MFCApplicationDimentioner application
//
#pragma once

#ifndef __AFXWIN_H__
	#error "include 'stdafx.h' before including this file for PCH"
#endif

#include "resource.h"       // main symbols


// CMFCApplicationDimentionerApp:
// See MFCApplicationDimentioner.cpp for the implementation of this class
//

class CMFCApplicationDimentionerApp : public CWinApp
{
public:
	CMFCApplicationDimentionerApp();


// Overrides
public:
	virtual BOOL InitInstance();

// Implementation
	UINT  m_nAppLook;
	afx_msg void OnAppAbout();
	DECLARE_MESSAGE_MAP()
};

extern CMFCApplicationDimentionerApp theApp;
