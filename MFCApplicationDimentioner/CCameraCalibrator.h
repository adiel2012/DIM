#pragma once

#include "resource.h"

#ifndef __CameraCalibrator
#define __CameraCalibrator


class CCameraCalibratorDialog : public CDialogEx
{
public:
	CCameraCalibratorDialog();
	~CCameraCalibratorDialog();
	// Dialog Data
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_CALIBRATION };
#endif

protected:
	//virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support

	// Implementation
protected:
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnClickedButtonClose();
	afx_msg void OnClickedButtonCalibrate();
};



#endif // __CameraCalibrator




