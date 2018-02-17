#include "stdafx.h"
#include "CCameraCalibrator.h"


#include "afxwinappex.h"
#include "afxdialogex.h"



BEGIN_MESSAGE_MAP(CCameraCalibratorDialog, CDialogEx)
	ON_BN_CLICKED(IDC_BUTTON_CLOSE, &CCameraCalibratorDialog::OnClickedButtonClose)
	ON_BN_CLICKED(IDC_BUTTON_CALIBRATE, &CCameraCalibratorDialog::OnClickedButtonCalibrate)
END_MESSAGE_MAP()



CCameraCalibratorDialog::CCameraCalibratorDialog() : CDialogEx(IDD_DIALOG_CALIBRATION)
{
}

CCameraCalibratorDialog::~CCameraCalibratorDialog()
{
}


void CCameraCalibratorDialog::OnClickedButtonClose()
{
	// TODO: Add your control notification handler code here
}


void CCameraCalibratorDialog::OnClickedButtonCalibrate()
{
	// TODO: Add your control notification handler code here


}
