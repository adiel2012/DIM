
// MFCApplicationDimentionerView.h : interface of the CMFCApplicationDimentionerView class
//

#pragma once


class CMFCApplicationDimentionerView : public CView
{
protected: // create from serialization only
	CMFCApplicationDimentionerView();
	DECLARE_DYNCREATE(CMFCApplicationDimentionerView)

// Attributes
public:
	CMFCApplicationDimentionerDoc* GetDocument() const;

// Operations
public:

// Overrides
public:
	virtual void OnDraw(CDC* pDC);  // overridden to draw this view
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// Implementation
public:
	virtual ~CMFCApplicationDimentionerView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// Generated message map functions
protected:
	DECLARE_MESSAGE_MAP()
};

#ifndef _DEBUG  // debug version in MFCApplicationDimentionerView.cpp
inline CMFCApplicationDimentionerDoc* CMFCApplicationDimentionerView::GetDocument() const
   { return reinterpret_cast<CMFCApplicationDimentionerDoc*>(m_pDocument); }
#endif

