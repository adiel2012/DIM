
// MFCApplicationDimentionerView.cpp : implementation of the CMFCApplicationDimentionerView class
//

#include "stdafx.h"
// SHARED_HANDLERS can be defined in an ATL project implementing preview, thumbnail
// and search filter handlers and allows sharing of document code with that project.
#ifndef SHARED_HANDLERS
#include "MFCApplicationDimentioner.h"
#endif

#include "MFCApplicationDimentionerDoc.h"
#include "MFCApplicationDimentionerView.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// CMFCApplicationDimentionerView

IMPLEMENT_DYNCREATE(CMFCApplicationDimentionerView, CView)

BEGIN_MESSAGE_MAP(CMFCApplicationDimentionerView, CView)
END_MESSAGE_MAP()

// CMFCApplicationDimentionerView construction/destruction

CMFCApplicationDimentionerView::CMFCApplicationDimentionerView()
{
	// TODO: add construction code here

}

CMFCApplicationDimentionerView::~CMFCApplicationDimentionerView()
{
}

BOOL CMFCApplicationDimentionerView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: Modify the Window class or styles here by modifying
	//  the CREATESTRUCT cs

	return CView::PreCreateWindow(cs);
}

// CMFCApplicationDimentionerView drawing

void CMFCApplicationDimentionerView::OnDraw(CDC* /*pDC*/)
{
	CMFCApplicationDimentionerDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	// TODO: add draw code for native data here
}


// CMFCApplicationDimentionerView diagnostics

#ifdef _DEBUG
void CMFCApplicationDimentionerView::AssertValid() const
{
	CView::AssertValid();
}

void CMFCApplicationDimentionerView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CMFCApplicationDimentionerDoc* CMFCApplicationDimentionerView::GetDocument() const // non-debug version is inline
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CMFCApplicationDimentionerDoc)));
	return (CMFCApplicationDimentionerDoc*)m_pDocument;
}
#endif //_DEBUG


// CMFCApplicationDimentionerView message handlers
