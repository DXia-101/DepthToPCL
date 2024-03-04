#include "te2DCanvasController.h"
#include <QVBoxLayout>

te2DCanvasController::te2DCanvasController(QObject *parent)
	: QObject(parent)
{
	m_te2DCanvas = new te2DCanvas();
	m_te2DCanvasToolBar = new te2DCanvasToolBar();

	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasDrawStatus, m_te2DCanvas, &te2DCanvas::ReplaceToDrawState);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasEraseStatus, m_te2DCanvas, &te2DCanvas::ReplaceToEraseState);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasRedo, m_te2DCanvas, &te2DCanvas::Redo);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasUndo, m_te2DCanvas, &te2DCanvas::Undo);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasShapeSelected, m_te2DCanvas, &te2DCanvas::ShapeSelect);
}

te2DCanvasController::~te2DCanvasController()
{}

void te2DCanvasController::displayUIInWidget(QVBoxLayout * layout)
{
	layout->addWidget(m_te2DCanvasToolBar);
	layout->addWidget(m_te2DCanvas);
	m_te2DCanvasToolBar->show();
	m_te2DCanvas->show();
	layout->setStretchFactor(m_te2DCanvasToolBar, 1);
	layout->setStretchFactor(m_te2DCanvas, 9);
}

void te2DCanvasController::showAllUI()
{
	m_te2DCanvasToolBar->show();
	m_te2DCanvas->show();
}

void te2DCanvasController::hideAllUI()
{
	m_te2DCanvasToolBar->hide();
	m_te2DCanvas->hide();
}
