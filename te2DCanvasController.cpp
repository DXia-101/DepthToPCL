#include "te2DCanvasController.h"
#include <QVBoxLayout>

te2DCanvasController::Garbo te2DCanvasController::tmp;

te2DCanvasController* te2DCanvasController::instance = nullptr;

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
	connect(this, &te2DCanvasController::sig_ClearAll2DCanvasMarks, m_te2DCanvas, &te2DCanvas::ClearAll2DCanvasMarks);
}

te2DCanvasController::~te2DCanvasController()
{}

te2DCanvasController::te2DCanvasController(const te2DCanvasController&)
{
	
}

te2DCanvasController& te2DCanvasController::operator=(const te2DCanvasController&)
{
	return *this;
}

te2DCanvasController* te2DCanvasController::getInstance()
{
	if (!instance)
	{
		te2DCanvasController* pInstance = new te2DCanvasController();
		instance = pInstance;
	}
	return instance;
}

void te2DCanvasController::destroy()
{
	if (NULL != te2DCanvasController::instance) {
		delete te2DCanvasController::instance;
		te2DCanvasController::instance = NULL;
	}
}

void te2DCanvasController::displayUIInWidget(QVBoxLayout * layout)
{
	layout->addWidget(m_te2DCanvasToolBar);
	layout->addWidget(m_te2DCanvas);
	m_te2DCanvasToolBar->show();
	m_te2DCanvas->show();
	layout->setStretchFactor(m_te2DCanvasToolBar, 1);
	layout->setStretchFactor(m_te2DCanvas, 9);
}

void te2DCanvasController::setImage(const te::Image& img, bool resetView)
{
	m_te2DCanvas->setImage(img, resetView);
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
