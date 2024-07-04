#include "te2DCanvasController.h"
#include <QVBoxLayout>
#include "teAiModel.h"
#include "teLabelBrowser.h"
#include "Depth2RGB.h"

te2DCanvasController::te2DCanvasController(QObject *parent)
	: QObject(parent)
{
	m_te2DCanvas = new te2DCanvas();
	m_te2DCanvasToolBar = new te2DCanvasToolBar();

	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasDrawStatus, m_te2DCanvas, &te2DCanvas::sig_ReplaceToDrawState);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasEraseStatus, m_te2DCanvas, &te2DCanvas::sig_ReplaceToEraseState);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasRedo, m_te2DCanvas, &te2DCanvas::Redo);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasUndo, m_te2DCanvas, &te2DCanvas::Undo);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_te2DCanvasShapeSelected, m_te2DCanvas, &te2DCanvas::ShapeSelect);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowDimension, m_te2DCanvas, &te2DCanvas::ShowDimension);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowResult, m_te2DCanvas, &te2DCanvas::ShowResult);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowLocalMask , m_te2DCanvas, &te2DCanvas::ShowLocalMask);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowGlobalMask, m_te2DCanvas, &te2DCanvas::ShowGlobalMask);
	connect(this, &te2DCanvasController::sig_StartMark, m_te2DCanvas, &te2DCanvas::StartMarked);
	connect(this, &te2DCanvasController::sig_CurrentStateChanged, m_te2DCanvas, &te2DCanvas::CurrentStateChanged);
	connect(m_te2DCanvas, &te2DCanvas::sig_PolygonMarkingCompleted, this, &te2DCanvasController::add2DAiInstance);
	
}

te2DCanvasController::~te2DCanvasController()
{}

void te2DCanvasController::displayToolBarInWidget(QVBoxLayout* layout)
{
	layout->addWidget(m_te2DCanvasToolBar);
	m_te2DCanvasToolBar->show();
}

void te2DCanvasController::displayCanvasInWidget(QStackedLayout* layout)
{
	layout->addWidget(m_te2DCanvas);
	m_te2DCanvas->show();
}

void te2DCanvasController::setImage(const te::Image& img, bool resetView)
{
	m_te2DCanvas->setImage(img, resetView);
	emit sig_ResetMouseRadius();
}

void te2DCanvasController::setteAiModel(teAiModel* aiModel)
{
	m_teAiModel = aiModel;
}

void te2DCanvasController::setteLabelBrowser(teLabelBrowser* browser)
{
	m_teLabelBrowser = browser;
}

void te2DCanvasController::showAllUI()
{
	m_te2DCanvasToolBar->show();
	m_te2DCanvas->show();
	if (IsNeedReload) 
	{
		ShowCurrentImages();
		ShowAllItems();
		IsNeedReload = false;
	}
	else if(IsFirstShow != true)
	{
		ShowAllItems();
	}
	IsFirstShow = false;
}

void te2DCanvasController::add2DAiInstance(QList<te::GraphicsItem*> polygonItems)
{
	m_teAiModel->clearCurrentTrainSampleMark();
	te::SampleMark sampleMark = m_teAiModel->getCurrentTrainSampleInfo();
	for (te::GraphicsItem* item : polygonItems) {
		te::ConnectedRegionGraphicsItem* polygonItem = dynamic_cast<te::ConnectedRegionGraphicsItem*>(item);
		QList<QPolygonF> contours = polygonItem->polygonList();
		te::AiInstance instance;
		instance.name = polygonItem->label().toStdString();
		te::PolygonF polygon;
		te::PolygonF::PointType point;
		for (const QPointF& polygonPoint : contours.front()) {
			point.x = static_cast<float>(polygonPoint.x());
			point.y = static_cast<float>(polygonPoint.y());
			polygon.push_back(point);
		}
		instance.contour.polygons.push_back(polygon);
		sampleMark.gtDataSet.push_back(instance);
	}
	m_teAiModel->updateCurrentTrainSampleMark(sampleMark);
	emit sig_updateTrainWidget();
}

void te2DCanvasController::ShowAllItems()
{
	ShowAllMarkers();
	ShowAllResults();
	if (m_te2DCanvasToolBar->isDimensionShow() == false) {
		m_te2DCanvas->ShowDimension(0);
	}
	if (m_te2DCanvasToolBar->isResultShow() == false) {
		m_te2DCanvas->ShowResult(0);
	}
	if (m_te2DCanvasToolBar->isLocalMaskShow() == false) {

	}
	if (m_te2DCanvasToolBar->isGlobalMaskShow() == false) {

	}
}

void te2DCanvasController::ShowCurrentImages()
{
	cv::Mat image = cv::imread(m_teAiModel->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	if (image.empty()) {
		return;
	}
	TeJetColorCode trans;
	m_te2DCanvas->setImage(trans.dealWithCvt(image, m_teAiModel->getCurrentInvalidPointThreshold(), m_teAiModel->getCurrentValidPointThreshold()));
	cv::waitKey(0);
}

void te2DCanvasController::ReLoadGTAndRST()
{
	m_te2DCanvas->RemoveDimentsion();
	m_te2DCanvas->RemoveResult();
	ShowAllItems();
}

void te2DCanvasController::NeedReload()
{
	IsNeedReload = true;
}

void te2DCanvasController::slotSetImage(te::Image* img)
{
	setImage(*img);
}

void te2DCanvasController::LoadOriginImage(QString imagepath)
{
	m_te2DCanvas->RemoveDimentsion();
	m_te2DCanvas->RemoveResult();
	ShowCurrentImages();
	ShowAllItems();
}

void te2DCanvasController::receptiveFieldChange(double factor)
{
	factor *= m_te2DCanvas->GetScale();
	emit sig_receptiveFieldChange(factor);
}

void te2DCanvasController::ShowAllResults()
{
	te::SampleMark samplemark = m_teAiModel->getCurrentResultSampleInfo();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te2DCanvas->ResultsShowInCanvas(&instance, QString::fromStdString(instance.name), m_teLabelBrowser->getFontColorByFirstColumnValue(QString::fromStdString(instance.name)));
	}
}

void te2DCanvasController::ShowAllMarkers()
{
	te::SampleMark samplemark = m_teAiModel->getCurrentTrainSampleInfo();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te2DCanvas->MarkersShowInCanvas(&instance, QString::fromStdString(instance.name), m_teLabelBrowser->getFontColorByFirstColumnValue(QString::fromStdString(instance.name)));
	}
}

void te2DCanvasController::hideAllUI()
{
	m_te2DCanvasToolBar->hide();
	m_te2DCanvas->hide();
}
