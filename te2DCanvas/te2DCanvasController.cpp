#include "te2DCanvasController.h"
#include <QVBoxLayout>
#include "teDataStorage.h"
#include "Depth2RGB.h"

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
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowDimension, m_te2DCanvas, &te2DCanvas::ShowDimension);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowResult, m_te2DCanvas, &te2DCanvas::ShowResult);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowLocalMask , m_te2DCanvas, &te2DCanvas::ShowLocalMask);
	connect(m_te2DCanvasToolBar, &te2DCanvasToolBar::sig_ShowGlobalMask, m_te2DCanvas, &te2DCanvas::ShowGlobalMask);
	connect(this, &te2DCanvasController::sig_ClearAll2DCanvasSymbol, m_te2DCanvas, &te2DCanvas::RemoveDimentsion);
	connect(this, &te2DCanvasController::sig_ClearAll2DCanvasSymbol, m_te2DCanvas, &te2DCanvas::RemoveResult);
	connect(this, &te2DCanvasController::sig_StartMarking, m_te2DCanvas, &te2DCanvas::StartMarked);
	connect(this, &te2DCanvasController::sig_StartMarking, this, &te2DCanvasController::ShowFirstImage);
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
}

void te2DCanvasController::showAllUI()
{
	m_te2DCanvasToolBar->show();
	m_te2DCanvas->show();
	if (IsNeedReload) 
	{
		cv::Mat image = cv::imread(teDataStorage::getInstance()->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
		if (image.empty()) {
			return;
		}
		m_te2DCanvas->RemoveDimentsion();
		m_te2DCanvas->RemoveResult();
		TeJetColorCode trans;
		m_te2DCanvas->setImage(trans.dealWithCvt(image, teDataStorage::getInstance()->getCurrentIndex()));
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
	teDataStorage::getInstance()->clearCurrentTrainSampleMark();
	te::SampleMark sampleMark = teDataStorage::getInstance()->getCurrentTrainSampleInfo();
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
	teDataStorage::getInstance()->updateCurrentTrainSampleMark(sampleMark);
	emit teDataStorage::getInstance()->sig_updateCurrentTrainSampleMark();
}

void te2DCanvasController::ShowFirstImage()
{
	connect(this, &te2DCanvasController::sig_currentLabelChange, m_te2DCanvas, &te2DCanvas::LabelChanged);
	std::string str = teDataStorage::getInstance()->getSelectOriginImage(0);
	cv::Mat image = cv::imread(str,cv::IMREAD_UNCHANGED);
	if (image.empty()) {
		return;
	}
	emit sig_ClearAll2DCanvasSymbol();
	TeJetColorCode trans;
	m_te2DCanvas->setImage(trans.dealWithCvt(image, 0));
	ShowAllItems();
}

void te2DCanvasController::ShowAllItems()
{
	if (m_te2DCanvasToolBar->isDimensionShow() == true) {
		ShowAllMarkers();
	}
	if (m_te2DCanvasToolBar->isResultShow() == true) {
		ShowAllResults();
	}
	if (m_te2DCanvasToolBar->isLocalMaskShow()) {

	}
	if (m_te2DCanvasToolBar->isGlobalMaskShow()) {

	}
}

void te2DCanvasController::ShowCurrentImages()
{
	cv::Mat image = cv::imread(teDataStorage::getInstance()->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	if (image.empty()) {
		return;
	}
	TeJetColorCode trans;
	m_te2DCanvas->setImage(trans.dealWithCvt(image, -1));
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

void te2DCanvasController::ShowAllResults()
{
	te::SampleMark samplemark = teDataStorage::getInstance()->getCurrentResultSampleInfo();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te2DCanvas->ResultsShowInCanvas(&instance, QString::fromStdString(instance.name), teDataStorage::getInstance()->FindContentColor(QString::fromStdString(instance.name)));
	}
}

void te2DCanvasController::ShowAllMarkers()
{
	te::SampleMark samplemark = teDataStorage::getInstance()->getCurrentTrainSampleInfo();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te2DCanvas->MarkersShowInCanvas(&instance, QString::fromStdString(instance.name), teDataStorage::getInstance()->FindContentColor(QString::fromStdString(instance.name)));
	}
}

void te2DCanvasController::hideAllUI()
{
	m_te2DCanvasToolBar->hide();
	m_te2DCanvas->hide();
}
