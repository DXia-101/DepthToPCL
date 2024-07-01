#include "te3DCanvasController.h"
#include <QVBoxLayout>
#include <QStackedLayout>
#include "Transfer_Function.h"
#include "teAiModel.h"
#include "te3DCanvas.h"
#include "te3DCanvasMenu.h"
#include "te3DCanvasToolBar.h"
#include "te3DPolyLine.h"
#include "teLabelBrowser.h"

#define DEBUG
#include <QTime>
#include <QDebug>

te3DCanvasController::te3DCanvasController(QObject *parent)
	: QObject(parent)
{
	m_te3DCanvas = new te3DCanvas();
	m_te3DCanvasMenu = new te3DCanvasMenu();
	m_te3DCanvasToolBar = new te3DCanvasToolBar();
	m_te3DPolyLine = new te3DPolyLine();
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_HeightTransform, m_te3DCanvas, &te3DCanvas::HeightTransform);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_PerspectiveToXaxis, m_te3DCanvas, &te3DCanvas::PerspectiveToXaxis);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_PerspectiveToYaxis, m_te3DCanvas, &te3DCanvas::PerspectiveToYaxis);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_PerspectiveToZaxis, m_te3DCanvas, &te3DCanvas::PerspectiveToZaxis);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_GtCheckStateChanged, m_te3DCanvas,&te3DCanvas::ShowDimension);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_RSTCheckStateChanged, m_te3DCanvas,&te3DCanvas::ShowResult);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_StartMarking, this, &te3DCanvasController::StartDrawPolyLine);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_ReloadPointCloud, this, &te3DCanvasController::ReloadPointCloud);

	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_CoordinateAxis, m_te3DCanvas, &te3DCanvas::CoordinateAxisRendering);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_BackgroundColor, m_te3DCanvas, &te3DCanvas::SetBackgroundColor);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_CoordinateAxis, m_te3DCanvas, &te3DCanvas::CoordinateAxisRendering);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_PointCloudColor, m_te3DCanvas, &te3DCanvas::PointCloudColorSet);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_PointCloudPointSize, m_te3DCanvas, &te3DCanvas::PointCloudPointSizeSet);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_AABBSurrounding, m_te3DCanvas, &te3DCanvas::AxisAlignedBoundingBox);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_OBBSurrounding, m_te3DCanvas, &te3DCanvas::OrientedBoundingBox);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_CrossSection, m_te3DCanvas, &te3DCanvas::CrossSection);
	
	connect(m_te3DCanvas, &te3DCanvas::sig_3DCanvasMarkingCompleted, this, &te3DCanvasController::add3DAiInstance);	
	connect(m_te3DCanvas, &te3DCanvas::sig_ShowAllItems, this, &te3DCanvasController::ShowAllItems);

	connect(this, &te3DCanvasController::sig_setHeightCoefficientFactor, m_te3DCanvasMenu, &te3DCanvasMenu::setHeightCoefficientFactor);
	connect(this, &te3DCanvasController::sig_CurrentStateChanged, m_te3DCanvas, &te3DCanvas::CurrentStateChanged);
	connect(this, &te3DCanvasController::sig_MarkerButtonRecovery, m_te3DCanvasMenu, &te3DCanvasMenu::ButtonRecovery);
}

te3DCanvasController::~te3DCanvasController()
{
}

void te3DCanvasController::displayToolBarInWidget(QVBoxLayout* layout)
{
	layout->addWidget(m_te3DCanvasToolBar);
	layout->addWidget(m_te3DCanvasMenu);
	m_te3DCanvasToolBar->show();
	m_te3DCanvasMenu->show();
}

void te3DCanvasController::displayCanvasInWidget(QStackedLayout* layout)
{
	layout->addWidget(m_te3DCanvas);
	layout->addWidget(m_te3DPolyLine);
	m_te3DCanvas->show();
	m_te3DPolyLine->hide();
}

void te3DCanvasController::SavePointCloud(QString filepath, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcr)
{
	m_te3DCanvas->SavePointCloud(filepath, pcr);
}

void te3DCanvasController::setteAiModel(teAiModel* aiModel)
{
	m_teAiModel = aiModel;
}

void te3DCanvasController::setteLabelBrowser(teLabelBrowser* labelBrowser)
{
	m_teLabelBrowser = labelBrowser;
}

QRect te3DCanvasController::getGeometry()
{
	return m_te3DCanvas->geometry();
}

void te3DCanvasController::ManagePolyLine(QStackedLayout* layout)
{
	if (layout->currentWidget() == m_te3DPolyLine) {
		layout->setCurrentWidget(m_te3DCanvas);
	}
	else {
		layout->setCurrentWidget(m_te3DPolyLine);
	}
}

void te3DCanvasController::NeedReload()
{
	IsNeedReload = true;
}

void te3DCanvasController::hideAllUI()
{
	m_te3DCanvasToolBar->hide();
	m_te3DCanvasMenu->hide();
	m_te3DCanvas->hide();
}

void te3DCanvasController::showAllUI()
{
	m_te3DCanvasToolBar->show();
	m_te3DCanvasMenu->show();
	m_te3DCanvas->show();
	if (IsNeedReload)
	{
		LoadPointCloud(QString::fromStdString(m_teAiModel->getCurrentPointCloud()));
		IsNeedReload = false;
	}
}

void te3DCanvasController::add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	te::AiInstance instance;
	instance.name = m_teLabelBrowser->getSelectedRowCategory().toStdString();
	std::vector<std::vector<cv::Point>> contours = Transfer_Function::Cloud2Contour(m_teAiModel->getCurrentImageWidth(), m_teAiModel->getCurrentImageHeight(), cloud);
	
	te::PolygonF polygon;
	for (const std::vector<cv::Point>& contourPoints : contours) {
		te::PolygonF::PointType point;
		for (const cv::Point& contourPoint : contourPoints) {
			point.x = static_cast<float>(contourPoint.x);
			point.y = static_cast<float>(contourPoint.y);
			polygon.push_back(point);
		}
		instance.contour.polygons.push_back(polygon);
		polygon.clear();
	}
	te::SampleMark sampleMark = m_teAiModel->getCurrentTrainSampleInfo();
	sampleMark.gtDataSet.push_back(instance);
	m_teAiModel->updateCurrentTrainSampleMark(sampleMark);
	emit sig_updateTrainWidget();
}

void te3DCanvasController::ShowAllItems()
{
	ShowAllMarkers();
	ShowAllResults();
	if (m_te3DCanvasMenu->isDimensionShow() == false) {
		m_te3DCanvas->ShowDimension(0);
	}
	if (m_te3DCanvasMenu->isResultShow() == false) {
		m_te3DCanvas->ShowResult(0);
	}
}

void te3DCanvasController::LoadPointCloud(QString fileName)
{
	m_te3DCanvas->LoadPointCloud(fileName);
	m_te3DCanvasMenu->on_ConfirmTransformationBtn_clicked();
	m_te3DCanvasToolBar->MaintainCoordinateAxis();
	ShowAllItems();
}

void te3DCanvasController::ReLoadGTAndRST()
{
	m_te3DCanvas->UpdateDimentsion();
	m_te3DCanvas->UpdateResult();
	//ShowAllItems();
	m_te3DCanvas->getvtkRenderer()->GetRenderWindow()->Render();
}

void te3DCanvasController::SetCentroid()
{
	m_te3DCanvas->setRotationCenter();
}

void te3DCanvasController::StartDrawPolyLine()
{
	if (m_te3DPolyLine->isVisible()) {
		m_te3DPolyLine->hide();
		if (m_te3DPolyLine->GetPointList().size() > 2) 
		{
			m_te3DCanvas->te3DCanvasStartMarking(m_te3DPolyLine->GetPointList());
		}
	}
	else {
		m_te3DPolyLine->show();
		m_te3DPolyLine->SetDraw(true);
	}

	emit sig_ManagePolyLine();
}

void te3DCanvasController::ReloadPointCloud()
{
	m_te3DCanvas->ReductionPointCloud(QString::fromStdString(m_teAiModel->getCurrentPointCloud()));
}

void te3DCanvasController::ShowAllResults()
{
	cv::Mat image = cv::imread(m_teAiModel->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	te::SampleMark samplemark = m_teAiModel->getCurrentResultSampleInfo();
	m_te3DCanvas->ClearmarkerPCID();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te3DCanvas->ResultsShowInCanvas(&instance, image, m_teLabelBrowser->getFontColorByFirstColumnValue(QString::fromStdString(instance.name)));
	}
}

void te3DCanvasController::ShowAllMarkers()
{
	cv::Mat image = cv::imread(m_teAiModel->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	te::SampleMark samplemark = m_teAiModel->getCurrentTrainSampleInfo();
	m_te3DCanvas->ClearresultPCID();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te3DCanvas->MarkersShowInCanvas(&instance, image, m_teLabelBrowser->getFontColorByFirstColumnValue(QString::fromStdString(instance.name)));
	}
}
