#include "te3DCanvasController.h"
#include <QVBoxLayout>
#include "Transfer_Function.h"
#include "teDataStorage.h"

te3DCanvasController::Garbo te3DCanvasController::tmp;

te3DCanvasController* te3DCanvasController::instance = nullptr;

te3DCanvasController::te3DCanvasController(QObject *parent)
	: QObject(parent)
{
	m_te3DCanvas = new te3DCanvas();
	m_te3DCanvasMenu = new te3DCanvasMenu();
	m_te3DCanvasToolBar = new te3DCanvasToolBar();
	//m_interactor = CustomInteractorStyle::New();
	//m_interactor->setRenderWindow(m_te3DCanvas->m_renderWindow,m_te3DCanvas->m_renderer);
	//m_te3DCanvas->m_renderWindow->GetInteractor()->SetInteractorStyle(m_interactor);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_HeightTransform, m_te3DCanvas, &te3DCanvas::PointCloudHeightTransform);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_ConnectHeightTransForm, this, &te3DCanvasController::sig_ConnectHeightTransform);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_HeightTransform, this, &te3DCanvasController::SaveHeightTransFromFactor);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_DisconnectHeightTransForm, this, &te3DCanvasController::sig_DisonnectHeightTransform);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_DisconnectHeightTransForm, m_te3DCanvas, &te3DCanvas::ReductionPointCloud);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_PerspectiveToXaxis, m_te3DCanvas, &te3DCanvas::PerspectiveToXaxis);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_PerspectiveToYaxis, m_te3DCanvas, &te3DCanvas::PerspectiveToYaxis);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_PerspectiveToZaxis, m_te3DCanvas, &te3DCanvas::PerspectiveToZaxis);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_StartMarking, m_te3DCanvas, &te3DCanvas::te3DCanvasStartMarking);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_GtCheckStateChanged, m_te3DCanvas,&te3DCanvas::ShowDimension);
	connect(m_te3DCanvasMenu, &te3DCanvasMenu::sig_RSTCheckStateChanged, m_te3DCanvas,&te3DCanvas::ShowResult);
	
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_BackgroundColorSetting, this, &te3DCanvasController::BackgroundColorSelect);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_CoordinateAxisRendering, this, &te3DCanvasController::CoordinateAxisSelect);

	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_PointCloudColorSetting, this, &te3DCanvasController::PointCloudColorSelect);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_PointCloudPointSizeSetting, this, &te3DCanvasController::PointCloudPointSizeSelect);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_GaussFilter, this, &te3DCanvasController::GaussFilterAction);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_DirectFilter, this, &te3DCanvasController::DirectFilterAction);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_AABBSurrounding, m_te3DCanvas, &te3DCanvas::AxisAlignedBoundingBox);
	connect(m_te3DCanvasToolBar, &te3DCanvasToolBar::sig_OBBSurrounding, m_te3DCanvas, &te3DCanvas::OrientedBoundingBox);
	connect(this, &te3DCanvasController::sig_CoordinateAxis, m_te3DCanvas, &te3DCanvas::CoordinateAxisRendering);
	
	connect(m_te3DCanvas, &te3DCanvas::sig_3DCanvasMarkingCompleted, this, &te3DCanvasController::add3DAiInstance);
	connect(this, &te3DCanvasController::sig_LabelChanged, m_te3DCanvas, &te3DCanvas::LabelChanged);

	connect(this, &te3DCanvasController::sig_SavePointCloud, m_te3DCanvas, &te3DCanvas::SavePointCloud);
	connect(this, &te3DCanvasController::sig_LoadPointCloud, m_te3DCanvas, &te3DCanvas::LoadPointCloud);
	connect(this, &te3DCanvasController::sig_ReRenderOriginCloud, m_te3DCanvas, &te3DCanvas::reRenderOriginCloud);
	connect(this, &te3DCanvasController::sig_currentLabelChange, m_te3DCanvas, &te3DCanvas::LabelChanged);

	connect(this, &te3DCanvasController::sig_ShowAllPointCloud, this, &te3DCanvasController::ShowAllItems);
	
}

te3DCanvasController::~te3DCanvasController()
{
	//m_interactor->Delete();
}

te3DCanvasController::te3DCanvasController(const te3DCanvasController&)
{

}

te3DCanvasController& te3DCanvasController::operator=(const te3DCanvasController&)
{
	return *this;
}

te3DCanvasController* te3DCanvasController::getInstance()
{
	if (!instance)
	{
		te3DCanvasController* pInstance = new te3DCanvasController();
		instance = pInstance;
	}
	return instance;
}

void te3DCanvasController::destroy()
{
	if (NULL != te3DCanvasController::instance) {
		delete te3DCanvasController::instance;
		te3DCanvasController::instance = NULL;
	}
}

void te3DCanvasController::displayUIInWidget(QVBoxLayout* layout)
{	
	layout->addWidget(m_te3DCanvasToolBar);
	layout->addWidget(m_te3DCanvasMenu);
	layout->addWidget(m_te3DCanvas);
	m_te3DCanvasToolBar->show();
	m_te3DCanvasMenu->show();
	m_te3DCanvas->show();
	layout->setStretchFactor(m_te3DCanvasToolBar, 1);
	layout->setStretchFactor(m_te3DCanvasMenu, 1);
	layout->setStretchFactor(m_te3DCanvas, 10);
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
	sig_LoadPointCloud(QString::fromStdString(teDataStorage::getInstance()->getCurrentPointCloud()));
	sig_ReRenderOriginCloud();
	sig_ShowAllPointCloud();
}

void te3DCanvasController::add3DAiInstance(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	cv::Mat image(0, 0, CV_32F);

	Transfer_Function::Cloud2cvMat(m_te3DCanvas->getAxisSet().curwidth, m_te3DCanvas->getAxisSet().curheight, m_te3DCanvas->getAxisSet().OriginX, m_te3DCanvas->getAxisSet().OriginY, cloud, image);

	te::AiInstance instance;
	instance.name = teDataStorage::getInstance()->getCurrentLabelCategory().toStdString();
	std::vector<std::vector<cv::Point>> contours;
	Transfer_Function::cvMat2Contour(image, &contours);
	//contours.erase(contours.begin());
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
	te::SampleMark sampleMark = teDataStorage::getInstance()->getCurrentTrainSampleInfo();
	sampleMark.gtDataSet.push_back(instance);
	teDataStorage::getInstance()->updateCurrentTrainSampleMark(sampleMark);
}

void te3DCanvasController::ShowAllItems()
{
	if (m_te3DCanvasMenu->isDimensionShow()) {
		ShowAllMarkers();
	}
	if (m_te3DCanvasMenu->isResultShow()) {
		ShowAllResults();
	}
}

void te3DCanvasController::HegithTransForm()
{
	m_te3DCanvas->PointCloudHeightTransform(hegithTransFactor);
}

void te3DCanvasController::SaveHeightTransFromFactor(int factor)
{
	hegithTransFactor = factor;
}

void te3DCanvasController::SaveAxis(QString axis)
{
	this->axis = axis;
	connect(this, &te3DCanvasController::sig_MaintainCoordinateAxis, this, &te3DCanvasController::MaintainCoordinateAxis);
}

void te3DCanvasController::MaintainCoordinateAxis()
{
	emit sig_CoordinateAxis(this->axis);
}

void te3DCanvasController::ShowAllResults()
{
	cv::Mat image = cv::imread(teDataStorage::getInstance()->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	te::SampleMark samplemark = teDataStorage::getInstance()->getCurrentResultSampleInfo();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te3DCanvas->MarkersShowInCanvas(&instance, image, teDataStorage::getInstance()->FindContentColor(QString::fromStdString(instance.name)));
	}
}

void te3DCanvasController::ShowAllMarkers()
{
	cv::Mat image = cv::imread(teDataStorage::getInstance()->getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	te::SampleMark samplemark = teDataStorage::getInstance()->getCurrentTrainSampleInfo();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		m_te3DCanvas->MarkersShowInCanvas(&instance, image, teDataStorage::getInstance()->FindContentColor(QString::fromStdString(instance.name)));
	}
}

void te3DCanvasController::BackgroundColorSelect()
{
	dialog_colorselect = new te3DCanvasPointCloudColorSelectDialog();
	QColor color = dialog_colorselect->getColor();

	connect(this, &te3DCanvasController::sig_BackgroundColor, m_te3DCanvas, &te3DCanvas::SetBackgroundColor);
	emit sig_BackgroundColor(color);
}

void te3DCanvasController::CoordinateAxisSelect()
{
	dialog_render = new te3DCanvasCoordinateAxisRenderDialog();
	connect(dialog_render, &te3DCanvasCoordinateAxisRenderDialog::sig_CoordinateAxisRender, this, &te3DCanvasController::sig_CoordinateAxis);
	connect(dialog_render, &te3DCanvasCoordinateAxisRenderDialog::sig_CoordinateAxisRender, this, &te3DCanvasController::SaveAxis);
	if (dialog_render->exec() == QDialog::Accepted) {}
	delete dialog_render;
}

void te3DCanvasController::PointCloudColorSelect()
{
	dialog_colorselect = new te3DCanvasPointCloudColorSelectDialog();
	QColor color = dialog_colorselect->getColor();
	delete dialog_colorselect;

	connect(this, &te3DCanvasController::sig_PointCloudColor, m_te3DCanvas, &te3DCanvas::PointCloudColorSet);
	emit sig_PointCloudColor(color);
}

void te3DCanvasController::PointCloudPointSizeSelect()
{
	pointsize_set_dialog = new PointCloud_PointSize_Set_Dialog();
	int point_size = pointsize_set_dialog->GetSize();
	if (pointsize_set_dialog->exec() == QDialog::Accepted) {}
	delete pointsize_set_dialog;

	connect(this, &te3DCanvasController::sig_PointCloudPointSize, m_te3DCanvas, &te3DCanvas::PointCloudPointSizeSet);
	emit sig_PointCloudPointSize(point_size);
}

void te3DCanvasController::GaussFilterAction()
{
	dialog_Guass_filter = new Filter_Guass();
	connect(dialog_Guass_filter, &Filter_Guass::sendData, m_te3DCanvas, &te3DCanvas::GuassFilter);
	if (dialog_Guass_filter->exec() == QDialog::Accepted) {}
	delete dialog_Guass_filter;
}

void te3DCanvasController::DirectFilterAction()
{
	dialog_Direct_filter = new Filter_Direct();
	connect(dialog_Direct_filter, &Filter_Direct::sendData, m_te3DCanvas, &te3DCanvas::DirectFilter);
	if (dialog_Direct_filter->exec() == QDialog::Accepted) {}
	delete dialog_Direct_filter;
}
