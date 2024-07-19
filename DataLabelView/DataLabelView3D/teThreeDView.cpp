#include "teThreeDView.h"
#include "teThreeDViewMenber.h"
#include "teInteractorStyle.h"
#include <QRegularExpression>

using namespace te;

QString incrementNumber(const QString& input)
{
	QRegularExpression regex("(\\d+)$");
	QRegularExpressionMatch match = regex.match(input);
	if (match.hasMatch()) {
		QString numberStr = match.captured(1);
		int number = numberStr.toInt();
		number++;
		return QString::number(number);
	}
	return QString();
}


ThreeDView::ThreeDView(QWidget* parent)
	: QVTKOpenGLNativeWidget(parent)
{
	initWidget();
	initCoordinateAxis();
}

ThreeDView::~ThreeDView()
{
}

void te::ThreeDView::outOfBounds()
{
	viewModel.lock()->notified(ViewModel::OutOfBounds);
}

void ThreeDView::initWidget()
{
	menber = new ThreeDViewMenber();
	menber->cloud = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();

	menber->renderer = vtkSmartPointer<vtkRenderer>::New();
	menber->renderWindow = this->renderWindow();
	menber->renderWindow->AddRenderer(menber->renderer);
	//m_renderer->GetActiveCamera()->vtkCamera::ParallelProjectionOn();

	menber->viewer.reset(new pcl::visualization::PCLVisualizer(menber->renderer, menber->renderWindow, "viewer", false));
	this->setRenderWindow(menber->renderWindow);
	menber->viewer->setupInteractor(menber->renderWindow->GetInteractor(), menber->renderWindow);
	menber->viewer->setBackgroundColor(0.0, 0.3, 0.4);
	menber->viewer->addCoordinateSystem(1.0);
	this->update();
	menber->viewer->addPointCloud<pcl::PointXYZRGB>(menber->cloud, "cloud");
}

void ThreeDView::initCoordinateAxis()
{
	menber->axes_actor = vtkSmartPointer<vtkAxesActor>::New();
	menber->axes_actor->SetAxisLabels(1);
	menber->axes_actor->SetPosition(0, 0, 0);
	menber->axes_actor->SetTotalLength(2, 2, 2);
	menber->axes_actor->SetShaftType(0);
	menber->axes_actor->SetCylinderRadius(0.02);

	menber->customInteractor = InteractorStyle::New();
	menber->customInteractor->setRenderWindow(menber->renderWindow, menber->renderer, menber->axes_actor);
	menber->renderWindow->GetInteractor()->SetInteractorStyle(menber->customInteractor);

	setInteractorCallback(*this);

	menber->markerWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
	menber->markerWidget->SetOrientationMarker(menber->axes_actor);
	menber->markerWidget->SetInteractor(menber->renderWindow->GetInteractor());
	menber->markerWidget->SetEnabled(1);
	menber->markerWidget->SetInteractive(false);

	menber->renderWindow->Render();
}

void te::ThreeDView::RerenderWindow()
{
	adjustCamera();
	setRotationCenter();
	menber->customInteractor->resetData();
	setCoordinateInfo();
	menber->renderWindow->Render();
	menber->customInteractor->resetData();
}

void ThreeDView::setRotationCenter()
{
	std::vector<double> centroid = getCloudCentroid();
	menber->customInteractor->setRotationCenter(centroid[0], centroid[1], centroid[2]);
}

void ThreeDView::setCoordinateInfo()
{
	pcl::PointXYZ min;
	pcl::PointXYZ max;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::copyPointCloud(*menber->cloud, *cloud_xyz);
	pcl::getMinMax3D(*cloud_xyz, min, max);
	int currentDisplayImageLength = max.x - min.x;
	int currentDisplayImageHeight = max.y - min.y;
	viewModel.lock()->setCurrentDisplayImageLength(currentDisplayImageLength);
	viewModel.lock()->setCurrentDisplayImageHeight(currentDisplayImageHeight);
	viewModel.lock()->setCurrentMaxPt(max);
	viewModel.lock()->setCurrentMinPt(min);

}

void ThreeDView::setCameraPosition()
{
	if (!menber->cloud->empty()) {
		pcl::PointXYZ maxPt = viewModel.lock()->getCurrentMaxPt();
		pcl::PointXYZ minPt = viewModel.lock()->getCurrentMinPt();


		Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
		Eigen::Vector3f diff = maxPt.getVector3fMap() - minPt.getVector3fMap();
		float distance = diff.norm();

		CoordinateAxisDirection direction = viewModel.lock()->getCoordinateAxisDire();

		if (direction == PositiveXaxis)
			menber->viewer->setCameraPosition(center(0) + distance, center(1), center(2), center(0), center(1), center(2), 0, 1, 0);
		else if (direction == NegativeXaxis)
			menber->viewer->setCameraPosition(center(0) + distance, center(1), center(2), center(0), center(1), center(2), 0, -1, 0);
		else if (direction == PositiveYaxis)
			menber->viewer->setCameraPosition(center(0), center(1) + distance, center(2), center(0), center(1), center(2), 1, 0, 0);
		else if (direction == NegativeYaxis)
			menber->viewer->setCameraPosition(center(0), center(1) + distance, center(2), center(0), center(1), center(2), -1, 0, 0);
		else if (direction == PositiveZaxis)
			menber->viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 1, 0, 0);
		else if (direction == NegativeZaxis)
			menber->viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), -1, 0, 0);

		menber->viewer->spinOnce();
	}
}

void ThreeDView::adjustCamera()
{
	if (!menber->cloud->empty()) {
		pcl::PointXYZ maxPt = viewModel.lock()->getCurrentMaxPt();
		pcl::PointXYZ minPt = viewModel.lock()->getCurrentMinPt();

		Eigen::Vector3f center((maxPt.x + minPt.x) / 2, (maxPt.y + minPt.y) / 2, (maxPt.z + minPt.z) / 2);
		Eigen::Vector3f diff = maxPt.getVector3fMap() - minPt.getVector3fMap();
		float distance = diff.norm();

		menber->viewer->setCameraPosition(center(0), center(1), center(2) + distance, center(0), center(1), center(2), 0, 0, 0); //耗时最多 2秒多
		menber->viewer->spinOnce();
	}
}

void ThreeDView::adjustPointCloudTransform()
{
	vtkSmartPointer<vtkPropCollection> propCollection = menber->renderer->GetViewProps();
	vtkProp3D* pActor = vtkProp3D::SafeDownCast(propCollection->GetLastProp());
	pActor->SetUserTransform(menber->customInteractor->getRotationTransform());
	menber->renderWindow->Render();
}

void ThreeDView::setMarkerPointCloudVisible()
{
	int arg = viewModel.lock()->getThreeDisShowMarkers();
	for (const auto& pair : menber->markerPCID) {
		for (const QString& id : pair.second) {
			menber->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, arg, id.toStdString());
		}
	}
}

void ThreeDView::setResultPointCloudVisible()
{
	int arg = viewModel.lock()->getThreeDisShowResults();
	for (const auto& pair : menber->resultPCID) {
		for (const QString& id : pair.second) {
			menber->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, arg, id.toStdString());
		}
	}
}

void ThreeDView::initPointCloud()
{
	QString filePath = QString::fromStdString(viewModel.lock()->getCurrentPointCloud());
	if (!filePath.isEmpty())
	{
		menber->cloud->points.clear();
		if (filePath.endsWith("pcd"))
		{
			if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filePath.toStdString(), *menber->cloud) == -1)
			{
				return;
			}
		}
	}
	menber->renderWindow->Render();
	menber->customInteractor->setRotationCenter(getCloudCentroid()[0], getCloudCentroid()[1], getCloudCentroid()[2]);
	menber->customInteractor->resetData();
	viewModel.lock()->setVtkWindowSize(menber->renderer->GetVTKWindow()->GetSize());
	viewModel.lock()->setRenderViewport(menber->renderer->GetViewport());
}

void ThreeDView::updateMarkerPointCloud()
{
	QColor color = viewModel.lock()->getCurrentLabelInfo().second;
	auto itStr = menber->markerPCID.begin();
	auto itPC = menber->markerPointCloud.begin();
	for (itStr, itPC; itStr != menber->markerPCID.end(); ++itStr, ++itPC)
	{
		assert(itStr->first == itPC->first);
		for (int i = 0; i < itStr->second.size(); ++i)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(itPC->second.at(i), color.red(), color.green(), color.blue());
			menber->viewer->updatePointCloud(itPC->second.at(i), currentColor, itStr->second.at(i).toStdString());
		}
	}
}

void ThreeDView::updateResultPointCloud()
{
	QColor color = viewModel.lock()->getCurrentLabelInfo().second;
	auto itStr = menber->resultPCID.begin();
	auto itPC = menber->resultPointCloud.begin();
	for (itStr, itPC; itStr != menber->resultPCID.end(); ++itStr, ++itPC)
	{
		assert(itStr->first == itPC->first);
		for (int i = 0; i < itStr->second.size(); ++i)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(itPC->second.at(i), color.red(), color.green(), color.blue());
			menber->viewer->updatePointCloud(itPC->second.at(i), currentColor, itStr->second.at(i).toStdString());
		}
	}
}

void ThreeDView::addMarkerPointCloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr markPointCloud = viewModel.lock()->getSegmentedPointCloud();

	if (markPointCloud != nullptr)
	{
		std::pair<QString, QColor> labelInfo = viewModel.lock()->getCurrentLabelInfo();
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(markPointCloud, labelInfo.second.red(), labelInfo.second.green(), labelInfo.second.blue());
		QString CloudId;
		auto it = menber->markerPCID.find(labelInfo.first);
		auto pct = menber->markerPointCloud.find(labelInfo.first);
		int index = viewModel.lock()->getCurrentIndex();
		if (it != menber->markerPCID.end()) {
			QString count = incrementNumber(it->second.back());
			CloudId = labelInfo.first + QString::number(index) + "marker" + count;
			it->second.push_back(CloudId);
			pct->second.push_back(markPointCloud);
		}
		else {
			CloudId = labelInfo.first + QString::number(index) + "marker" + "0";
			menber->markerPCID.insert(std::make_pair(labelInfo.first, std::vector<QString>{CloudId}));
			menber->markerPointCloud.insert(std::make_pair(labelInfo.first, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{markPointCloud}));
		}
		menber->viewer->addPointCloud(markPointCloud, currentColor, CloudId.toStdString());
		adjustPointCloudTransform();
	}
}

void ThreeDView::showMarkerPointCloud()
{
	te::SampleMark samplemark = viewModel.lock()->getCurrentTrainSampleInfo();
	clearMarkerPointCloud();
	clearMarkerVector();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr markerPointCloud;
		markerPointCloud = viewModel.lock()->getPointCloudByContour(menber->cloud, &instance);
		QColor color = viewModel.lock()->getLabelColor(QString::fromStdString(instance.name));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(markerPointCloud, color.red(), color.green(), color.blue());
		QString CloudId;
		auto it = menber->markerPCID.find(QString::fromStdString(instance.name));
		auto pct = menber->markerPointCloud.find(QString::fromStdString(instance.name));
		int index = viewModel.lock()->getCurrentIndex();
		if (it != menber->markerPCID.end()) {
			QString count = incrementNumber(it->second.back());
			CloudId = QString::fromStdString(instance.name) + QString::number(index) + "marker" + count;
			it->second.push_back(CloudId);
			pct->second.push_back(markerPointCloud);
		}
		else {
			CloudId = QString::fromStdString(instance.name) + QString::number(index) + "marker" + "0";
			menber->markerPCID.insert(std::make_pair(QString::fromStdString(instance.name), std::vector<QString>{CloudId}));
			menber->markerPointCloud.insert(std::make_pair(QString::fromStdString(instance.name), std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{markerPointCloud}));
		}
		menber->viewer->addPointCloud(markerPointCloud, currentColor, CloudId.toStdString());

		adjustPointCloudTransform();
	}
}

void ThreeDView::showResultPointCloud()
{
	te::SampleMark samplemark = viewModel.lock()->getCurrentResultSampleInfo();
	clearResultPointCloud();
	clearResultVector();
	for (te::AiInstance instance : samplemark.gtDataSet) {
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultPointCloud;
		resultPointCloud = viewModel.lock()->getPointCloudByContour(menber->cloud, &instance);
		QColor color = viewModel.lock()->getLabelColor(QString::fromStdString(instance.name));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> currentColor(resultPointCloud, color.red(), color.green(), color.blue());
		QString CloudId;
		auto it = menber->resultPCID.find(QString::fromStdString(instance.name));
		auto pct = menber->resultPointCloud.find(QString::fromStdString(instance.name));
		int index = viewModel.lock()->getCurrentIndex();
		if (it != menber->resultPCID.end()) {
			QString count = incrementNumber(it->second.back());
			CloudId = QString::fromStdString(instance.name) + QString::number(index) + "result" + count;
			it->second.push_back(CloudId);
			pct->second.push_back(resultPointCloud);
		}
		else {
			CloudId = QString::fromStdString(instance.name) + QString::number(index) + "result" + "0";
			menber->resultPCID.insert(std::make_pair(QString::fromStdString(instance.name), std::vector<QString>{CloudId}));
			menber->resultPointCloud.insert(std::make_pair(QString::fromStdString(instance.name), std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>{resultPointCloud}));
		}
		menber->viewer->addPointCloud(resultPointCloud, currentColor, CloudId.toStdString());

		adjustPointCloudTransform();
	}
}

void te::ThreeDView::clearMarkerPointCloud()
{
	for (const auto& pair : menber->markerPCID) {
		for (const QString& id : pair.second) {
			menber->viewer->removePointCloud(id.toStdString());
		}
	}
	menber->renderWindow->Render();
}

void te::ThreeDView::clearResultPointCloud()
{
	for (const auto& pair : menber->resultPCID) {
		for (const QString& id : pair.second) {
			menber->viewer->removePointCloud(id.toStdString());
		}
	}
	menber->renderWindow->Render();
}

void te::ThreeDView::clearMarkerVector()
{
	menber->markerPCID.clear();
	menber->markerPointCloud.clear();
}

void te::ThreeDView::clearResultVector()
{
	menber->resultPCID.clear();
	menber->resultPointCloud.clear();
}

void te::ThreeDView::crossSection()
{
	if (menber->cloud->empty()) {
		return;
	}
	else {
		//if (data1.isEmpty() || data2.isEmpty() || data3.isEmpty() || data4.isEmpty()) {
		//	return;
		//}
		//pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Filter_out = (new pcl::PointCloud<pcl::PointXYZRGB>())->makeShared();
		//pcl_crossSection(menber->cloud, cloud_Filter_out, data1.toFloat(), data2.toFloat(), data3, data4.toFloat());
		//pcl::copyPointCloud(*cloud_Filter_out, *menber->cloud);

		//viewer->removeAllPointClouds();
		//viewer->removeAllShapes();
		//viewer->addPointCloud(cloud, "cloud");
		//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
		//m_renderWindow->Render();
	}
}

void te::ThreeDView::setPointCloudHeight()
{
	double factor = viewModel.lock()->getPointCloudHeight();
	if (factor != 1 && !menber->cloud->empty())
	{
		for (auto& point : menber->cloud->points) {
			point.z = point.z * factor;
		}
	}
	RerenderWindow();
}

void te::ThreeDView::setBackgroundColor()
{
	QColor color = viewModel.lock()->getPointCloudBackgroundColor();
	menber->viewer->setBackgroundColor(color.redF(), color.greenF(), color.blueF());
	menber->renderWindow->Render();
}

void te::ThreeDView::setPointCloudColor()
{
	QColor color = viewModel.lock()->getPointCloudColor();
	QColor temp;
	temp.setRgb(143, 153, 159, 255);
	if (!menber->cloud->empty() && (color != temp)) {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>selected_color(menber->cloud, color.redF() * 255, color.greenF() * 255, color.blueF() * 255);
		menber->viewer->updatePointCloud(menber->cloud, selected_color, "cloud");
		menber->renderWindow->Render();
	}
	else {
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB>selected_color(menber->cloud, 255, 255, 255);
		menber->viewer->updatePointCloud(menber->cloud, selected_color, "cloud");
		menber->renderWindow->Render();
	}
	menber->renderWindow->Render();
}

void te::ThreeDView::setAxisRender()
{
	if (menber->cloud->empty()) {
		return;
	}

	menber->cloud->clear();
	pcl::copyPointCloud(*viewModel.lock()->coordinateAxisRendering(menber->cloud), *menber->cloud);
	menber->viewer->updatePointCloud(menber->cloud, "cloud");
	adjustCamera();
	menber->renderWindow->Render();
}

void te::ThreeDView::setPointSize()
{
	int size = viewModel.lock()->getPointCloudPointSize();
	menber->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "cloud");
	menber->renderWindow->Render();
}

void te::ThreeDView::setAxisAlignedBoundingBox()
{

}

void te::ThreeDView::setOrientedBoundingBox()
{

}

void te::ThreeDView::saveSegementPara()
{
	//double* FBRange = menber->renderer->GetActiveCamera()->GetClippingRange();

	//vtkMatrix4x4* mat = menber->renderer->GetActiveCamera()->GetCompositeProjectionTransformMatrix(menber->renderer->GetTiledAspectRatio(), FBRange[0], FBRange[1]);
	//vtkMatrix4x4* transmat = menber->customInteractor->getRotationTransform()->GetMatrix();

	//viewModel.lock()->setCompositeProjectionTransform(mat);
	//viewModel.lock()->setThreeDTransmat(transmat);
	viewModel.lock()->setPointCloudToSegmented(menber->cloud);
}

std::vector<double> te::ThreeDView::getCloudCentroid()
{
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*menber->cloud, centroid);
	return { centroid.x(), centroid.y(), centroid.z() };
}

void te::ThreeDView::setInteractorCallback(ThreeDView& ptr)
{
	menber->customInteractor->setCallback
	(
		[&ptr]()
		{
			ptr.outOfBounds();
		}
	);
}

void ThreeDView::refresh(ViewModel::updateMode mode)
{
	switch (mode)
	{
	case ViewModel::InitPointCloud:
		initPointCloud();
		setRotationCenter();
		setCoordinateInfo();
		break;
	case ViewModel::SetCameraPosition:
		setCameraPosition();
		break;
	case ViewModel::Segement:
		saveSegementPara();
		break;
	case ViewModel::AutoAdjustCamera:
		adjustCamera();
		break;
	case ViewModel::MarkerPointCloudVisible:
		setMarkerPointCloudVisible();
		break;
	case ViewModel::ResultPointCloudVisible:
		setResultPointCloudVisible();
		break;
	case ViewModel::UpdateMarkerPointCloud:
		updateMarkerPointCloud();
		break;
	case ViewModel::UpdateResultPointCloud:
		updateResultPointCloud();
		break;
	case ViewModel::AddMarkerPointCloud:
		addMarkerPointCloud();
		break;
	case ViewModel::ShowMarkerPointCloud:
		showMarkerPointCloud();
		break;
	case ViewModel::ShowResultPointCloud:
		showResultPointCloud();
		break;
	case ViewModel::CrossSection:
		crossSection();
		break;
	case ViewModel::PointCloudHeight:
		setPointCloudHeight();
		break;
	case ViewModel::BackgroundColor:
		setBackgroundColor();
		break;
	case ViewModel::AxisRender:
		setAxisRender();
		break;
	case ViewModel::PointCloudColor:
		setPointCloudColor();
		break;
	case ViewModel::PointCloudPointSize:
		setPointSize();
		break;
	case ViewModel::AABB:
		setAxisAlignedBoundingBox();
		break;
	case ViewModel::OBB:
		setOrientedBoundingBox();
		break;
	default:
		break;
	}

}

void ThreeDView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &ThreeDView::refresh);
	}
}
