#include "teViewModel.h"
#include "teModel.h"
#include <QColor>

using namespace te;

ViewModel::ViewModel(QObject* parent)
	: QObject(parent)
{
}

ViewModel::~ViewModel()
{}

void ViewModel::setModel(Model* ptr)
{
	model = ptr;
	setTrainStatisticsCallback(*this);
}

void ViewModel::loadTrainingImages(const QStringList& filePaths)
{
	model->loadTrainingImages(filePaths);
}

void ViewModel::prepareTrain(std::string path)
{
	model->setmodelpath(path);
	te::TrainParaRegister train = getTrainPara();
	model->initTrainConfig(&train);
	model->trainModel();
}

void ViewModel::prepareTest(std::string path)
{
	model->setmodelpath(path);
	te::TestParaRegister test = getTestPara();
	model->initTestConfig(&test);
	model->testModel();
}

void ViewModel::stopTrain()
{
	model->stopTrain();
}

int ViewModel::getCurrentIndex()
{
	int index = 0;
	if (model->getData("currentIndex", index))
	{
		return index;
	}
	return -1;
}

void ViewModel::setCurrentIndex(int index)
{
	model->setData("currentIndex", index);
}

QString ViewModel::getCurrentLabel()
{
	return model->getCurrentLabel();
}

void ViewModel::setCurrentLabel(QString str)
{
	model->setCurrentLabel(str);
}

void te::ViewModel::removeLabelInfo(QString category)
{
	model->removeLabelInfo(category);
}

int ViewModel::getCurrentDisplayImageLength()
{
	int length = 1;
	if (model->getData("currentDisplayImageLength", length))
		return length;
	else
		return 0;
}

void ViewModel::setCurrentDisplayImageLength(int length)
{
	model->setData("currentDisplayImageLength", length);
}

int ViewModel::getCurrentDisplayImageHeight()
{
	int height = 1;
	if (model->getData("currentDisplayImageHeight", height))
		return height;
	else
		return 0;
}

void ViewModel::setCurrentDisplayImageHeight(int height)
{
	model->setData("currentDisplayImageHeight", height);
}

pcl::PointXYZ ViewModel::getCurrentMaxPt()
{
	pcl::PointXYZ maxPt = pcl::PointXYZ(0, 0, 0);
	if (model->getData("maxPt", maxPt))
		return maxPt;
	else
		return pcl::PointXYZ();
}

void ViewModel::setCurrentMaxPt(pcl::PointXYZ maxPt)
{
	model->setData("maxPt", maxPt);
}

pcl::PointXYZ ViewModel::getCurrentMinPt()
{
	pcl::PointXYZ minPt = pcl::PointXYZ(0, 0, 0);
	if (model->getData("minPt", minPt))
		return minPt;
	else
		return pcl::PointXYZ();
}

void ViewModel::setCurrentMinPt(pcl::PointXYZ minPt)
{
	model->setData("minPt", minPt);
}

CoordinateAxisDirection ViewModel::getCoordinateAxisDire()
{
	int direction = PositiveXaxis;
	CoordinateAxisDirection temp;
	if (model->getData("CoordinateAxisDirection", direction))
		temp = static_cast<CoordinateAxisDirection>(direction);
	else
		temp = CoordinateAxisDirection::NegativeXaxis;
	return temp;
}

void ViewModel::setCoordinateAxisDire(CoordinateAxisDirection temp)
{
	int dire = temp;
	model->setData("CoordinateAxisDirection", dire);
}

int ViewModel::getThreeDisShowMarkers()
{
	int arg = 1;
	if (model->getData("isShowThreeDMarkers", arg))
		return arg;
	else
		return -1;
}

void ViewModel::setThreeDisShowMarkers(int arg)
{
	model->setData("isShowThreeDMarkers", arg);
}

int ViewModel::getThreeDisShowResults()
{
	int arg = 1;
	if (model->getData("isShowThreeDResults", arg))
		return arg;
	else
		return -1;
}

void ViewModel::setThreeDisShowResults(int arg)
{
	model->setData("isShowThreeDResults", arg);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ViewModel::getPointCloudToSegmented()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	if (model->getData("PointCloudToSegmented", cloud))
		return cloud;
	else
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

void ViewModel::setPointCloudToSegmented(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->setData("PointCloudToSegmented", cloud);
}

double ViewModel::getPointCloudHeight()
{
	double factor = 1;
	if (model->getData("PointCloudHeight", factor))
		return factor;
	else
		return 1;
}

void ViewModel::setPointCloudHeight(double factor)
{
	model->setData("PointCloudHeight", factor);
}

QColor ViewModel::getPointCloudBackgroundColor()
{
	QColor color = Qt::black;
	if (model->getData("PointCloudBackgroundColor", color))
		return color;
	else
		return QColor();
}

void ViewModel::setPointCloudBackgroundColor(QColor color)
{
	model->setData("PointCloudBackgroundColor", color);
}

QColor ViewModel::getPointCloudColor()
{
	QColor color = Qt::black;
	if (model->getData("PointCloudColor", color))
		return color;
	else
		return QColor();
}

void ViewModel::setPointCloudColor(QColor color)
{
	model->setData("PointCloudColor", color);
}

int ViewModel::getPointCloudPointSize()
{
	int size = 0;
	if (model->getData("PointCloudPointSize", size))
		return size;
	else
		return 1;
}

void ViewModel::setPointCloudPointSize(int size)
{
	model->setData("PointCloudPointSize", size);
}

QString ViewModel::getRenderAxis()
{
	QString str = "";
	if (model->getData("RenderAxis", str))
		return str;
	else
		return "z";
}

void ViewModel::setRenderAxis(QString str)
{
	model->setData("RenderAxis", str);
}

enum ViewModel::TypeWidget ViewModel::getCurrentWidgetType()
{
	int pattern = 0;
	ViewModel::TypeWidget temp;
	if (model->getData("CurrentWidgetType", pattern))
	{
		temp = static_cast<ViewModel::TypeWidget>(pattern);
	}
	else
	{
		temp = ViewModel::TypeWidget::ReceptiveFieldView;
	}
	return temp;
}

void ViewModel::setCurrentWidgetType(TypeWidget temp)
{
	int pattern = temp;
	model->setData("CurrentWidgetType", pattern);
}

float ViewModel::getReceptiveField()
{
	float factor = 0;
	if (model->getData("ReceptiveField", factor))
		return factor;
	else
		return 1.0;
}

void ViewModel::setReceptiveField(float factor)
{
	model->setData("ReceptiveField", factor);
}

TestParaRegister ViewModel::getTestPara()
{
	TestParaRegister para;
	if (model->getData("TestPara", para))
		return para;
	else
		return para;
}

void ViewModel::setTestPara(TestParaRegister para)
{
	model->setData("TestPara", para);
}

TrainParaRegister ViewModel::getTrainPara()
{
	TrainParaRegister para;
	if (model->getData("TrainPara", para))
		return para;
	else
		return para;
}

void ViewModel::setTrainPara(TrainParaRegister para)
{
	model->setData("TrainPara", para);
}

int ViewModel::getTrainStatisticsState()
{
	int arg = 0;
	if (model->getData("TrainStatisticsState", arg))
		return arg;
	else
		return arg;
}

void ViewModel::setTrainStatisticsState(int arg)
{
	model->setData("TrainStatisticsState", arg);
}

TrainStatisticsViewMenber ViewModel::getTrainStateMenber()
{
	TrainStatisticsViewMenber arg;
	arg.iteration = 0.0; arg.fAvgLoss = 0.0; arg.fPosAcc = 0.0;
	if (model->getData("TrainStateMenber", arg))
		return arg;
	else
		return arg;
}

void ViewModel::setTrainStateMenber(TrainStatisticsViewMenber arg)
{
	model->setData("TrainStateMenber", arg);
}

QVector<QPointF> ViewModel::getThreeDMarkerPolygon()
{
	return model->getThreeDMarkerPolygon();
}

void ViewModel::setThreeDMarkerPolygon(QVector<QPointF> vec)
{
	model->setThreeDMarkerPolygon(vec);
}

int* ViewModel::getVtkWindowSize()
{
	return model->getVtkWindowSize();
}

void ViewModel::setVtkWindowSize(int* size)
{
	model->setVtkWindowSize(size);
}

double* ViewModel::getRenderViewport()
{
	return model->getRenderViewport();
}

void ViewModel::setRenderViewport(double* viewport)
{
	model->setRenderViewport(viewport);
}

vtkMatrix4x4* ViewModel::getCompositeProjectionTransform()
{
	vtkMatrix4x4* mat;
	if (model->getData("CompositeProjectionTransform", mat))
		return mat;
	else
		return mat;
}

void ViewModel::setCompositeProjectionTransform(vtkMatrix4x4* mat)
{
	model->setData("CompositeProjectionTransform", mat);
}

vtkMatrix4x4* ViewModel::getThreeDTransmat()
{
	vtkMatrix4x4* mat;
	if (model->getData("ThreeDTransmat", mat))
		return mat;
	else
		return mat;
}

void ViewModel::setThreeDTransmat(vtkMatrix4x4* mat)
{
	model->setData("ThreeDTransmat", mat);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ViewModel::getSegmentedPointCloud()
{
	return model->getSegmentedPointCloud();
}

void ViewModel::setSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->setSegmentedPointCloud(cloud);
}

void ViewModel::setTrainStatisticsCallback(ViewModel& ptr)
{
	model->SetCallback
	(
		[&ptr](int iteration, float fAvgLoss, float fPosAcc)
		{
			TrainStatisticsViewMenber menber;
			menber.iteration = iteration;
			menber.fAvgLoss = fAvgLoss;
			menber.fPosAcc = fPosAcc;
			ptr.setTrainStateMenber(menber);
		}
	);
}

QColor ViewModel::getLabelColor(const QString& category)
{
	return model->getLabelColor(category);
}

std::pair<QString, QColor> ViewModel::getCurrentLabelInfo()
{
	QString category = getCurrentLabel();
	QColor color = getLabelColor(category);
	return std::make_pair(category, color);
}

void ViewModel::changeCurrentLabelInfo(const QString& category, const QColor& color)
{
	model->changeLabelInfo(category, color);
	notified(UpdateMarkerPointCloud);
	notified(UpdateResultPointCloud);
}

void ViewModel::addLabelInfo(const QString& category, const QColor& color)
{
	model->addLabelInfo(category, color);
}

QMap<QString, int> ViewModel::getTrainMarkerCount()
{
	return model->getCurrentTrainMarksNumber();
}

QMap<QString, int> ViewModel::getTestResultCount()
{
	return model->getCurrentResultMarksNumber();
}

void ViewModel::setThreshold(std::pair<double, double> temp)
{
	model->updateInvalidPointThreshold(temp.first);
	model->updateValidPointThreshold(temp.second);
}

void ViewModel::add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->add3DAiInstance(cloud);
	notified(ViewModel::updateTrainCount);
	notified(ViewModel::AddMarkerPointCloud);
}

void ViewModel::add2DAiInstance(QList<GraphicsItem*> polygonItems)
{
	model->add2DAiInstance(polygonItems);
	notified(ViewModel::updateTrainCount);
	notified(ViewModel::AddMarkerPointCloud);
}

void ViewModel::trainModel()
{
	model->trainModel();
}

void ViewModel::testModel()
{
	model->testModel();
}

void ViewModel::initTrainConfig(TrainParaRegister* para)
{
	model->initTrainConfig(para);
}

void ViewModel::initTestConfig(TestParaRegister* para)
{
	model->initTestConfig(para);
}

double ViewModel::getCurrentInvalidPointThreshold()
{
	return model->getCurrentInvalidPointThreshold();
}

double ViewModel::getCurrentValidPointThreshold()
{
	return model->getCurrentValidPointThreshold();
}

std::string ViewModel::getSelectShrinkageChart(int index)
{
	return model->getSelectShrinkageChart(index);
}

std::string ViewModel::getCurrentShrinkageChart()
{
	return model->getCurrentShrinkageChart();
}

std::string ViewModel::getSelectOriginImage(int index)
{
	return model->getSelectOriginImage(index);
}

std::string ViewModel::getCurrentOriginImage()
{
	return model->getCurrentOriginImage();
}

std::string ViewModel::getSelectPointCloud(int index)
{
	return model->getSelectPointCloud(index);
}

std::string ViewModel::getCurrentPointCloud()
{
	return model->getCurrentPointCloud();
}

std::string ViewModel::getSelectTrainGt(int index)
{
	return model->getSelectTrainGt(index);
}

std::string ViewModel::getSelectResultGt(int index)
{
	return model->getSelectResultGt(index);
}

SampleMark ViewModel::getSelectTrainSampleInfo(int index)
{
	return model->getSelectTrainSampleInfo(index);
}

SampleMark ViewModel::getSelectResultSampleInfo(int index)
{
	return model->getSelectResultSampleInfo(index);
}

SampleMark ViewModel::getCurrentTrainSampleInfo()
{
	return model->getCurrentTrainSampleInfo();
}

SampleMark ViewModel::getCurrentResultSampleInfo()
{
	return model->getCurrentResultSampleInfo();
}

bool ViewModel::updateShrinkageChart(int index, std::string& filepath)
{
	return model->updateShrinkageChart(index, filepath);
}

bool ViewModel::updatePointCloud(int index, std::string& filepath)
{
	return model->updatePointCloud(index, filepath);
}

bool ViewModel::updateTrainGtFilePath(int index, std::string& filepath)
{
	return model->updateTrainGtFilePath(index, filepath);
}

bool ViewModel::updateResultGtFilePath(int index, std::string& filepath)
{
	return model->updateResultGtFilePath(index, filepath);
}

bool ViewModel::clearCurrentTrainSampleMark()
{
	return model->clearCurrentTrainSampleMark();
}

bool ViewModel::clearAllTestSampleMark()
{
	return model->clearAllTestSampleMark();
}

bool ViewModel::clearAllPointCloud()
{
	return model->clearAllPointCloud();
}

bool ViewModel::clearAllShrinkageChart()
{
	return model->clearAllShrinkageChart();
}

void ViewModel::DropAllTables()
{
	model->DropAllTables();
}

void ViewModel::getTrainSamples(std::vector<SampleInfo>* trainSamples)
{
	model->getTrainSamples(trainSamples);
}

void ViewModel::getResultSamples(std::vector<SampleInfo>* resultSamples)
{
	model->getResultSamples(resultSamples);
}

void ViewModel::segment()
{
	vtkMatrix4x4* mat = getCompositeProjectionTransform();
	vtkMatrix4x4* transmat = getThreeDTransmat();
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = getPointCloudToSegmented();
	model->segment(cloud, mat, transmat);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ViewModel::coordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return model->coordinateAxisRendering(cloud);
}

void ViewModel::extractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->extractingPointCloudsBasedOnContours(cloud);
}

void ViewModel::axisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->axisAlignedBoundingBox(cloud);
}

void ViewModel::orientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->orientedBoundingBox(cloud);
}

void ViewModel::subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
	model->subtractTargetPointcloud(cloud1, cloud2);
}

void ViewModel::perspectiveToAxis(pcl::PointXYZ maxPt, pcl::PointXYZ minPt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->perspectiveToAxis(maxPt, minPt, cloud);
}

void ViewModel::pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
	model->pcl_crossSection(cloud_in, cloud_out);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ViewModel::getPointCloudByContour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, AiInstance* inst)
{
	return model->getPointCloudByContour(cloud, inst);
}