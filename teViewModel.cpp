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

void te::ViewModel::prepareTrain(std::string path)
{
	model->setmodelpath(path);
	te::TrainParaRegister* train = getTrainPara();
	model->initTrainConfig(train);
	model->trainModel();
}

void te::ViewModel::prepareTest(std::string path)
{
	model->setmodelpath(path);
	te::TestParaRegister* test = getTestPara();
	model->initTestConfig(test);
	model->testModel();
}

void te::ViewModel::stopTrain()
{
	model->stopTrain();
}

int ViewModel::getCurrentIndex()
{
	int index = 0;
	if (model->getData<int>("currentIndex", index))
	{
		return index;
	}
	return -1;
}

void ViewModel::setCurrentIndex(int index)
{
	model->setData("currentIndex", index);
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

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ViewModel::getSegmentPointCloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	if (model->getData("SegmentPointCloud", cloud))
		return cloud;
	else
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

void ViewModel::setSegmentPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->setData("SegmentPointCloud", cloud);
}

double te::ViewModel::getPointCloudHeight()
{
	double factor = 1;
	if (model->getData("PointCloudHeight", factor))
		return factor;
	else
		return 1;
}

void te::ViewModel::setPointCloudHeight(double factor)
{
	model->setData("PointCloudHeight", factor);
}

QColor te::ViewModel::getPointCloudBackgroundColor()
{
	QColor color = Qt::black;
	if (model->getData("PointCloudBackgroundColor", color))
		return color;
	else
		return QColor();
}

void te::ViewModel::setPointCloudBackgroundColor(QColor color)
{
	model->setData("PointCloudBackgroundColor", color);
}

QColor te::ViewModel::getPointCloudColor()
{
	QColor color = Qt::black;
	if (model->getData("PointCloudColor", color))
		return color;
	else
		return QColor();
}

void te::ViewModel::setPointCloudColor(QColor color)
{
	model->setData("PointCloudColor", color);
}

int te::ViewModel::getPointCloudPointSize()
{
	int size = 0;
	if (model->getData("PointCloudPointSize", size))
		return size;
	else
		return 1;
}

void te::ViewModel::setPointCloudPointSize(int size)
{
	model->setData("PointCloudPointSize", size);
}

QString te::ViewModel::getRenderAxis()
{
	QString str = "";
	if (model->getData("RenderAxis", str))
		return str;
	else
		return "z";
}

void te::ViewModel::setRenderAxis(QString str)
{
	model->setData("RenderAxis", str);
}

enum ViewModel::TypeWidget te::ViewModel::getCurrentWidgetType()
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

void te::ViewModel::setCurrentWidgetType(TypeWidget temp)
{
	int pattern = temp;
	model->setData("CurrentWidgetType", pattern);
}

float te::ViewModel::getReceptiveField()
{
	float factor = 0;
	if (model->getData("ReceptiveField", factor))
		return factor;
	else
		return 1.0;
}

void te::ViewModel::setReceptiveField(float factor)
{
	model->setData("ReceptiveField", factor);
}

TestParaRegister* te::ViewModel::getTestPara()
{
	TestParaRegister* para = nullptr;
	if (model->getData("TestPara", para))
		return para;
	else
		return para;
}

void te::ViewModel::setTestPara(TestParaRegister* para)
{
	model->setData("TestPara", para);
}

TrainParaRegister* te::ViewModel::getTrainPara()
{
	TrainParaRegister* para = nullptr;
	if (model->getData("TrainPara", para))
		return para;
	else
		return para;
}

void te::ViewModel::setTrainPara(TrainParaRegister* para)
{
	model->setData("TrainPara", para);
}

int te::ViewModel::getTrainStatisticsState()
{
	int arg = 0;
	if (model->getData("TrainStatisticsState", arg))
		return arg;
	else
		return arg;
}

void te::ViewModel::setTrainStatisticsState(int arg)
{
	model->setData("TrainStatisticsState", arg);
}

TrainStatisticsViewMenber te::ViewModel::getTrainStateMenber()
{
	TrainStatisticsViewMenber arg;
	arg.iteration = 0.0; arg.fAvgLoss = 0.0; arg.fPosAcc = 0.0;
	if (model->getData("TrainStateMenber", arg))
		return arg;
	else
		return arg;
}

void te::ViewModel::setTrainStateMenber(TrainStatisticsViewMenber arg)
{
	model->setData("TrainStateMenber", arg);
}

void te::ViewModel::setTrainStatisticsCallback(ViewModel& ptr)
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
	return model->getCurrentLabelInfo();
}

void ViewModel::changeLabelInfo(const QString& category, const QColor& color)
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

void ViewModel::setThrDMarkPointList(QVector<QPointF> vec)
{
	model->setThrDMarkPointList(vec);
	emit notified(ViewModel::AddMarkerPointCloud);
}

void ViewModel::setThreshold(std::pair<double, double> temp)
{
	model->updateInvalidPointThreshold(temp.first);
	model->updateValidPointThreshold(temp.second);
}

void ViewModel::add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	model->add3DAiInstance(cloud);
	notified(ViewModel::AddMarkerPointCloud);
}

void ViewModel::add2DAiInstance(QList<GraphicsItem*> polygonItems)
{
	model->add2DAiInstance(polygonItems);
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

double te::ViewModel::getCurrentInvalidPointThreshold()
{
	return model->getCurrentInvalidPointThreshold();
}

double te::ViewModel::getCurrentValidPointThreshold()
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

SampleMark te::ViewModel::getSelectTrainSampleInfo(int index)
{
	return model->getSelectTrainSampleInfo(index);
}

SampleMark te::ViewModel::getSelectResultSampleInfo(int index)
{
	return model->getSelectResultSampleInfo(index);
}

SampleMark te::ViewModel::getCurrentTrainSampleInfo()
{
	return model->getCurrentTrainSampleInfo();
}

SampleMark te::ViewModel::getCurrentResultSampleInfo()
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

void ViewModel::segment(double* clipRange, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, vtkMatrix4x4* transmat)
{
	model->segment(clipRange, cloudin, transmat);
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