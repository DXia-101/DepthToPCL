#include "teModel.h"

#include "Depth2RGB.h"
using namespace te;

Model::Model()
{
}

Model::~Model()
{
}

void Model::trainModel()
{
}

void Model::testModel()
{
}

void Model::initTrainConfig(TrainParaRegister* para)
{
}

void Model::initTestConfig(TestParaRegister* para)
{
}

void Model::modelpathSettings(const char* modelpath)
{
}

Training Model::getTrainHandle()
{
	return Training();
}

int Model::teException(void* pParam, AiStatus eStatus)
{
	return 0;
}

void Model::teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param)
{
}

void Model::teAiInferResult(AiResult& inferResult, DynamicMatrix& hotmap, void* pCallbackParam)
{
}

bool Model::isImageAlreadyExist(std::string filepath)
{
	return false;
}

std::string Model::getSelectShrinkageChart(int index)
{
	return std::string();
}

std::string Model::getCurrentShrinkageChart()
{
	return std::string();
}

std::string Model::getSelectOriginImage(int index)
{
	return std::string();
}

std::string Model::getCurrentOriginImage()
{
	return std::string();
}

std::string Model::getSelectPointCloud(int index)
{
	return std::string();
}

std::string Model::getCurrentPointCloud()
{
	return std::string();
}

std::string Model::getSelectTrainGt(int index)
{
	return std::string();
}

std::string Model::getSelectResultGt(int index)
{
	return std::string();
}

SampleMark Model::getSelectTrainSampleInfo(int index)
{
	return SampleMark();
}

SampleMark Model::getSelectResultSampleInfo(int index)
{
	return SampleMark();
}

SampleMark Model::getCurrentTrainSampleInfo()
{
	return SampleMark();
}

SampleMark Model::getCurrentResultSampleInfo()
{
	return SampleMark();
}

QMap<QString, int> Model::getCurrentTrainMarksNumber()
{
	return QMap<QString, int>();
}

QMap<QString, int> Model::getCurrentResultMarksNumber()
{
	return QMap<QString, int>();
}

bool Model::updateTrainSampleMark(int index, SampleMark& samplemark)
{
	return false;
}

bool Model::updateResultSampleMark(int index, SampleMark& samplemark)
{
	return false;
}

bool Model::updateCurrentTrainSampleMark(SampleMark& samplemark)
{
	return false;
}

bool Model::updateCurrentResultSampleMark(SampleMark& samplemark)
{
	return false;
}

bool Model::updateShrinkageChart(int index, std::string& filepath)
{
	return false;
}

bool Model::updatePointCloud(int index, std::string& filepath)
{
	return false;
}

bool Model::updateTrainGtFilePath(int index, std::string& filepath)
{
	return false;
}

bool Model::updateResultGtFilePath(int index, std::string& filepath)
{
	return false;
}

bool Model::clearCurrentTrainSampleMark()
{
	return false;
}

bool Model::clearAllTestSampleMark()
{
	return false;
}

bool Model::clearAllPointCloud()
{
	return false;
}

bool Model::clearAllShrinkageChart()
{
	return false;
}

void Model::DropAllTables()
{
}

void Model::getTrainSamples(std::vector<SampleInfo>* trainSamples)
{
}

void Model::getResultSamples(std::vector<SampleInfo>* resultSamples)
{
}

void Model::initThreasholds(int size)
{
}

void Model::updateInvalidPointThreshold(double threshold)
{
}

double Model::getSelectInvalidPointThreshold(int index)
{
	return 0.0;
}

double Model::getCurrentInvalidPointThreshold()
{
	return 0.0;
}

void Model::addInvalidPointThreshold(int index, double threshold)
{
}

void Model::updateValidPointThreshold(double threshold)
{
}

double Model::getSelectValidPointThreshold(int index)
{
	return 0.0;
}

double Model::getCurrentValidPointThreshold()
{
	return 0.0;
}

void Model::addValidPointThreshold(int index, double threshold)
{
}

void Model::loadTrainingImages(const QStringList& filePaths)
{
}

void Model::setCallback(CallbackFunction callback)
{
}

void Model::triggerCallback(int iteration, float fAvgLoss, float fPosAcc)
{
}

bool Model::savePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud)
{
	return false;
}

void Model::segment(double* clipRange, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudin, vtkMatrix4x4* transmat)
{
	return;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Model::coordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_Elevation_rendering(new pcl::PointCloud<pcl::PointXYZRGB>);


	std::shared_ptr<QString> renderAxis;
	getData("RenderAxis", renderAxis);

	double min, max;
	if (*renderAxis == "z")
	{
		min = getCurrentInvalidPointThreshold();
		max = getCurrentInvalidPointThreshold();
		int factor = max - min;
		if (factor >= 1 && factor < 255) {
			factor = 255 / factor;
		}
		else {
			factor = 1;
		}
		max *= factor;
		min *= factor;
	}
	else if (*renderAxis == "x")
	{
		std::shared_ptr<pcl::PointXYZ> maxPt;
		getData("maxPt", maxPt);
		std::shared_ptr<pcl::PointXYZ> minPt;
		getData("minPt", minPt);
		min = minPt->x;
		max = maxPt->x;
	}
	else if (*renderAxis == "y")
	{
		std::shared_ptr<pcl::PointXYZ> maxPt;
		getData("maxPt", maxPt);
		std::shared_ptr<pcl::PointXYZ> minPt;
		getData("minPt", minPt);
		min = minPt->y;
		max = maxPt->y;
	}

	TeJetColorCode trans;
	for (int index = 0; index < cloud->points.size(); ++index)
	{
		double value = 0;
		if (*renderAxis == "z")
		{
			value = cloud->points[index].z;
		}
		else if (*renderAxis == "x")
		{
			value = cloud->points[index].x;
		}
		else if (*renderAxis == "y")
		{
			value = cloud->points[index].y;
		}

		pcl::PointXYZRGB point = cloud->points[index];

		float absDepth = value > min ? value : min;
		absDepth = absDepth < max ? absDepth : max;
		float realDepth = max == min ? 0 : ((absDepth - min) / (max - min));
		int iIndex = 1023 * realDepth;//将[0-1.0]之间的数据映射到[0-1024)之间
		point.r = trans.m_pJetTab1024[iIndex].r;
		point.g = trans.m_pJetTab1024[iIndex].g;
		point.b = trans.m_pJetTab1024[iIndex].b;

		cloud_Elevation_rendering->push_back(point);
	}
	return cloud_Elevation_rendering;
}

void Model::extractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::worldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* mat, double* output2D)
{
}

void Model::worldToScreen(int* windowsize, double* viewport, pcl::PointXYZRGB* input3D, vtkMatrix4x4* transform, vtkMatrix4x4* composit, double* output2D)
{
}

void Model::axisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::orientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::subtractTargetPointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
{
}

void Model::perspectiveToAxis(pcl::PointXYZ maxPt, pcl::PointXYZ minPt, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::pcl_crossSection(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out)
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr te::Model::getPointCloudByContour(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, AiInstance*)
{
	return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

void Model::add3DAiInstance(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
}

void Model::add2DAiInstance(QList<te::GraphicsItem*> polygonItems)
{
}

void Model::addLabelInfo(const QString&, const QColor&)
{
}

void Model::changeLabelInfo(const QString&, const QColor&)
{
}

void Model::removeLabelInfo(const QString&)
{
}

std::pair<QString, QColor> Model::getCurrentLabelInfo()
{
	return std::pair<QString, QColor>();
}

QColor Model::getLabelColor(const QString&)
{
	return QColor();
}

void Model::setThrDMarkPointList(QVector<QPointF>)
{
}

QVector<QPointF> Model::getThrDMarkPointList()
{
	return QVector<QPointF>();
}

void te::Model::SetCallback(CallbackFunction callback)
{
}

void te::Model::TriggerCallback(int iteration, float fAvgLoss, float fPosAcc)
{
}
