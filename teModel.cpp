#include "teModel.h"

#include "Depth2RGB.h"
#include "IDataStore.h"
using namespace te;

Model::Model(std::unique_ptr<IDataStore> dataStore)
	:m_dataStore(std::move(dataStore))
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
	return m_dataStore->isImageAlreadyExist(filepath);
}

std::string Model::getSelectShrinkageChart(int index)
{
	return m_dataStore->getSelectResultFormResourceTable(index, "ShrinkageChartPath");
}

std::string Model::getCurrentShrinkageChart()
{
	return m_dataStore->getSelectResultFormResourceTable(getCurrentIndex(), "ShrinkageChartPath");
}

std::string Model::getSelectOriginImage(int index)
{
	return m_dataStore->getSelectResultFormResourceTable(index, "OriginImagePath");
}

std::string Model::getCurrentOriginImage()
{
	return m_dataStore->getSelectResultFormResourceTable(getCurrentIndex(), "OriginImagePath");
}

std::string Model::getSelectPointCloud(int index)
{
	return m_dataStore->getSelectResultFormResourceTable(index, "PointCloudPath");
}

std::string Model::getCurrentPointCloud()
{
	return m_dataStore->getSelectResultFormResourceTable(getCurrentIndex(), "PointCloudPath");
}

std::string Model::getSelectTrainGt(int index)
{
	return m_dataStore->getSelectResultFormGtTable(index).first;
}

std::string Model::getSelectResultGt(int index)
{
	return m_dataStore->getSelectResultFormGtTable(index).second;
}

SampleMark Model::getSelectTrainSampleInfo(int index)
{
	return m_dataStore->getSelectResultFormSampleMarkTable(index, "TrainSampleMark");
}

SampleMark Model::getSelectResultSampleInfo(int index)
{
	return m_dataStore->getSelectResultFormSampleMarkTable(index, "ResultSampleMark");
}

SampleMark Model::getCurrentTrainSampleInfo()
{
	return m_dataStore->getSelectResultFormSampleMarkTable(getCurrentIndex(), "TrainSampleMark");
}

SampleMark Model::getCurrentResultSampleInfo()
{
	return m_dataStore->getSelectResultFormSampleMarkTable(getCurrentIndex(), "ResultSampleMark");
}

QMap<QString, int> Model::getCurrentTrainMarksNumber()
{
	te::SampleMark temp = getCurrentTrainSampleInfo();
	QMap<QString, int> nameCounts;

	for (const te::AiInstance& instance : temp.gtDataSet)
	{
		QString name = QString::fromStdString(instance.name);
		nameCounts[name]++;
	}

	return nameCounts;
}

QMap<QString, int> Model::getCurrentResultMarksNumber()
{
	te::SampleMark temp = getCurrentResultSampleInfo();
	QMap<QString, int> nameCounts;

	for (const te::AiInstance& instance : temp.gtDataSet)
	{
		QString name = QString::fromStdString(instance.name);
		nameCounts[name]++;
	}

	return nameCounts;
}

bool Model::updateTrainSampleMark(int index, SampleMark& samplemark)
{
	return m_dataStore->updateTrainSampleMark(index, samplemark);
}

bool Model::updateResultSampleMark(int index, SampleMark& samplemark)
{
	return m_dataStore->updateResultSampleMark(index, samplemark);
}

bool Model::updateCurrentTrainSampleMark(SampleMark& samplemark)
{
	return m_dataStore->updateTrainSampleMark(getCurrentIndex(), samplemark);
}

bool Model::updateCurrentResultSampleMark(SampleMark& samplemark)
{
	return m_dataStore->updateResultSampleMark(getCurrentIndex(), samplemark);
}

int te::Model::getCurrentIndex()
{
	int index = 0;
	if (getData<int>("currentIndex", index))
	{
		return index;
	}
	return index;
}

int te::Model::getCurrentLoadImageNum()
{
	int index = 0;
	if (getData<int>("LoadImageNum", index))
	{
		return index;
	}
	return index;
}

void te::Model::setCurrentLoadImageNum(int num)
{
	setData("LoadImageNum", num);
}

bool Model::updateShrinkageChart(int index, std::string& filepath)
{
	return m_dataStore->updateShrinkageChart(index, filepath);
}

bool Model::updatePointCloud(int index, std::string& filepath)
{
	return m_dataStore->updatePointCloud(index, filepath);
}

bool Model::updateTrainGtFilePath(int index, std::string& filepath)
{
	return m_dataStore->updateTrainGtFilePath(index, filepath);
}

bool Model::updateResultGtFilePath(int index, std::string& filepath)
{
	return m_dataStore->updateResultGtFilePath(index, filepath);
}

bool Model::clearCurrentTrainSampleMark()
{
	return m_dataStore->clearTrainSampleMark(getCurrentIndex());
}

bool Model::clearAllTestSampleMark()
{
	bool ret = true;
	for (int i = 0; i < getCurrentLoadImageNum(); ++i) {
		ret = m_dataStore->updateResultSampleMark(i, SampleMark());
	}
	return ret;
}

bool Model::clearAllPointCloud()
{
	return m_dataStore->clearAllPointCloud();
}

bool Model::clearAllShrinkageChart()
{
	return m_dataStore->clearAllShrinkageChart();
}

void Model::DropAllTables()
{
	m_dataStore->DropAllTables();
}

void Model::getTrainSamples(std::vector<SampleInfo>* trainSamples)
{
	for (int i = 0; i < getCurrentLoadImageNum(); ++i) {
		te::SampleInfo sampleInfo;
		sampleInfo.sampleMark = getSelectTrainSampleInfo(i);
		te::Image ss = te::Image::load(getSelectOriginImage(i));
		sampleInfo.sampleData.imageMatrix.push_back(ss);
		sampleInfo.sampleData.roi = { 0,0,ss.width(),ss.height() };

		trainSamples->push_back(sampleInfo);
	}
}

void Model::getResultSamples(std::vector<SampleInfo>* resultSamples)
{
	for (int i = 0; i < getCurrentLoadImageNum(); ++i) {
		te::SampleInfo sampleInfo;
		sampleInfo.sampleMark = getSelectResultSampleInfo(i);

		resultSamples->push_back(sampleInfo);
	}
}

void Model::initThreasholds(int size)
{
	ValidPointThresholds = std::vector<double>(size, 0.0);
	InvalidPointThresholds = std::vector<double>(size, 0.0);
}

void Model::updateInvalidPointThreshold(double threshold)
{
	if (getCurrentIndex() < InvalidPointThresholds.size())
		InvalidPointThresholds[getCurrentIndex()] = threshold;
}

double Model::getSelectInvalidPointThreshold(int index)
{
	if (index < InvalidPointThresholds.size())
		return InvalidPointThresholds[index];
	else
		return 0;
}

double Model::getCurrentInvalidPointThreshold()
{
	if (getCurrentIndex() < InvalidPointThresholds.size())
		return InvalidPointThresholds[getCurrentIndex()];
	else
		return 0;
}

void Model::addInvalidPointThreshold(int index, double threshold)
{
	InvalidPointThresholds[index] = threshold;
}

void Model::updateValidPointThreshold(double threshold)
{
	if (getCurrentIndex() < ValidPointThresholds.size())
		ValidPointThresholds[getCurrentIndex()] = threshold;
}

double Model::getSelectValidPointThreshold(int index)
{
	if (index < ValidPointThresholds.size())
		return ValidPointThresholds[index];
	else
		return 0;
}

double Model::getCurrentValidPointThreshold()
{
	if (getCurrentIndex() < ValidPointThresholds.size())
		return ValidPointThresholds[getCurrentIndex()];
	else
		return 0;
}

void Model::addValidPointThreshold(int index, double threshold)
{
	ValidPointThresholds[index] = threshold;
}

void Model::loadTrainingImages(const QStringList& filePaths)
{
	for (QString filePath : filePaths) {
		if (!isImageAlreadyExist(filePath.toStdString()))
			m_dataStore->insertOriginImage(filePath.toStdString());
	}
	setCurrentLoadImageNum(filePaths.size());
}

void Model::setCallback(CallbackFunction callback)
{
	callback_ = callback;
}

void Model::triggerCallback(int iteration, float fAvgLoss, float fPosAcc)
{
	if (callback_)
	{
		callback_(iteration, fAvgLoss, fPosAcc);
	}
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
