#include "teModel.h"
#include "Transfer_Function.h"
#include "Depth2RGB.h"
#include "IDataStore.h"
#include "teAugmentation.h"
#include "teImage.h"
#include "teRapidjsonObjectTree.h"
#include "teTimer.h"
#include "teAiExTypes.h"
#include "teTimer.h"
#include "Render3DAlgorithm.h"
#include <QColor>
#include <QString>
#include <vector>
#include <string>
#include "teTraining.h"
#include "tePrediction.h"


#include<ctime>
#include<chrono>

namespace te
{
	class ModelMenber
	{
	public:
		std::vector<double> InvalidPointThresholds;
		std::vector<double> ValidPointThresholds;

		std::string modelPath;

		TrainConfig config;
		int DeviceID;
		AiStatus status;
		ModelInfer infer_;
		Training train_;
	};
}

using namespace te;

static AiResult m_InferResult;
Model* Model::instance = nullptr;

Model::Model(std::unique_ptr<IDataStore> dataStore)
	:m_dataStore(std::move(dataStore))
{
	instance = this;
	menber = new ModelMenber();
}

Model::~Model()
{
	if (instance == this) {
		instance = nullptr; // 在析构函数中清除静态指针
	}
}

void Model::trainModel()
{
	std::condition_variable cv;
	AiStatus status;
	std::string initInfo;
	std::vector<te::SampleInfo> trainSamples;
	getTrainSamples(&trainSamples);
	menber->train_.setDeviceInfo({ E_NVIDIA, menber->DeviceID });
	menber->train_.setTrainConfig(menber->config);
	status = menber->train_.init(trainSamples, initInfo);

	if (status != E_Success)
	{
		printf("train_.init %d\n", status);
		system("Pause");
		return;
	}

	menber->train_.start(teTrainStateCallBack, &cv);

	{
		std::mutex lock;
		std::unique_lock locker(lock);
		cv.wait(locker);
	}

	menber->train_.stop();
}

void Model::testModel()
{
	std::condition_variable cv;
	std::vector<te::SampleInfo> trainSamples;
	getTrainSamples(&trainSamples);
	int maxImageW = 0, maxImageH = 0;
	for (size_t i = 0; i < trainSamples.size(); i++)
	{
		if (maxImageW < trainSamples[i].sampleData.roi.width)
			maxImageW = trainSamples[i].sampleData.roi.width;
		if (maxImageH < trainSamples[i].sampleData.roi.height)
			maxImageH = trainSamples[i].sampleData.roi.height;
	}

	menber->status = menber->infer_.init(maxImageW, maxImageH);
	if (menber->status != E_Success)
	{
		printf("infer_.init %d\n", menber->status);
		system("Pause");
		return;
	}
	NetMemorySize neededMemory;
	menber->status = menber->infer_.calcuMemoryWorkspace(10 * 1024 * (uint64_t)1024 * 1024, 10 * 1024 * (uint64_t)1024 * 1024, neededMemory);
	if (menber->status != E_Success)
	{
		printf("infer_.calcuMemoryWorkspace %d\n", menber->status);
		system("Pause");
		return;
	}

	menber->status = menber->infer_.setShareMemoryAddr(nullptr, 0, nullptr, 0);
	if (menber->status != E_Success)
	{
		printf("infer_.setShareMemoryAddr %d\n", menber->status);
		system("Pause");
		return;
	}

	menber->status = menber->infer_.setResultCallbackFunc(teAiInferResult);

	if (!clearAllTestSampleMark())
		return;

	for (size_t i = 0; i < trainSamples.size(); i++)
	{
		std::mutex lock;
		std::unique_lock locker(lock);

		auto startTimePoint = std::chrono::steady_clock::now();
		menber->infer_.pushSampleData(trainSamples[i].sampleData, &cv);

		cv.wait(locker);
		auto endTimePoint = std::chrono::steady_clock::now();

		te::SampleMark samplemark;
		samplemark.gtDataSet = m_InferResult;
		updateResultSampleMark(i, samplemark);
	}
}

void Model::initTrainConfig(TrainParaRegister* para)
{
	ToolType toolType = ToolType::E_PixelDetect_Tool;

	menber->config.batchSize = para->TrainBatchSize;
	menber->config.patchWidth = para->PatchWidth;
	menber->config.patchHeight = para->PatchHeight;
	menber->config.receptiveField_A = para->receptiveField;
	menber->config.receptiveField_B = 2;
	menber->config.trainIterCnt = para->trainIterCnt;
	menber->config.saveFrequency = para->saveFrequency;
	menber->config.eToolType = toolType;
	menber->config.eTrainMode = TrainMode::E_TE_RESET;
	menber->config.locateType = para->eLocateType;
	menber->config.locateSide = para->locateSide;
	menber->config.netName = para->netName;
	menber->config.augmentHandle = nullptr;
	menber->config.modelPath = menber->modelPath;
	menber->config.sampleDesc.resize(1);
	menber->config.sampleDesc = para->sampleDesc;
	menber->config.augmentHandle = new AugmentProcess();
	menber->config.augmentHandle = nullptr;

	menber->DeviceID = para->DeviceID;
}

void Model::initTestConfig(TestParaRegister* para)
{
	menber->status = menber->infer_.setMaxBatchSize(para->maxbatchsize);
	menber->status = menber->infer_.setBatchSize(para->batchsize);
	menber->status = menber->infer_.setCoutourDesc(para->contourdesc);
	menber->status = menber->infer_.setComputeDesc(para->deviceinfo);
	menber->status = menber->infer_.setModelPath(menber->modelPath.c_str());
	menber->status = menber->infer_.setPrecisionType(para->precision);
	menber->status = menber->infer_.setExceptionFunc(teException, nullptr);
}

void Model::setmodelpath(std::string modelpath)
{
	menber->modelPath = modelpath;
}

void te::Model::stopTrain()
{
	menber->train_.stop();
}

Training Model::getTrainHandle()
{
	return menber->train_;
}

int Model::teException(void* pParam, AiStatus eStatus)
{
	if (eStatus != E_Success)
	{
		printf("teException.error %d\n", eStatus);
	}
	return 0;
}

void Model::teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param)
{
	printf("**teTrainStateCallBack**\n");
	if (status != E_Success)
	{
		printf("train_.error %d\n", status);
		auto cv = (std::condition_variable*)param;
		cv->notify_all();
	}
	if (instance != nullptr) {
		instance->TriggerCallback(stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);
	}


	if (stateinfo.fProgress > 0.999999f)
	{
		auto cv = (std::condition_variable*)param;
		cv->notify_all();
	}
}

void Model::teAiInferResult(AiResult& inferResult, DynamicMatrix& hotmap, void* pCallbackParam)
{
	if (inferResult.size() > 0)
		std::cout << inferResult[0].name << std::endl;

	{
		m_InferResult = inferResult;
		auto cv = (std::condition_variable*)pCallbackParam;
		cv->notify_all();
	}
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

std::pair<int, int> Model::getCurrentImageSize()
{
	cv::Mat image = cv::imread(getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	if (image.empty()) {
		std::cerr << "Error: Cloud not read the image." << std::endl;
		return { 0,0 };
	}
	return std::make_pair(image.cols, image.rows);
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
	menber->ValidPointThresholds = std::vector<double>(size, 0.0);
	menber->InvalidPointThresholds = std::vector<double>(size, 0.0);
}

void Model::updateInvalidPointThreshold(double threshold)
{
	if (getCurrentIndex() < menber->InvalidPointThresholds.size())
		menber->InvalidPointThresholds[getCurrentIndex()] = threshold;
}

double Model::getSelectInvalidPointThreshold(int index)
{
	if (index < menber->InvalidPointThresholds.size())
		return menber->InvalidPointThresholds[index];
	else
		return 0;
}

double Model::getCurrentInvalidPointThreshold()
{
	if (getCurrentIndex() < menber->InvalidPointThresholds.size())
		return menber->InvalidPointThresholds[getCurrentIndex()];
	else
		return 0;
}

void Model::addInvalidPointThreshold(int index, double threshold)
{
	menber->InvalidPointThresholds[index] = threshold;
}

void Model::updateValidPointThreshold(double threshold)
{
	if (getCurrentIndex() < menber->ValidPointThresholds.size())
		menber->ValidPointThresholds[getCurrentIndex()] = threshold;
}

double Model::getSelectValidPointThreshold(int index)
{
	if (index < menber->ValidPointThresholds.size())
		return menber->ValidPointThresholds[index];
	else
		return 0;
}

double Model::getCurrentValidPointThreshold()
{
	if (getCurrentIndex() < menber->ValidPointThresholds.size())
		return menber->ValidPointThresholds[getCurrentIndex()];
	else
		return 0;
}

void Model::addValidPointThreshold(int index, double threshold)
{
	menber->ValidPointThresholds[index] = threshold;
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
	return Render_3D_Algorithm::SavePointCloud(fileName, saveCloud);
}

void Model::segment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, vtkMatrix4x4* mat, vtkMatrix4x4* transmat)
{
	setSegmentedPointCloud(Render_3D_Algorithm::Segment(getThreeDMarkerPolygon(), getVtkWindowSize(), getRenderViewport(), cloud, mat, transmat));
}

void Model::coordinateAxisRendering(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	QString renderAxis = "z";
	getData("RenderAxis", renderAxis);

	double min = 0.0, max = 0.0;
	if (renderAxis == "z")
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
	else if (renderAxis == "x")
	{
		pcl::PointXYZ maxPt;
		getData("maxPt", maxPt);
		pcl::PointXYZ minPt;
		getData("minPt", minPt);
		min = minPt.x;
		max = maxPt.x;
	}
	else if (renderAxis == "y")
	{
		pcl::PointXYZ maxPt;
		getData("maxPt", maxPt);
		pcl::PointXYZ minPt;
		getData("minPt", minPt);
		min = minPt.y;
		max = maxPt.y;
	}

	Render_3D_Algorithm::CoordinateAxisRendering(cloud, renderAxis, min, max);
}

void Model::extractingPointCloudsBasedOnContours(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::axisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::orientedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	return;
}

void Model::subtractTargetPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud2)
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
	te::AiInstance instance;
	instance.name = getCurrentLabel().toStdString();
	std::vector<std::vector<cv::Point>> contours = Transfer_Function::Cloud2Contour(getCurrentImageSize().first, getCurrentImageSize().second, cloud);

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
	te::SampleMark sampleMark = getCurrentTrainSampleInfo();
	sampleMark.gtDataSet.push_back(instance);
	updateCurrentTrainSampleMark(sampleMark);
}

void Model::add2DAiInstance(QList<te::GraphicsItem*> polygonItems)
{
	clearCurrentTrainSampleMark();
	te::SampleMark sampleMark = getCurrentTrainSampleInfo();
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
	updateCurrentTrainSampleMark(sampleMark);
}

void Model::addLabelInfo(const QString& category, const QColor& color)
{
	labelStore[category.toStdString()] = color.name().toStdString();
}

void Model::changeLabelInfo(const QString& category, const QColor& color)
{
	labelStore[category.toStdString()] = color.name().toStdString();
}

void Model::removeLabelInfo(const QString& category)
{
	labelStore.erase(category.toStdString());
}

QColor Model::getLabelColor(const QString& category)
{
	return QColor(QString::fromStdString(labelStore[category.toStdString()]));
}

void Model::setThreeDMarkerPolygon(QVector<QPointF> vec)
{
	setData("ThreeDMarkerPolygon", vec);
}

QVector<QPointF> Model::getThreeDMarkerPolygon()
{
	QVector<QPointF> vec;
	if (getData("ThreeDMarkerPolygon", vec))
		return vec;
	else
		return vec;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr te::Model::getSegmentedPointCloud()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
	if (getData("SegmentedPointCloud", cloud))
		return cloud;
	else
		return pcl::PointCloud<pcl::PointXYZRGB>::Ptr();
}

void te::Model::setSegmentedPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	setData("SegmentedPointCloud", cloud);
}

std::vector<int> te::Model::getVtkWindowSize()
{
	std::vector<int> size;
	if (getData("VtkWindowSize", size))
		return size;
	else
		return size;
}

void te::Model::setVtkWindowSize(std::vector<int> size)
{
	setData("VtkWindowSize", size);
}

std::vector<double> te::Model::getRenderViewport()
{
	std::vector<double> viewport;
	if (getData("RenderViewport", viewport))
		return viewport;
	else
		return viewport;
}

void te::Model::setRenderViewport(std::vector<double> viewport)
{
	setData("RenderViewport", viewport);
}

QString te::Model::getCurrentLabel()
{
	QString category = "";
	if (getData("CurrentLabel", category))
	{
		return category;
	}
	return "";
}

void te::Model::setCurrentLabel(QString str)
{
	setData("CurrentLabel", str);
}

void te::Model::SetCallback(CallbackFunction callback)
{
}

void te::Model::TriggerCallback(int iteration, float fAvgLoss, float fPosAcc)
{
}
