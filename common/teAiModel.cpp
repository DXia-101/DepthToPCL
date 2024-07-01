#include "teAiModel.h"

#include "pcl_function.h"
#include <QString>
#include <QDir>
#include "IDataStore.h"

teAiModel::teAiModel(std::unique_ptr<IDataStore> dataStore)
	:m_dataStore(std::move(dataStore))
{
	currentIndex = 0;
	currentLoadImageNum = 0;
}

teAiModel::~teAiModel()
{
}

std::vector<std::string> teAiModel::getShrinkageChart()
{
	return m_dataStore->getResultFromResourceTable("ShrinkageChartPath");
}

std::vector<std::string> teAiModel::getOriginImage()
{
	return m_dataStore->getResultFromResourceTable("OriginImagePath");
}

std::vector<std::string> teAiModel::getPointCloud()
{
	return m_dataStore->getResultFromResourceTable("PointCloudPath");
}

std::string teAiModel::getSelectShrinkageChart(int index)
{
	return m_dataStore->getSelectResultFormResourceTable(index, "ShrinkageChartPath");
}

std::string teAiModel::getCurrentShrinkageChart()
{
	return m_dataStore->getSelectResultFormResourceTable(currentIndex, "ShrinkageChartPath");
}

std::string teAiModel::getSelectOriginImage(int index)
{
	return m_dataStore->getSelectResultFormResourceTable(index, "OriginImagePath");
}

std::string teAiModel::getCurrentOriginImage()
{
	return m_dataStore->getSelectResultFormResourceTable(currentIndex, "OriginImagePath");
}

std::string teAiModel::getSelectPointCloud(int index)
{
	return m_dataStore->getSelectResultFormResourceTable(index, "PointCloudPath");
}

std::string teAiModel::getCurrentPointCloud()
{
	return m_dataStore->getSelectResultFormResourceTable(currentIndex, "PointCloudPath");
}

std::string teAiModel::getSelectTrainGt(int index)
{
	return m_dataStore->getSelectResultFormGtTable(index).first;
}

std::string teAiModel::getSelectResultGt(int index)
{
	return m_dataStore->getSelectResultFormGtTable(index).second;
}

te::SampleMark teAiModel::getSelectTrainSampleInfo(int index)
{
	return m_dataStore->getSelectResultFormSampleMarkTable(index, "TrainSampleMark");
}

te::SampleMark teAiModel::getSelectResultSampleInfo(int index)
{
	return m_dataStore->getSelectResultFormSampleMarkTable(index, "ResultSampleMark");
}

te::SampleMark teAiModel::getCurrentTrainSampleInfo()
{
	return m_dataStore->getSelectResultFormSampleMarkTable(currentIndex, "TrainSampleMark");
}

te::SampleMark teAiModel::getCurrentResultSampleInfo()
{
	return m_dataStore->getSelectResultFormSampleMarkTable(currentIndex, "ResultSampleMark");
}

QMap<QString, int> teAiModel::getCurrentTrainMarksNumber()
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

QMap<QString, int> teAiModel::getCurrentResultMarksNumber()
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

int teAiModel::getCurrentImageWidth()
{
	cv::Mat image = cv::imread(getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	if (image.empty()) {
		std::cerr << "Error: Cloud not read the image." << std::endl;
		return 0;
	}
	return image.cols;
}

int teAiModel::getCurrentImageHeight()
{
	cv::Mat image = cv::imread(getCurrentOriginImage(), cv::IMREAD_UNCHANGED);
	if (image.empty()) {
		std::cerr << "Error: Cloud not read the image." << std::endl;
		return 0;
	}
	return image.rows;
}

bool teAiModel::updateTrainSampleMark(int index, te::SampleMark& samplemark)
{
	return m_dataStore->updateTrainSampleMark(index,samplemark);
}

bool teAiModel::updateResultSampleMark(int index, te::SampleMark& samplemark)
{
	return m_dataStore->updateResultSampleMark(index, samplemark);
}

bool teAiModel::updateCurrentTrainSampleMark(te::SampleMark& samplemark)
{
	return m_dataStore->updateTrainSampleMark(currentIndex,samplemark);
}

bool teAiModel::updateCurrentResultSampleMark(te::SampleMark& samplemark)
{
	return m_dataStore->updateResultSampleMark(currentIndex, samplemark);
}

bool teAiModel::updateShrinkageChart(int index, std::string& filepath)
{
	return m_dataStore->updateShrinkageChart(index, filepath);
}

bool teAiModel::updatePointCloud(int index, std::string& filepath)
{
	return m_dataStore->updatePointCloud(index, filepath);
}

bool teAiModel::updateTrainGtFilePath(int index, std::string& filepath)
{
	return m_dataStore->updateTrainGtFilePath(index, filepath);
}

bool teAiModel::updateResultGtFilePath(int index, std::string& filepath)
{
	return m_dataStore->updateResultGtFilePath(index, filepath);
}

bool teAiModel::isImageAlreadyExist(std::string filepath)
{
	return m_dataStore->isImageAlreadyExist(filepath);
}

bool teAiModel::clearCurrentTrainSampleMark()
{
	return m_dataStore->clearTrainSampleMark(currentIndex);
}

bool teAiModel::clearAllTestSampleMark()
{
	return m_dataStore->clearAllTestSampleMark();
}

bool teAiModel::clearCurrentPointCloudAndThumbnail()
{
	bool ret = QFile::remove(QString::fromStdString(m_dataStore->GetCurrentPath()) + QString::fromStdString(getSelectShrinkageChart(currentIndex)));
	ret = QFile::remove(QString::fromStdString(m_dataStore->GetCurrentPath()) + QString::fromStdString(getCurrentPointCloud()));
	return ret;
}

bool teAiModel::clearAllPointCloud()
{
	return m_dataStore->clearAllPointCloud();
}

bool teAiModel::clearAllThumbnail()
{
	return m_dataStore->clearAllThumbnail();
}

void teAiModel::DropAllTables()
{
	m_dataStore->DropAllTables();
}

void teAiModel::getTrainSamples(std::vector<te::SampleInfo>* trainSamples)
{
	for (int i = 0; i < currentLoadImageNum; ++i) {
		te::SampleInfo sampleInfo;
		sampleInfo.sampleMark = getSelectTrainSampleInfo(i);
		te::Image ss = te::Image::load(getSelectOriginImage(i));
		sampleInfo.sampleData.imageMatrix.push_back(ss);
		sampleInfo.sampleData.roi = { 0,0,ss.width(),ss.height() };

		trainSamples->push_back(sampleInfo);
	}
}

void teAiModel::getResultSamples(std::vector<te::SampleInfo>* resultSamples)
{
	for (int i = 0; i < currentLoadImageNum; ++i) {
		te::SampleInfo sampleInfo;
		sampleInfo.sampleMark = getSelectResultSampleInfo(i);

		resultSamples->push_back(sampleInfo);
	}
}

void teAiModel::InitThreasholds(int size)
{
	ValidPointThresholds = std::vector<double>(size, 0.0);
	InvalidPointThresholds = std::vector<double>(size, 0.0);
}

void teAiModel::InvalidPointThresholdsChange(double threshold)
{
	for (int i = 0; i < InvalidPointThresholds.size(); ++i) {
		InvalidPointThresholds[i] = threshold;
	}
}

void teAiModel::InvalidPointThresholdChange(double threshold)
{
	if (currentIndex < InvalidPointThresholds.size())
		InvalidPointThresholds[currentIndex] = threshold;
}

double teAiModel::getSelectInvalidPointThreshold(int index)
{
	if (index < InvalidPointThresholds.size())
		return InvalidPointThresholds[index];
	else
		return 0;
}

double teAiModel::getCurrentInvalidPointThreshold()
{
	if (currentIndex < InvalidPointThresholds.size())
		return InvalidPointThresholds[currentIndex];
	else
		return 0;
}

void teAiModel::addInvalidPointThreshold(int index, double threshold)
{
	InvalidPointThresholds[index] = threshold;
}

void teAiModel::ValidPointThresholdsChange(double threshold)
{
	for (int i = 0; i < ValidPointThresholds.size(); ++i) {
		ValidPointThresholds[i] = threshold;
	}
}

void teAiModel::ValidPointThresholdChange(double threshold)
{
	if (currentIndex < ValidPointThresholds.size())
		ValidPointThresholds[currentIndex] = threshold;
}

double teAiModel::getSelectValidPointThreshold(int index)
{
	if (index < ValidPointThresholds.size())
		return ValidPointThresholds[index];
	else
		return 0;
}

double teAiModel::getCurrentValidPointThreshold()
{
	if (currentIndex < ValidPointThresholds.size())
		return ValidPointThresholds[currentIndex];
	else
		return 0;
}

void teAiModel::addValidPointThreshold(int index, double threshold)
{
	ValidPointThresholds[index] = threshold;
}

void teAiModel::LoadTrainingImages(const QStringList& filePaths)
{
	for (QString filePath : filePaths) {
		if(!isImageAlreadyExist(filePath.toStdString()))
			m_dataStore->insertOriginImage(filePath.toStdString());
	}
	currentLoadImageNum = filePaths.size();
	m_dataStore->setCurrentLoadImageNum(currentLoadImageNum);
}

int teAiModel::getCurrentIndex()
{
	return currentIndex;
}

QString teAiModel::GetCurrentPath()
{
	return QString::fromStdString(m_dataStore->GetCurrentPath());
}

void teAiModel::setCurrentIndex(int num)
{
	currentIndex = num;
}

int teAiModel::getCurrentLoadImageNum()
{
	return currentLoadImageNum;
}
