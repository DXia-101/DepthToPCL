#include "teDataStorage.h"

teDataStorage::Garbo teDataStorage::tmp;

teDataStorage* teDataStorage::instance = nullptr;

teDataStorage::teDataStorage(QObject *parent)
	: QObject(parent)
{
	InitDatabase();
}

teDataStorage::~teDataStorage()
{}

teDataStorage::teDataStorage(const teDataStorage&)
{
}

teDataStorage& teDataStorage::operator=(const teDataStorage&)
{
	return *this;
}

teDataStorage* teDataStorage::getInstance()
{
	if (!instance)
	{
		teDataStorage* pInstance = new teDataStorage();
		instance = pInstance;
	}
	return instance;
}

void teDataStorage::destroy()
{
	if (NULL != teDataStorage::instance) {
		delete teDataStorage::instance;
		teDataStorage::instance = NULL;
	}
}

void teDataStorage::InitDatabase()
{
	db = te::Sqlite3DB::open("D:/teAi3D.db");

	ResourceTable = db->createDataModel("Resource",
		{
			te::makeFieldType<te::StdU8String>("OriginImagePath"),
			te::makeFieldType<te::StdU8String>("ShrinkageChartPath"),
			te::makeFieldType<te::StdU8String>("PointCloudPath"),
			te::makeFieldType<te::SampleMark>("TrainSampleMark"),
			te::makeFieldType<te::SampleMark>("ResultSampleMark"),
		});

	GtTable = db->createDataModel("TrainGt",
		{
			te::makeFieldType<te::StdU8String>("TrainGtFilePath"),
			te::makeFieldType<te::StdU8String>("ResultGtFilePath"),
		});
}

void teDataStorage::displayUIInWidget(QVBoxLayout* layout)
{
	m_teLabelBrowser = new teLabelBrowser();
	layout->addWidget(m_teLabelBrowser);
	m_teLabelBrowser->show();

	connect(m_teLabelBrowser, &teLabelBrowser::sig_currentRowSelected, this, &teDataStorage::currentRowChange);
}

int teDataStorage::getCurrentIndex()
{
	return currentIndex;
}

int teDataStorage::getCurrentLoadImageNum()
{
	return currentLoadImageNum;
}

std::string teDataStorage::getSelectResultFormResourceTable(int index, std::string keyword)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index+1);
	std::string result;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(result);
		}
	}
	return result;
}

std::vector<std::string> teDataStorage::getResultFromResourceTable(std::string keyword)
{
	std::vector<std::string> result;
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource";
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			te::StdU8String path;
			dataQuery->getRecord_t(path);
			result.push_back(path);
		}
	}
	return result;
}

std::pair<std::string, std::string> teDataStorage::getSelectResultFormGtTable(int index)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select TrainGtFilePath,ResultGtFilePath from TrainGt where ID = " + std::to_string(index+1);
	std::string TrainGt,ResultGt;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(TrainGt, ResultGt);
		}
	}
	return {TrainGt,ResultGt};
}

te::SampleMark teDataStorage::getSelectResultFormSampleMarkTable(int index, std::string keyword)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index+1);
	te::SampleMark result;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(result);
		}
	}
	return result;
}

std::vector<std::string> teDataStorage::getShrinkageChart()
{
	return getResultFromResourceTable("ShrinkageChartPath");
}

std::vector<std::string> teDataStorage::getOriginImage()
{
	return getResultFromResourceTable("OriginImagePath");
}

std::vector<std::string> teDataStorage::getPointCloud()
{
	return getResultFromResourceTable("PointCloudPath");
}

std::string teDataStorage::getSelectShrinkageChart(int index)
{
	return getSelectResultFormResourceTable(index,"ShrinkageChartPath");
}

std::string teDataStorage::getSelectOriginImage(int index)
{
	return getSelectResultFormResourceTable(index, "OriginImagePath");
}

std::string teDataStorage::getCurrentOriginImage()
{
	return getSelectResultFormResourceTable(currentIndex, "OriginImagePath");
}

std::string teDataStorage::getSelectPointCloud(int index)
{
	return getSelectResultFormResourceTable(index, "PointCloudPath");
}

std::string teDataStorage::getCurrentPointCloud()
{
	return getSelectResultFormResourceTable(currentIndex, "PointCloudPath");
}

std::string teDataStorage::getSelectTrainGt(int index)
{
	return getSelectResultFormGtTable(index).first;
}

std::string teDataStorage::getSelectResultGt(int index)
{
	return getSelectResultFormGtTable(index).second;
}

te::SampleMark teDataStorage::getSelectTrainSampleInfo(int index)
{
	return getSelectResultFormSampleMarkTable(index,"TrainSampleMark");
}

te::SampleMark teDataStorage::getSelectResultSampleInfo(int index)
{
	return getSelectResultFormSampleMarkTable(index,"ResultSampleMark");
}

te::SampleMark teDataStorage::getCurrentTrainSampleInfo()
{
	return getSelectResultFormSampleMarkTable(currentIndex, "TrainSampleMark");
}

te::SampleMark teDataStorage::getCurrentResultSampleInfo()
{
	return getSelectResultFormSampleMarkTable(currentIndex, "ResultSampleMark");
}

bool teDataStorage::insertOriginImage(std::string filepath)
{
	te::StdU8String originimage = filepath;
	te::StdU8String shrinkagechart = "";
	te::StdU8String pointcloud = "";
	te::SampleMark train;
	te::SampleMark result;
	
	ResourceTable->transaction();
	ResourceTable->insertRecord_t(-1, originimage, shrinkagechart, pointcloud, train, result);
	return ResourceTable->commit();
}

bool teDataStorage::insertGtFilePath(std::string filepath)
{
	te::StdU8String GtFilePath = filepath;

	ResourceTable->transaction();
	ResourceTable->insertRecord_t(-1,GtFilePath,"");
	return ResourceTable->commit();
}

bool teDataStorage::updateTrainSampleMark(int index, te::SampleMark& samplemark)
{
	return ResourceTable->updateRecord(index+1, { std::pair<te::StdU8String,te::SampleMark>("TrainSampleMark",samplemark) });
}

bool teDataStorage::updateResultSampleMark(int index, te::SampleMark& samplemark)
{
	return ResourceTable->updateRecord(index+1, { std::pair<te::StdU8String,te::SampleMark>("ResultSampleMark",samplemark) });
}

bool teDataStorage::updateCurrentTrainSampleMark(te::SampleMark& samplemark)
{
	return ResourceTable->updateRecord(currentIndex + 1, { std::pair<te::StdU8String,te::SampleMark>("TrainSampleMark",samplemark) });
}

bool teDataStorage::updateCurrentResultSampleMark(te::SampleMark& samplemark)
{
	return ResourceTable->updateRecord(currentIndex + 1, { std::pair<te::StdU8String,te::SampleMark>("ResultSampleMark",samplemark) });
}

bool teDataStorage::updateShrinkageChart(int index, std::string& filepath)
{
	return ResourceTable->updateRecord(index+1, { std::pair<te::StdU8String,te::StdU8String>("ShrinkageChartPath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::updatePointCloud(int index, std::string& filepath)
{
	return ResourceTable->updateRecord(index+1, { std::pair<te::StdU8String,te::StdU8String>("PointCloudPath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::updateTrainGtFilePath(int index, std::string& filepath)
{
	return GtTable->updateRecord(index+1, { std::pair<te::StdU8String,te::StdU8String>("TrainGtFilePath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::updateResultGtFilePath(int index, std::string& filepath)
{
	return GtTable->updateRecord(index+1, { std::pair<te::StdU8String,te::StdU8String>("ResultGtFilePath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::isOriginImage(std::string filepath)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select COUNT(*) from Resource where OriginImagePath = '" + filepath + "'";
	int count;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(count);
			if (count > 0) {
				return true;
			}
			else {
				return false;
			}
		}
		return false;
	}
	return false;
}

QString teDataStorage::getCurrentLabelCategory()
{
	return currentCategory;
}

QColor teDataStorage::getCurrentLabelColor()
{
	return currentColor;
}

void teDataStorage::getTrainSamples(std::vector<te::SampleInfo>* trainSamples)
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

void teDataStorage::getResultSamples(std::vector<te::SampleInfo>* resultSamples)
{
	for (int i = 0; i < currentLoadImageNum; ++i) {
		te::SampleInfo sampleInfo;
		sampleInfo.sampleMark = getSelectResultSampleInfo(i);

		resultSamples->push_back(sampleInfo);
	}
}

void teDataStorage::setDataDuringTraining(int iteration, float fAvgLoss, float fPosAcc)
{
	emit sig_DataChangeDuringTraining(iteration, fAvgLoss, fPosAcc);
}

void teDataStorage::DropAllTables()
{
	db->dropDataModel("Resource");

	ResourceTable = db->createDataModel("Resource",
		{
			te::makeFieldType<te::StdU8String>("OriginImagePath"),
			te::makeFieldType<te::StdU8String>("ShrinkageChartPath"),
			te::makeFieldType<te::StdU8String>("PointCloudPath"),
			te::makeFieldType<te::SampleMark>("TrainSampleMark"),
			te::makeFieldType<te::SampleMark>("ResultSampleMark"),
		});
}

QColor teDataStorage::FindContentColor(const QString& searchString)
{
	return m_teLabelBrowser->getFontColorByFirstColumnValue(searchString);
}

void teDataStorage::setCurrentLoadImageNum(int num)
{
	currentLoadImageNum = num;
}

void teDataStorage::LoadTrainingImages(const QStringList& filePaths)
{
	for (QString filePath : filePaths) {
		if (!isOriginImage(filePath.toStdString())) {
			insertOriginImage(filePath.toStdString());
		}
	}
	//在这处理高度变换的相关逻辑


	emit sig_teUpDataSet(filePaths.size(), 1,true);
	emit sig_LoadTrainImagesComplete();
}

void teDataStorage::currentRowChange(const QString& content, const QColor& fontColor)
{
	currentCategory = content;
	currentColor = fontColor;
	emit sig_currentLabelChange(content, fontColor);
}

void teDataStorage::clearCurrentMarkersGT()
{
	ResourceTable->updateRecord(currentIndex + 1, { std::pair<te::StdU8String,te::SampleMark>("TrainSampleMark",te::SampleMark()) });
}

void teDataStorage::setCurrentIndex(int index)
{
	currentIndex = index;
}