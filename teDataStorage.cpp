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
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index);
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
	std::string sqlquery = "select TrainGtFilePath,ResultGtFilePath from TrainGt where ID = " + std::to_string(index);
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
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index);
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

std::string teDataStorage::getSelectPointCloud(int index)
{
	return getSelectResultFormResourceTable(index, "PointCloudPath");
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
	return getSelectResultFormSampleMarkTable(index,"Train");
}

te::SampleMark teDataStorage::getSelectResultSampleInfo(int index)
{
	return getSelectResultFormSampleMarkTable(index,"Result");
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

bool teDataStorage::updateTrainSampleMark(int index, te::SampleMark samplemark)
{
	return ResourceTable->updateRecord(index, { std::pair<te::StdU8String,te::SampleMark>("TrainSampleMark",samplemark) });
}

bool teDataStorage::updateResultSampleMark(int index, te::SampleMark samplemark)
{
	return ResourceTable->updateRecord(index, { std::pair<te::StdU8String,te::SampleMark>("ResultSampleMark",samplemark) });
}

bool teDataStorage::updateShrinkageChart(int index, std::string filepath)
{
	return ResourceTable->updateRecord(index, { std::pair<te::StdU8String,te::StdU8String>("ShrinkageChartPath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::updatePointCloud(int index, std::string filepath)
{
	return ResourceTable->updateRecord(index, { std::pair<te::StdU8String,te::StdU8String>("PointCloudPath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::updateTrainGtFilePath(int index, std::string filepath)
{
	return GtTable->updateRecord(index, { std::pair<te::StdU8String,te::StdU8String>("TrainGtFilePath",static_cast<te::StdU8String>(filepath)) });
}

bool teDataStorage::updateResultGtFilePath(int index, std::string filepath)
{
	return GtTable->updateRecord(index, { std::pair<te::StdU8String,te::StdU8String>("ResultGtFilePath",static_cast<te::StdU8String>(filepath)) });
}

void teDataStorage::setCurrentLoadImageNum(int num)
{
	currentLoadImageNum = num;
}

void teDataStorage::setCurrentIndex(int index)
{
	currentIndex = index;
}