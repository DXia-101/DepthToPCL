#include "teAi3DStorage.h"
#include <QString>
#include <QDir>

teAi3DStorage::teAi3DStorage()
{
	QString dir = QDir::currentPath() + "/workspace/teAi3D.db";
	db = te::Sqlite3DB::open(dir.toStdString());

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

std::vector<std::string> teAi3DStorage::getResultFromResourceTable(const std::string& keyword)
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

std::string teAi3DStorage::getSelectResultFormResourceTable(const int& index, const std::string& keyword)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index + 1);
	std::string result;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(result);
		}
	}
	return result;
}

std::pair<std::string, std::string> teAi3DStorage::getSelectResultFormGtTable(const int& index)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select TrainGtFilePath,ResultGtFilePath from TrainGt where ID = " + std::to_string(index + 1);
	std::string TrainGt, ResultGt;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(TrainGt, ResultGt);
		}
	}
	return { TrainGt,ResultGt };
}

te::SampleMark teAi3DStorage::getSelectResultFormSampleMarkTable(const int& index, const std::string& keyword)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index + 1);
	te::SampleMark result;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(result);
		}
	}
	return result;
}

void teAi3DStorage::DropAllTables()
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

bool teAi3DStorage::insertOriginImage(const std::string& filepath)
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

bool teAi3DStorage::insertGtFilePath(const std::string& filepath)
{
	te::StdU8String GtFilePath = filepath;

	ResourceTable->transaction();
	ResourceTable->insertRecord_t(-1, GtFilePath, "");
	return ResourceTable->commit();
}

bool teAi3DStorage::updateTrainSampleMark(const int& index, te::SampleMark& samplemark)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<te::StdU8String,te::SampleMark>("TrainSampleMark",samplemark) });
}

bool teAi3DStorage::updateResultSampleMark(const int& index, te::SampleMark& samplemark)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<te::StdU8String,te::SampleMark>("ResultSampleMark",samplemark) });
}

bool teAi3DStorage::clearTrainSampleMark(const int& index)
{
	te::SampleMark sampleMark = getSelectResultFormSampleMarkTable(index, "TrainSampleMark");
	sampleMark.gtDataSet.clear();
	return ResourceTable->updateRecord(index + 1, { std::pair < te::StdU8String,te::SampleMark>("TrainSampleMark",sampleMark) });
}

bool teAi3DStorage::clearAllTestSampleMark()
{
	bool ret = true;
	for (int i = 0; i < currentLoadImageNum; ++i) {
		ret = ResourceTable->updateRecord(i + 1, { std::pair < te::StdU8String,te::SampleMark>("ResultSampleMark",te::SampleMark()) });
	}
	return ret;
}

bool teAi3DStorage::updateShrinkageChart(const int& index, const std::string& filepath)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<te::StdU8String,te::StdU8String>("ShrinkageChartPath",static_cast<te::StdU8String>(filepath)) });
}

bool teAi3DStorage::updatePointCloud(const int& index, const std::string& filepath)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<te::StdU8String,te::StdU8String>("PointCloudPath",static_cast<te::StdU8String>(filepath)) });
}

bool teAi3DStorage::updateTrainGtFilePath(const int& index, const std::string& filepath)
{
	return GtTable->updateRecord(index + 1, { std::pair<te::StdU8String,te::StdU8String>("TrainGtFilePath",static_cast<te::StdU8String>(filepath)) });
}

bool teAi3DStorage::updateResultGtFilePath(const int& index, const std::string& filepath)
{
	return GtTable->updateRecord(index + 1, { std::pair<te::StdU8String,te::StdU8String>("ResultGtFilePath",static_cast<te::StdU8String>(filepath)) });
}

bool teAi3DStorage::clearAllPointCloud()
{
	QDir currentDir = QString::fromStdString(GetCurrentPath());
	QStringList filters;
	filters << "*.pcd";
	QFileInfoList fileList = currentDir.entryInfoList(filters, QDir::Files);

	bool res = false;
	foreach(QFileInfo fileInfo, fileList) {
		QString filePath = fileInfo.absoluteFilePath();
		res = currentDir.remove(filePath);
		if (!res)
			return res;
	}
	return res;
}

bool teAi3DStorage::clearAllThumbnail()
{
	QDir currentDir = QString::fromStdString(GetCurrentPath());
	QStringList filters;
	filters << "*.bmp";
	QFileInfoList fileList = currentDir.entryInfoList(filters, QDir::Files);

	bool res = false;
	foreach(QFileInfo fileInfo, fileList) {
		QString filePath = fileInfo.absoluteFilePath();
		res = currentDir.remove(filePath);
		if (!res)
			return res;
	}
	return res;
}

void teAi3DStorage::setCurrentLoadImageNum(const int& num)
{
	currentLoadImageNum = num;
}

std::string teAi3DStorage::GetCurrentPath()
{
	QString res = QDir::currentPath() + "/workspace/";
	return res.toStdString();
}

bool teAi3DStorage::isImageAlreadyExist(std::string filepath)
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
