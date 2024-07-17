#include "teAi3DStorage.h"
#include <QString>
#include <QDir>
using namespace te;
Ai3DStorage::Ai3DStorage()
{
	QString path = QDir::currentPath() + "/workspace/";
	QDir dir;
	if (!dir.exists(path)) {
		dir.mkpath(path);
	}

	path = QDir::currentPath() + "/workspace/teAi3D.db";
	db = Sqlite3DB::open(path.toStdString());

	ResourceTable = db->createDataModel("Resource",
		{
			makeFieldType<StdU8String>("OriginImagePath"),
			makeFieldType<StdU8String>("ShrinkageChartPath"),
			makeFieldType<StdU8String>("PointCloudPath"),
			makeFieldType<SampleMark>("TrainSampleMark"),
			makeFieldType<SampleMark>("ResultSampleMark"),
		});

	GtTable = db->createDataModel("TrainGt",
		{
			makeFieldType<StdU8String>("TrainGtFilePath"),
			makeFieldType<StdU8String>("ResultGtFilePath"),
		});
}

std::vector<std::string> Ai3DStorage::getResultFromResourceTable(const std::string& keyword)
{
	std::vector<std::string> result;
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource";
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			StdU8String path;
			dataQuery->getRecord_t(path);
			result.push_back(path);
		}
	}
	return result;
}

std::string Ai3DStorage::getSelectResultFormResourceTable(const int& index, const std::string& keyword)
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

std::pair<std::string, std::string> Ai3DStorage::getSelectResultFormGtTable(const int& index)
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

SampleMark Ai3DStorage::getSelectResultFormSampleMarkTable(const int& index, const std::string& keyword)
{
	auto dataQuery = db->createDataQuery();
	std::string sqlquery = "select " + keyword + " from Resource where ID = " + std::to_string(index + 1);
	SampleMark result;
	if (dataQuery->query(const_cast<char*>(sqlquery.c_str()))) {
		while (dataQuery->next()) {
			dataQuery->getRecord_t(result);
		}
	}
	return result;
}

void Ai3DStorage::DropAllTables()
{
	db->dropDataModel("Resource");

	ResourceTable = db->createDataModel("Resource",
		{
			makeFieldType<StdU8String>("OriginImagePath"),
			makeFieldType<StdU8String>("ShrinkageChartPath"),
			makeFieldType<StdU8String>("PointCloudPath"),
			makeFieldType<SampleMark>("TrainSampleMark"),
			makeFieldType<SampleMark>("ResultSampleMark"),
		});
}

bool Ai3DStorage::insertOriginImage(const std::string& filepath)
{
	StdU8String originimage = filepath;
	StdU8String shrinkagechart = "";
	StdU8String pointcloud = "";
	SampleMark train;
	SampleMark result;

	ResourceTable->transaction();
	ResourceTable->insertRecord_t(-1, originimage, shrinkagechart, pointcloud, train, result);
	return ResourceTable->commit();
}

bool Ai3DStorage::insertGtFilePath(const std::string& filepath)
{
	StdU8String GtFilePath = filepath;

	ResourceTable->transaction();
	ResourceTable->insertRecord_t(-1, GtFilePath, "");
	return ResourceTable->commit();
}

bool Ai3DStorage::updateTrainSampleMark(const int& index, SampleMark& samplemark)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<StdU8String,SampleMark>("TrainSampleMark",samplemark) });
}

bool Ai3DStorage::updateResultSampleMark(const int& index, SampleMark& samplemark)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<StdU8String,SampleMark>("ResultSampleMark",samplemark) });
}

bool Ai3DStorage::clearTrainSampleMark(const int& index)
{
	SampleMark sampleMark = getSelectResultFormSampleMarkTable(index, "TrainSampleMark");
	sampleMark.gtDataSet.clear();
	return ResourceTable->updateRecord(index + 1, { std::pair < StdU8String,SampleMark>("TrainSampleMark",sampleMark) });
}

bool Ai3DStorage::updateShrinkageChart(const int& index, const std::string& filepath)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<StdU8String,StdU8String>("ShrinkageChartPath",static_cast<StdU8String>(filepath)) });
}

bool Ai3DStorage::updatePointCloud(const int& index, const std::string& filepath)
{
	return ResourceTable->updateRecord(index + 1, { std::pair<StdU8String,StdU8String>("PointCloudPath",static_cast<StdU8String>(filepath)) });
}

bool Ai3DStorage::updateTrainGtFilePath(const int& index, const std::string& filepath)
{
	return GtTable->updateRecord(index + 1, { std::pair<StdU8String,StdU8String>("TrainGtFilePath",static_cast<StdU8String>(filepath)) });
}

bool Ai3DStorage::updateResultGtFilePath(const int& index, const std::string& filepath)
{
	return GtTable->updateRecord(index + 1, { std::pair<StdU8String,StdU8String>("ResultGtFilePath",static_cast<StdU8String>(filepath)) });
}

bool Ai3DStorage::clearAllPointCloud()
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

bool Ai3DStorage::clearAllShrinkageChart()
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

std::string Ai3DStorage::GetCurrentPath()
{
	QString res = QDir::currentPath() + "/workspace/";
	return res.toStdString();
}

bool Ai3DStorage::isImageAlreadyExist(std::string filepath)
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
