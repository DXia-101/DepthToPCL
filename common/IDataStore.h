#pragma once

#include <string>
#include <vector>
#include <QColor>
#include "teAiExTypes.h"

namespace te {

	class IDataStore
	{
	public:
		virtual ~IDataStore() {}

		virtual std::vector<std::string> getResultFromResourceTable(const std::string& keyword) = 0;
		virtual std::string getSelectResultFormResourceTable(const int& index, const std::string& keyword) = 0;
		virtual std::pair<std::string, std::string> getSelectResultFormGtTable(const int& index) = 0;
		virtual SampleMark getSelectResultFormSampleMarkTable(const int& index, const std::string& keyword) = 0;
		virtual void DropAllTables() = 0;
		virtual bool insertOriginImage(const std::string& filepath) = 0;
		virtual bool insertGtFilePath(const std::string& filepath) = 0;
		virtual bool updateTrainSampleMark(const int& index, te::SampleMark& samplemark) = 0;
		virtual bool updateResultSampleMark(const int& index, te::SampleMark& samplemark) = 0;
		virtual bool clearTrainSampleMark(const int& index) = 0;
		virtual bool updateShrinkageChart(const int& index, const std::string& filepath) = 0;
		virtual bool updatePointCloud(const int& index, const std::string& filepath) = 0;
		virtual bool updateTrainGtFilePath(const int& index, const std::string& filepath) = 0;
		virtual bool updateResultGtFilePath(const int& index, const std::string& filepath) = 0;
		virtual bool clearAllPointCloud() = 0;
		virtual bool clearAllShrinkageChart() = 0;
		virtual std::string GetCurrentPath() = 0;
		virtual bool isImageAlreadyExist(std::string filepath) = 0;
	};

}