﻿#pragma once

#include "IDataStore.h"
#include "teSqlite3DataModel.h"
#include <string>
class QString;

namespace te {

	class Ai3DStorage : public IDataStore
	{
	public:
		Ai3DStorage();

		std::vector<std::string> getResultFromResourceTable(const std::string& keyword)override;
		std::string getSelectResultFormResourceTable(const int& index, const std::string& keyword)override;
		std::pair<std::string, std::string> getSelectResultFormGtTable(const int& index)override;
		SampleMark getSelectResultFormSampleMarkTable(const int& index, const std::string& keyword)override;
		void DropAllTables()override;
		bool insertOriginImage(const std::string& filepath)override;
		bool insertGtFilePath(const std::string& filepath)override;
		bool updateTrainSampleMark(const int& index, te::SampleMark& samplemark)override;
		bool updateResultSampleMark(const int& index, te::SampleMark& samplemark)override;
		bool clearTrainSampleMark(const int& index)override;
		bool updateShrinkageChart(const int& index, const std::string& filepath)override;
		bool updatePointCloud(const int& index, const std::string& filepath)override;
		bool updateTrainGtFilePath(const int& index, const std::string& filepath)override;
		bool updateResultGtFilePath(const int& index, const std::string& filepath)override;
		bool clearAllPointCloud()override;
		bool clearAllShrinkageChart()override;
		std::string GetCurrentPath() override;
		bool isImageAlreadyExist(std::string filepath) override;
	private:


	private:
		std::unique_ptr<te::Sqlite3DB> db;
		std::unique_ptr<te::Sqlite3DataModel> ResourceTable;
		std::unique_ptr<te::Sqlite3DataModel> GtTable;

	};
}