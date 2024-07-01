#pragma once
#define _TEAIMODEL_H_

#include <QObject>
#include <memory>
#include <QMap>

#include"teAiExTypes.h"
#include"teDataTypeRegistration.h"

TE_BEGIN_NAMESPACE

RTTR_REGISTRATION
{
	TE_REGISTER_TYPE(ConnectedRegion)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(ConnectedRegion, polygons)
	TE_REGISTER_END

	TE_REGISTER_TYPE(AiInstance)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(AiInstance, id)
	TE_REGISTER_PROPERTY(AiInstance, area)
	TE_REGISTER_PROPERTY(AiInstance, angle)
	TE_REGISTER_PROPERTY(AiInstance, probit)
	TE_REGISTER_PROPERTY(AiInstance, name)
	TE_REGISTER_PROPERTY(AiInstance, contour)
	TE_REGISTER_END

	TE_REGISTER_TYPE(SampleMark)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(SampleMark, gtDataSet)
	TE_REGISTER_PROPERTY(SampleMark, localMask)
	TE_REGISTER_END
}

TE_END_NAMESPACE

class IDataStore;
class QString;

class teAiModel
{
public:
	teAiModel(std::unique_ptr<IDataStore> dataStore);
	~teAiModel();

public:
	std::vector<std::string> getShrinkageChart();
	std::vector<std::string> getOriginImage();
	std::vector<std::string> getPointCloud();
	std::string getSelectShrinkageChart(int index);
	std::string getCurrentShrinkageChart();
	std::string getSelectOriginImage(int index);
	std::string getCurrentOriginImage();
	std::string getSelectPointCloud(int index);
	std::string getCurrentPointCloud();
	std::string getSelectTrainGt(int index);
	std::string getSelectResultGt(int index);
	te::SampleMark getSelectTrainSampleInfo(int index);
	te::SampleMark getSelectResultSampleInfo(int index);
	te::SampleMark getCurrentTrainSampleInfo();
	te::SampleMark getCurrentResultSampleInfo();
	QMap<QString, int> getCurrentTrainMarksNumber();
	QMap<QString, int> getCurrentResultMarksNumber();

	int getCurrentImageWidth();
	int getCurrentImageHeight();

	bool updateTrainSampleMark(int index, te::SampleMark& samplemark);
	bool updateResultSampleMark(int index, te::SampleMark& samplemark);
	bool updateCurrentTrainSampleMark(te::SampleMark& samplemark);
	bool updateCurrentResultSampleMark(te::SampleMark& samplemark);
	bool updateShrinkageChart(int index, std::string& filepath);
	bool updatePointCloud(int index, std::string& filepath);
	bool updateTrainGtFilePath(int index, std::string& filepath);
	bool updateResultGtFilePath(int index, std::string& filepath);

	bool isImageAlreadyExist(std::string filepath);

	bool clearCurrentTrainSampleMark();
	bool clearAllTestSampleMark();
	bool clearCurrentPointCloudAndThumbnail();
	bool clearAllPointCloud();
	bool clearAllThumbnail();
	void DropAllTables();

	void getTrainSamples(std::vector<te::SampleInfo>* trainSamples);
	void getResultSamples(std::vector<te::SampleInfo>* resultSamples);

	void InitThreasholds(int size);

	void InvalidPointThresholdsChange(double threshold);
	void InvalidPointThresholdChange(double threshold);
	double getSelectInvalidPointThreshold(int index);
	double getCurrentInvalidPointThreshold();
	void addInvalidPointThreshold(int index, double threshold);

	void ValidPointThresholdsChange(double threshold);
	void ValidPointThresholdChange(double threshold);
	double getSelectValidPointThreshold(int index);
	double getCurrentValidPointThreshold();
	void addValidPointThreshold(int index,double threshold);

	QString GetCurrentPath();

	void LoadTrainingImages(const QStringList& filePaths);
	int getCurrentLoadImageNum();
	void setCurrentIndex(int num);
	int getCurrentIndex();


private:
	std::unique_ptr<IDataStore> m_dataStore;

	int currentIndex;
	int currentLoadImageNum;
	std::vector<double> InvalidPointThresholds;
	std::vector<double> ValidPointThresholds;
};