#pragma once

#include <QObject>
#include "teSqlite3DataModel.h"

#include"teAiExTypes.h"
#include"teDataTypeRegistration.h"
#include "teLabelBrowser.h"

#include <QColor>
#include <QVBoxLayout>
#include <QMap>

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

class QVBoxLayout;

class teDataStorage  : public QObject
{
	Q_OBJECT

public:
	static teDataStorage* getInstance();
	static void destroy();

	void InitDatabase();
	void displayUIInWidget(QVBoxLayout* layout);
public:
	int getCurrentIndex();
	int getCurrentLoadImageNum();
	
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

	QString GetCurrentPath();

	int getCurrentImageWidth();
	int getCurrentImageHeight();

	bool insertOriginImage(std::string filepath);
	bool insertGtFilePath(std::string filepath);

	bool updateTrainSampleMark(int index,te::SampleMark& samplemark);
	bool updateResultSampleMark(int index,te::SampleMark& samplemark);
	bool updateCurrentTrainSampleMark(te::SampleMark& samplemark);
	bool updateCurrentResultSampleMark(te::SampleMark& samplemark);
	bool updateShrinkageChart(int index,std::string& filepath);
	bool updatePointCloud(int index,std::string& filepath);
	bool updateTrainGtFilePath(int index,std::string& filepath);
	bool updateResultGtFilePath(int index,std::string& filepath);

	/// <summary>
	/// 更新标记浏览器中标记的数目
	/// </summary>
	/// <param name="nameCounts">各标记的数目</param>
	void updateTrainWidget(QMap<QString, int>& nameCounts);
	/// <summary>
	/// 更新标记浏览器中测试标的数目
	/// </summary>
	/// <param name="nameCounts">各测试标的数目</param>
	void updateResultWidget(QMap<QString, int>& nameCounts);


	bool clearCurrentTrainSampleMark();
	bool clearAllTestSampleMark();

	bool DeleteCurrentPointCloudAndThumbnail();

	bool isOriginImage(std::string filepath);

	QString getCurrentLabelCategory();
	QColor getCurrentLabelColor();

	void getTrainSamples(std::vector<te::SampleInfo>* trainSamples);
	void getResultSamples(std::vector<te::SampleInfo>* resultSamples);

	void setDataDuringTraining(int iteration, float fAvgLoss, float fPosAcc);
	void DropAllTables();

	QColor FindContentColor(const QString& searchString);

	void InitThreasholds(int size);

	void InvalidPointThresholdsChange(double threshold);
	void InvalidPointThresholdChange(double threshold);
	double getSelectInvalidPointThreshold(int index);
	double getCurrentInvalidPointThreshold();
	void addInvalidPointThreshold(double threshold);

	void ValidPointThresholdsChange(double threshold);
	void ValidPointThresholdChange(double threshold);
	double getSelectValidPointThreshold(int index);
	double getCurrentValidPointThreshold();
	void addValidPointThreshold(double threshold);
private:
	std::string getSelectResultFormResourceTable(int index, std::string keyword);
	std::vector<std::string> getResultFromResourceTable(std::string keyword);
	std::pair<std::string,std::string> getSelectResultFormGtTable(int index);
	te::SampleMark getSelectResultFormSampleMarkTable(int index, std::string keyword);
	
public slots:
	void setCurrentIndex(int index);
	void setCurrentLoadImageNum(int num);
	void LoadTrainingImages(const QStringList& filePaths);
	void currentRowChange(const QString& content, const QColor& fontColor);
	void updateResultOperate();
signals:
	void sig_teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void sig_LoadTrainImagesComplete();
	void sig_currentLabelChange(const QString& content, const QColor& fontColor);
	void sig_DataChangeDuringTraining(int iteration, float fAvgLoss, float fPosAcc);
	void sig_ColorChanged();
	void sig_updateCurrentTrainSampleMark();

private:

	int currentIndex;
	int currentLoadImageNum;

	std::vector<double> InvalidPointThresholds;
	std::vector<double> ValidPointThresholds;
	
	std::unique_ptr<te::Sqlite3DB> db;
	std::unique_ptr<te::Sqlite3DataModel> ResourceTable;
	std::unique_ptr<te::Sqlite3DataModel> GtTable;

	QString currentCategory;
	QColor currentColor;

	teLabelBrowser* m_teLabelBrowser;
private:
	static teDataStorage* instance;

	teDataStorage(QObject* parent = nullptr);
	~teDataStorage();
	teDataStorage(const teDataStorage&);
	teDataStorage& operator=(const teDataStorage&);

	class Garbo
	{
	public:
		~Garbo()
		{
			teDataStorage::destroy();
		}
	};

	static Garbo tmp;
};
