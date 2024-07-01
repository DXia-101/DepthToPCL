#pragma once
#include <QObject>

#include <opencv2/opencv.hpp>

#include <string.h>

#include <vector>
#include <time.h>
#include <string>
#include <thread>

#ifdef _WINDOWS
#include <Windows.h>

#include <direct.h>
#include <io.h>
#include <process.h>
#include <windows.h>

#undef min
#undef max
#else
#include <dirent.h>
#endif

#include "teTraining.h"
#include "tePrediction.h"
#include "teAugmentation.h"

//#include"teSqlite3ORM.h"

#include "teImage.h"
#include "teRapidjsonObjectTree.h"
#include "teTimer.h"

#include "TrainParamRegister.h"
#include "TestParamRegister.h"


using namespace te;

typedef std::vector<std::vector<std::vector<cv::Point>>> Contours;
typedef std::vector<std::vector<std::vector<std::vector<cv::Point>>>> ContoursSet;

class teAiModel;

class teAlgorithmInterface : public QObject
{
	Q_OBJECT

public:
	teAlgorithmInterface(QObject* parent = nullptr);
	~teAlgorithmInterface();

	static int teException(void* pParam, AiStatus eStatus);
	static void teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param);
	static void teAiInferResult(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam);

	void TrainParameterSettings(const char* modelpath);
	void TestParameterSettings(const char* modelpath);
	void ParameterSettings(const char* modelpath);

	void setteAiModel(teAiModel*);
public slots:
	void trainModel(std::vector<te::SampleInfo>& trainSamples);
	void testModel(std::vector<te::SampleInfo>& trainSamples);
	void InitTrainConfig(te::TrainParamRegister* para);
	void InitTestConfig(te::TestParamRegister* para);
	void StopTrain();
	void StartTrain();
	void StartTest();

signals:
	void sig_TestingCompleted();

private:
	const char* modelPath;
	teAiModel* m_teAiModel;

public:
	TrainConfig config;
	int DeviceID;
	AiStatus status;
	te::ModelInfer infer_;
	Training train_;
};