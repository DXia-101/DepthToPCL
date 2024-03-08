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

#include <QThread>

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

#include "TrainParam.h"

using namespace te;

typedef std::vector<std::vector<std::vector<cv::Point>>> Contours;
typedef std::vector<std::vector<std::vector<std::vector<cv::Point>>>> ContoursSet;

enum RunMode {
	trainMode,
	testMode,
};

class AiModelInterface : public QThread
{
	Q_OBJECT

public:
	AiModelInterface(QThread* parent = nullptr);
	~AiModelInterface();

	static int teException(void* pParam, AiStatus eStatus);
	static void teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param);
	static void teAiInferResult(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam);

	void TrainParameterSettings(const char* modelpath);
	void TestParameterSettings(const char* modelpath);
	void ParameterSettings(const char* modelpath);

	void InitTestConfig(
		int maxbatchsize, int batchsize, int maxcontourcount,
		int maxcontourpointcount, int maxinnercontourcount,
		int deviceid, te::DeviceType devicetype, te::ComputePrecision precision
	);

protected:
	void run();

public slots:
	void trainModel(std::vector<te::SampleInfo>& trainSamples);
	void testModel(std::vector<te::SampleInfo>& trainSamples);
	void InitTrainConfig(te::TrainParam* para);

signals:
	void sig_TestingCompleted();

private:
	RunMode mode = trainMode;
	const char* modelPath;

public:
	TrainConfig config;

	int DeviceID;

	AiStatus status;
	te::ModelInfer infer_;
	ContourDesc contrDesc;
	DeviceInfo devInfo;

	std::vector<te::SampleInfo> m_Trainsamples;
	std::vector<te::SampleInfo> m_Resultsamples;
	std::vector<te::AiResult> m_SampleMark;
};