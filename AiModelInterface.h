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

#include"teImage.h"
#include"teRapidjsonObjectTree.h"
#include "teTimer.h"

using namespace te;

class AiModelInterface  : public QThread
{
	Q_OBJECT

public:
	AiModelInterface(QThread *parent = nullptr);
	~AiModelInterface();

	static int teException(void* pParam, AiStatus eStatus);
	static void teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param);
	static void teAiInferResult(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam);

	void ParameterSettings(int mode, std::vector<te::SampleInfo>& trainSamples, const char* modelPath, bool halfPrecise = false, DeviceType deviceType = E_GPU);

protected:
	void run();

public slots:
	void trainModel(std::vector<te::SampleInfo>& trainSamples, const char* modelPath);
	void testModel(std::vector<te::SampleInfo>& trainSamples, const char* modelPath, bool halfPrecise, DeviceType deviceType);

private:
	int mode = 0; //0 «—µ¡∑£¨1 «≤‚ ‘
	std::vector<te::SampleInfo> trainSamples;
	const char* modelPath;
	bool halfPrecise;
	DeviceType deviceType;
};