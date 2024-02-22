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

using namespace te;

typedef std::vector<std::vector<std::vector<cv::Point>>> Contours;
typedef std::vector<std::vector<std::vector<std::vector<cv::Point>>>> ContoursSet;

class AiModelInterface  : public QThread
{
	Q_OBJECT

public:
	AiModelInterface(QThread *parent = nullptr);
	~AiModelInterface();

	static int teException(void* pParam, AiStatus eStatus);
	static void teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param);
	static void teAiInferResult(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam);

	void ParameterSettings(int mode, std::vector<te::SampleInfo>& trainSamples, const char* modelpath, bool halfPrecise = false, DeviceType deviceType = E_GPU);

	void InitTrainConfig(
		int batchsize, int patchwidth, int patchheight, int receptiveField_A,
		int trainintercnt, int savefrequency, ToolType tooltype,
		TrainMode etrainmode, LocateType locatetype,
		int locatesize, std::string netname, te::BaseType DType,
		int Channel, int HeapID
	);

	void InitTestConfig(
		int maxbatchsize,int batchsize,int maxcontourcount,
		int maxcontourpointcount,int maxinnercontourcount,
		int deviceid,te::DeviceType devicetype,te::ComputePrecision precision
	);

	ContoursSet* getTrainContoursSet();
protected:
	void run();

public slots:
	void trainModel(std::vector<te::SampleInfo>& trainSamples);
	void testModel(std::vector<te::SampleInfo>& trainSamples);

signals:
	void StartInitTrainConfigSignal();
	void StartInitTestConfigSignal();
	void TestingCompleted();

private:
	int mode = 0; //0 «—µ¡∑£¨1 «≤‚ ‘
	
	const char* modelPath;
	bool halfPrecise;
	DeviceType deviceType;

	Contours m_contours;
	ContoursSet m_contoursSet;

public:
	TrainConfig config;
	std::vector<std::string> netNames;
	int DeviceID;

	AiStatus status;
	te::ModelInfer infer_;
	ContourDesc contrDesc;
	DeviceInfo devInfo;
};