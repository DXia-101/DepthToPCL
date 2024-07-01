#include "teAlgorithmInterface.h"
#include <QDebug>

#include "teAiModel.h"
#include "Transfer_Function.h"
#include "teAiExtypes.h"
#include "teTrainStatisticsChart.h"

teAlgorithmInterface::teAlgorithmInterface(QObject* parent)
	: QObject(parent)
{
	config.sampleDesc.resize(1);
}

teAlgorithmInterface::~teAlgorithmInterface()
{
}

static AiResult m_InferResult;

int teAlgorithmInterface::teException(void* pParam, AiStatus eStatus)
{
	if (eStatus != E_Success)
	{
		printf("teException.error %d\n", eStatus);
	}
	return 0;
}

void teAlgorithmInterface::teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param)
{
	printf("**teTrainStateCallBack**\n");
	if (status != E_Success)
	{
		printf("train_.error %d\n", status);
		auto cv = (std::condition_variable*)param;
		cv->notify_all();
	}

	teTrainStatisticsChart::getInstance()->ReceiveData(stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);

	if (stateinfo.fProgress > 0.999999f)
	{
		auto cv = (std::condition_variable*)param;
		cv->notify_all();
	}
}

void teAlgorithmInterface::teAiInferResult(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam)
{
	if (inferResult.size() > 0)
		std::cout << inferResult[0].name << std::endl;

	{
		m_InferResult = inferResult;
		auto cv = (std::condition_variable*)pCallbackParam;
		cv->notify_all();
	}
}

void teAlgorithmInterface::TrainParameterSettings(const char* modelpath)
{
	ParameterSettings(modelpath);
}

void teAlgorithmInterface::TestParameterSettings(const char* modelpath)
{
	ParameterSettings(modelpath);
}

void teAlgorithmInterface::ParameterSettings(const char* modelpath) {
	size_t length = strlen(modelpath);
	this->modelPath = new char[length + 1];
	memcpy(const_cast<char*>(modelPath), modelpath, strlen(modelpath) + 1);
}

void teAlgorithmInterface::setteAiModel(teAiModel* model)
{
	m_teAiModel = model;
}

void teAlgorithmInterface::InitTrainConfig(te::TrainParamRegister* para)
{
	ToolType toolType = ToolType::E_PixelDetect_Tool;

	config.batchSize = para->TrainBatchSize;
	config.patchWidth = para->PatchWidth;
	config.patchHeight = para->PatchHeight;
	config.receptiveField_A = para->receptiveField;
	config.receptiveField_B = 2;
	config.trainIterCnt = para->trainIterCnt;
	config.saveFrequency = para->saveFrequency;
	config.eToolType = toolType;
	config.eTrainMode = TrainMode::E_TE_RESET;
	config.locateType = para->eLocateType;
	config.locateSide = para->locateSide;
	config.netName = para->netName;
	config.augmentHandle = nullptr;
	config.modelPath = modelPath;
	config.sampleDesc.resize(1);
	config.sampleDesc = para->sampleDesc;
	config.augmentHandle = new AugmentProcess();
	config.augmentHandle = nullptr;

	DeviceID = para->DeviceID;
}

void teAlgorithmInterface::InitTestConfig(te::TestParamRegister* para)
{
	status = infer_.setMaxBatchSize(para->maxbatchsize);
	status = infer_.setBatchSize(para->batchsize);
	status = infer_.setCoutourDesc(para->contourdesc);
	status = infer_.setComputeDesc(para->deviceinfo);
	status = infer_.setModelPath(modelPath);
	status = infer_.setPrecisionType(para->precision);
	status = infer_.setExceptionFunc(teException, nullptr);
}

void teAlgorithmInterface::StopTrain()
{
	train_.stop();
}

void teAlgorithmInterface::StartTrain()
{
	std::vector<te::SampleInfo> m_Trainsamples;
	m_teAiModel->getTrainSamples(&m_Trainsamples);
	trainModel(m_Trainsamples);
}

void teAlgorithmInterface::StartTest()
{
	std::vector<te::SampleInfo> m_Resultsamples;
	m_teAiModel->getTrainSamples(&m_Resultsamples);
	testModel(m_Resultsamples);
}

void teAlgorithmInterface::trainModel(std::vector<te::SampleInfo>& trainSamples)
{
	std::condition_variable cv;
	AiStatus status;
	std::string initInfo;

	train_.setDeviceInfo({ E_GPU, DeviceID });
	train_.setTrainConfig(config);
	status = train_.init(trainSamples, initInfo);

	if (status != E_Success)
	{
		printf("train_.init %d\n", status);
		system("Pause");
		return;
	}

	train_.start(teTrainStateCallBack, &cv);

	{
		std::mutex lock;
		std::unique_lock locker(lock);
		cv.wait(locker);
	}

	train_.stop();
}

void teAlgorithmInterface::testModel(std::vector<te::SampleInfo>& trainSamples)
{
	std::condition_variable cv;

	int maxImageW = 0, maxImageH = 0;
	for (size_t i = 0; i < trainSamples.size(); i++)
	{
		if (maxImageW < trainSamples[i].sampleData.roi.width)
			maxImageW = trainSamples[i].sampleData.roi.width;
		if (maxImageH < trainSamples[i].sampleData.roi.height)
			maxImageH = trainSamples[i].sampleData.roi.height;
	}

	status = infer_.init(maxImageW, maxImageH);
	if (status != E_Success)
	{
		printf("infer_.init %d\n", status);
		system("Pause");
		return;
	}
	NetMemorySize neededMemory;
	status = infer_.calcuMemoryWorkspace(10 * 1024 * (uint64_t)1024 * 1024, 10 * 1024 * (uint64_t)1024 * 1024, neededMemory);
	if (status != E_Success)
	{
		printf("infer_.calcuMemoryWorkspace %d\n", status);
		system("Pause");
		return;
	}

	status = infer_.setShareMemoryAddr(nullptr, 0, nullptr, 0);
	if (status != E_Success)
	{
		printf("infer_.setShareMemoryAddr %d\n", status);
		system("Pause");
		return;
	}

	status = infer_.setResultCallbackFunc(teAiInferResult);

	if (!m_teAiModel->clearAllTestSampleMark())
		return;

	for (size_t i = 0; i < trainSamples.size(); i++)
	{
		std::mutex lock;
		std::unique_lock locker(lock);

		auto startTimePoint = currentSteadyTimePoint();
		infer_.pushSampleData(trainSamples[i].sampleData, &cv);

		cv.wait(locker);
		auto endTimePoint = currentSteadyTimePoint();

		te::SampleMark samplemark;
		samplemark.gtDataSet = m_InferResult;
		m_teAiModel->updateResultSampleMark(i, samplemark);
	}

	emit sig_TestingCompleted();
}
