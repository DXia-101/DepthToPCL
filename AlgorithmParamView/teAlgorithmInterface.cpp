#include "teAlgorithmInterface.h"
#include <QDebug>

#include "teAiModel.h"
#include "Transfer_Function.h"
#include "teAiExtypes.h"
#include "teTrainStatisticsChart.h"

teAlgorithmInterface::teAlgorithmInterface(QThread* parent)
	: QThread(parent)
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

	//printf("iter: %d  loss: %f  pacc: %f\n", stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);
	//emit DataUpdate(stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);
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
	this->mode = trainMode;
	ParameterSettings(modelpath);
}

void teAlgorithmInterface::TestParameterSettings(const char* modelpath)
{
	this->mode = testMode;
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

void teAlgorithmInterface::run()
{
	if (mode == trainMode) {
		std::vector<te::SampleInfo> m_Trainsamples;
		m_teAiModel->getTrainSamples(&m_Trainsamples);
		trainModel(m_Trainsamples);
	}
	else if (mode == testMode) {
		std::vector<te::SampleInfo> m_Resultsamples;
		m_teAiModel->getTrainSamples(&m_Resultsamples);
		testModel(m_Resultsamples);
	}
}

void teAlgorithmInterface::trainModel(std::vector<te::SampleInfo>& trainSamples)
{
	std::condition_variable cv;
	AiStatus status;
	/*auto gauss = std::make_shared<GaussianTransform>();
	gauss->mean.resize(1);
	gauss->mean[0].start = 0.00f;
	gauss->mean[0].end = 0.00f;
	gauss->variance.resize(1);
	gauss->variance[0].start = 0.00f;
	gauss->variance[0].end = 5.00f;
	config.augmentHandle->algorithmProcess.push_back(gauss);

	auto angle = std::make_shared<RandomAngleTransform>();
	angle->ranges.resize(1);
	angle->ranges[0].start = -180;
	angle->ranges[0].end = 180;
	config.augmentHandle->algorithmProcess.push_back(angle);

	auto saltpepper = std::make_shared<SaltAndPepperTransform>();
	saltpepper->ratio.resize(1);
	saltpepper->ratio[0].start = 0.00f;
	saltpepper->ratio[0].end = 0.03f;
	config.augmentHandle->algorithmProcess.push_back(saltpepper);

	auto brictr = std::make_shared<BrightContrastTransform>();
	brictr->bright.resize(1);
	brictr->bright[0].start = 0.95f;
	brictr->bright[0].end = 1.05f;
	brictr->contrast.resize(1);
	brictr->contrast[0].start = 0.95f;
	brictr->contrast[0].end = 1.05f;
	config.augmentHandle->algorithmProcess.push_back(brictr);

	auto satur = std::make_shared<SaturationTransform>();
	satur->saturation.resize(1);
	satur->saturation[0].start = 0.00f;
	satur->saturation[0].end = 10.0f;
	config.augmentHandle->algorithmProcess.push_back(satur);

	auto hue = std::make_shared<HueTransform>();
	hue->hue.resize(1);
	hue->hue[0].start = 0.00f;
	hue->hue[0].end = 10.0f;
	config.augmentHandle->algorithmProcess.push_back(hue);

	auto defini = std::make_shared<DefinitionTransform>();
	defini->percent.resize(1);
	defini->percent[0].start = 0.00f;
	defini->percent[0].end = 0.05f;
	config.augmentHandle->algorithmProcess.push_back(defini);

	auto translat = std::make_shared<RandomTranslateTransform>();
	translat->transx.resize(1);
	translat->transx[0].start = -2;
	translat->transx[0].end = 2;
	translat->transy.resize(1);
	translat->transy[0].start = -2;
	translat->transy[0].end = 2;
	config.augmentHandle->algorithmProcess.push_back(translat);

	auto scale = std::make_shared<RandomScaleTransform>();
	scale->xRatio.resize(1);
	scale->xRatio[0].start = 0.95f;
	scale->xRatio[0].end = 1.05f;
	scale->yRatio.resize(1);
	scale->yRatio[0].start = 0.95f;
	scale->yRatio[0].end = 1.05f;
	config.augmentHandle->algorithmProcess.push_back(scale);

	auto flip = std::make_shared<RandomFlipTransform>();
	flip->flipmode = te::TeFlipMode::E_BOTH_FLIP;
	config.augmentHandle->algorithmProcess.push_back(flip);*/
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
