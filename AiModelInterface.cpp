#include "AiModelInterface.h"
#include <QDebug>
#include "DataTransmission.h"

AiModelInterface::AiModelInterface(QThread *parent)
	: QThread(parent)
{
}

AiModelInterface::~AiModelInterface()
{
}

static AiResult m_InferResult;
int AiModelInterface::teException(void* pParam, AiStatus eStatus)
{
	if (eStatus != E_Success)
	{
		printf("teException.error %d\n", eStatus);
	}
	return 0;
}

void AiModelInterface::teTrainStateCallBack(AiStatus status, TrainState& stateinfo, void* param)
{
	printf("**teTrainStateCallBack**\n");
	if (status != E_Success)
	{
		printf("train_.error %d\n", status);
		auto cv = (std::condition_variable*)param;
		cv->notify_all();
	}

	printf("iter: %d  loss: %f  pacc: %f\n", stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);
	//emit DataUpdate(stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);
	DataTransmission::GetInstance()->setData(stateinfo.iteration, stateinfo.fAvgLoss, stateinfo.fPosAcc);

	if (stateinfo.fProgress > 0.999999f)
	{
		auto cv = (std::condition_variable*)param;
		cv->notify_all();
	}
}

void AiModelInterface::teAiInferResult(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam)
{
	if (inferResult.size() > 0)
		std::cout << inferResult[0].name << std::endl;

	{
		m_InferResult = inferResult;
		auto cv = (std::condition_variable*)pCallbackParam;
		cv->notify_all();
	}
}

void AiModelInterface::ParameterSettings(int mode, std::vector<te::SampleInfo>& trainSamples, const char* modelpath, bool halfPrecise, DeviceType deviceType)
{
	this->mode = mode;
	this->trainSamples = trainSamples;
	size_t length = strlen(modelpath);
	this->modelPath = new char[length+1];
	memcpy(const_cast<char*>(modelPath), modelpath, strlen(modelpath)+1);
	//this->modelPath = modelpath;
	this->halfPrecise = halfPrecise;
	this->deviceType = deviceType;
	
}

void AiModelInterface::run()
{
	if (!mode) {
		trainModel(trainSamples);
	}
	else {
		testModel(trainSamples);
	}
}

void AiModelInterface::trainModel(std::vector<te::SampleInfo>& trainSamples)
{
	std::condition_variable cv;
	AiStatus status;

	ToolType toolType = ToolType::E_PixelDetect_Tool;
	std::vector<std::string> netNames = Training::getNetworkList(toolType);

	TrainConfig config;
	config.batchSize = 6;
	config.patchWidth = 0;
	config.patchHeight = 0;
	config.receptiveField_A = 64;
	config.receptiveField_B = 2;
	config.trainIterCnt = 128;
	config.saveFrequency = 3;
	config.eToolType = toolType;
	config.eTrainMode = TrainMode::E_TE_RESET;//E_TE_INHERITANCE
	config.locateType = LocateType::E_LocateRectangle;
	config.locateSide = 0;
	config.netName = netNames[1];//"default";//
	config.augmentHandle = nullptr;
	config.modelPath = modelPath;
	config.sampleDesc.resize(1);
	config.sampleDesc[0].dtype = te::BaseType::E_Float32;
	config.sampleDesc[0].channel = 1;
	config.sampleDesc[0].heapId = 0;


	config.augmentHandle = new AugmentProcess();

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

	config.augmentHandle = nullptr;

	std::string initInfo;

	Training train_;
	train_.setDeviceInfo({ E_GPU, 0 });
	train_.setTrainConfig(config);
	status = train_.init(trainSamples, initInfo);

	if (status != E_Success)
	{
		printf("train_.init %d\n", status);
		system("Pause");
		return;
	}

	train_.start(teTrainStateCallBack, &cv);
	//
	{
		std::mutex lock;
		std::unique_lock locker(lock);
		cv.wait(locker);
	}

	train_.stop();
}

void AiModelInterface::testModel(std::vector<te::SampleInfo>& trainSamples)
{
	std::condition_variable cv;
	AiStatus status;

	te::ModelInfer infer_;
	status = infer_.setMaxBatchSize(1);
	status = infer_.setBatchSize(1);
	ContourDesc contrDesc = { 12, 5, 256 };// { 1, 0, 4 };
	status = infer_.setCoutourDesc(contrDesc);
	DeviceInfo devInfo = { deviceType, 0 };
	status = infer_.setComputeDesc(devInfo);
	status = infer_.setModelPath(modelPath);
	status = infer_.setPrecisionType(halfPrecise ? ComputePrecision::E_FP16 : ComputePrecision::E_FP32);
	status = infer_.setExceptionFunc(teException, nullptr);

	int maxImageW = 0, maxImageH = 0;
	for (size_t i = 0; i < trainSamples.size(); i++)
	{
		if (maxImageW < trainSamples[i].sampleData.roi.width)
			maxImageW = trainSamples[i].sampleData.roi.width;
		if (maxImageH < trainSamples[i].sampleData.roi.height)
			maxImageH = trainSamples[i].sampleData.roi.height;
	}

	/*cv::Mat image = cv::imread("1.bmp", 1);

	if (maxImageW < image.cols)
		maxImageW = image.cols;
	if (maxImageH < image.rows)
		maxImageH = image.rows;

	SampleInfo si;
	si.sampleData.imageMatrix.push_back(te::Image(image.cols, image.rows, te::Image::Format::E_BGR, image.data, image.step));
	si.sampleData.roi = te::Rect(0, 0, image.cols, image.rows);
	trainSamples.clear();
	trainSamples.push_back(si);*/

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

	for (size_t i = 0; i < trainSamples.size(); i++)
	{
		std::mutex lock;
		std::unique_lock locker(lock);

		auto startTimePoint = currentSteadyTimePoint();
		infer_.pushSampleData(trainSamples[i].sampleData, &cv);

		cv.wait(locker);
		auto endTimePoint = currentSteadyTimePoint();

		std::cout << "time:" << calculateTimeDiff<Milliseconds>(startTimePoint, endTimePoint) << std::endl;

		cv::Mat image = trainSamples[i].sampleData.imageMatrix[0].getCvMat();

		for (int j = 0; j < m_InferResult.size(); j++)
		{
			auto& srcpoly = m_InferResult[j].contour.polygons;

			std::vector<std::vector<cv::Point>> polygons;

			for (int k = 0; k < srcpoly.size(); k++)
			{
				std::vector<cv::Point> intPoly;
				for (int m = 0; m < srcpoly[k].size(); m++)
				{
					auto pt2f = srcpoly[k][m];
					cv::Point pt2i((int)pt2f.x, (int)pt2f.y);
					intPoly.push_back(pt2i);
				}
				polygons.push_back(intPoly);
			}

			//cv::fillPoly(image, polygons, { 0,255,0 });
			cv::polylines(image, polygons, 1, { 0,255,0 });
		}
		cv::imwrite("result.bmp", image);
		cv::resize(image, image, cv::Size(), 0.3, 0.3);
		cv::imshow("result", image);
		cv::waitKey(0);
	}
}
