/*M/////////////////////////////////////////////////////////////////////
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//                           License Agreement
//            For Establish Deep Computer Vision Library.
//
// Copyright (C) 2017-2047, TruthEyes Corporation, all rights reserved.
/////////////////////////////////////////////////////////////////////////

/**@file     teAiExTypes.h
* @brief     Ai external data types and funtion types.
* @details   Ai external data types and funtion types.
* @author    chengcx
* @date      2023-12-22
* @version   V1.3.3
* @copyright Copyright (c) 2047. HangZhou TruthEyes Corporation.
*/

#pragma once
#include "teGeometricType.h"
#include "teImage.h"

TE_BEGIN_NAMESPACE

#define TeAlign(x, align) (((x) + (align) - 1) / (align) * (align))
#define GENERATE_ENUM(ENUM)     ENUM,
#define GENERATE_ENUM_STR(ENUM) #ENUM,
#define DEF_ENUM_TYPE(NAME, ENUMS)   enum NAME { ENUMS(GENERATE_ENUM) };
#define DEF_ENUM_STRING(NAME, ENUMS) static const char* NAME[] = { ENUMS(GENERATE_ENUM_STR) 0 };

#define AiToolType(elem) elem(E_Location_Tool)\
    elem(E_PixelDetect_Tool)\
    elem(E_Classify_Tool)\
    elem(E_UnSupervised_Tool)\
    elem(E_AngleRegress_Tool)\
    elem(E_Total_Tool)

DEF_ENUM_TYPE(ToolType, AiToolType);

/**indicate the inference device: cpu, gpu, or other device**/
enum DeviceType
{
	E_CPU,///< inference on cpu.
	E_GPU,///< inference on gpu.
};

/**indicate the inference precision**/
enum ComputePrecision
{
	E_FP16,
	E_FP32,
};

/**indicate the executive status of a function call**/
enum AiStatus
{
	E_Success = 0,///< execute successfully.
	E_DeviceNotSupport,///< Unable to find a suitable GPU, or the driver does not meet the requirements(update driver).
	E_InvalidArgument,///< The input parameter is invalid.
	E_OutOfCpuMemory,///< Insufficient memory of CPU.
	E_OutOfGpuMemory,///< Insufficient memory of GPU.
	E_LoadModelFail,///< Loading model failed.
	E_OutOfDetectRange,///< Out of detection range
	E_CurrentDeviceBusy,///< The calling gpu device is currently busy.
	E_ChipNotSupport,///< The calling model is not supported due to encryption.
	E_DllNotMatchModel,///< The model does not match with the DLL.
	E_DllsIncompatible,///< The DLLs isnot consistent.
	E_PathError,///< The specified file could not be found due to the wrong input path.
	E_CompilerNotAllowRun,///< The compiler is not allowed to run in release mode.
	E_NetMisMatch,///< the net is not match with the model path you given.
	E_GetInOutDataError,
	E_OperatorError,///< some error happened when deeplearning operators running.
	E_TrainInputError,///< input parameters of training are erronious.
	E_AllocateMemoryError,///< allocate memory error.
	E_UnkownError,///< unkown error.
	E_Total_Status,
};

enum LocateType {
	E_LocateCenter,   //regress: x+y, probit
	E_LocateCircle,   //regress: x+y, probit, one side length; like a circle or square.
	E_LocateRectangle,//regress: x+y, probit, width and height; like a rectangle.
	E_LocateRotateBox,//regress: x+y, probit, one side length, and arrow; rotatebox.
};

/**indicate a smple, has multilayers and roi. using to train and infer**/
struct SampleData
{
	std::vector<te::Image> imageMatrix;///< multiple image layers of a sample.
	te::Rect roi;///< rectangle of interest on the sample.
};

/**indicate groundtruth or result of AI tool**/
struct AiInstance
{
	int     id;
	float   area;
	float   angle;
	float   probit;
	std::string name;
	te::ConnectedRegionF contour;
};

/**descriptor about image, one layer of sample.**/
struct ImageDesc
{
	te::BaseType dtype;///<only E_UInt8 and E_Float32 are supported.
	int heapId;///<defferent images with same channel and datatype can be put into a heap together.
	int channel;///<gray=1 or color=3?...default is bgr.
};

/**descriptor about result contour.**/
struct ContourDesc
{
	int maxContourCount;///< maximum count of contour with a single sample.(Max object count)
	int maxInnerContourCount;///< maximum inner polygon count of a contour.
	int maxContourPointCount;///< maximum points to descript a contour.
};

/**descriptor about the compute device.**/
struct DeviceInfo
{
	DeviceType deviceType;///< gpu, cpu, or other device.
	int deviceID;///<
};

/**descriptor about memories the network comsumed.**/
struct NetMemorySize
{
	size_t staticMemoryDev;///< how many exclusive memory on device does the network'infer occupy.
	size_t staticMemoryHost;///< how many exclusive memory on host does the network'infer occupy.
	size_t shareMemoryDev;///< how many sharing memory on device does the network'infer occupy.
	size_t shareMemoryHost;///< how many sharing memory on host does the network'infer occupy.
};

using AiInstSet = std::vector<AiInstance>;

using AiResult = std::vector<AiInstance>;

using ContourSet = std::vector<ConnectedRegionF>;

/**descriptor about the groundtruth of a sample.**/
struct SampleMark
{
	AiInstSet  gtDataSet;
	ContourSet localMask;
};

/**descriptor about a sample with mark.**/
struct SampleInfo
{
	SampleData sampleData;
	SampleMark sampleMark;
};

/**descriptor about the training status at some time.**/
struct TrainState
{
	int   iteration;///< current training iteration, 
	bool  bSaveFlag;///< is the model saved at this iteration.
	float fProgress;///< training progress, current iteration / total iteration.
	float fPosAcc;///< positive sample accuracy
	float fNegAcc;///< negative sample accuracy
	float fAvgLoss;///< average loss.
	float fMaxLoss;///< maximum loss.
	float fProbLoss;///< probit loss.
	float fLocLoss;///< location loss.
	float fWeightEva;///< weighted loss.
	int64_t epochTime;///< epoch time
	size_t  nPosCnt;///< positive sample count
	size_t  nNegCnt;///< negative sample count
	int reserved[32];///¡¶ reserved.
};

typedef int(*AiComputeLockFunc)(void* pParam, int iType);

typedef int(*AiExceptionFunc)(void* pParam, AiStatus eStatus);

typedef void(*AiInferCallBack)(AiResult& inferResult, te::DynamicMatrix& hotmap, void* pCallbackParam);

TE_END_NAMESPACE