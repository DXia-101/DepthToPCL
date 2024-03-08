#include "TrainParam.h"
#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

TE_BEGIN_NAMESPACE

RTTR_REGISTRATION
{
	TE_REGISTER_ENUM_TYPE(LocateType)
	(
		TE_REGISTER_ENUM_VALUE(E_LocateCenter),
		TE_REGISTER_ENUM_VALUE(E_LocateCircle),
		TE_REGISTER_ENUM_VALUE(E_LocateRectangle),
		TE_REGISTER_ENUM_VALUE(E_LocateRotateBox)
	)
	TE_REGISTER_END

	TE_REGISTER_TYPE(ImageDesc)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(ImageDesc, dtype)
	TE_REGISTER_PROPERTY(ImageDesc, heapId)
	TE_REGISTER_PROPERTY(ImageDesc, channel)
	TE_REGISTER_END

	TE_REGISTER_TYPE(TrainParam)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(TrainParam, TrainBatchSize)
	TE_REGISTER_PROPERTY(TrainParam, PatchWidth)
	TE_REGISTER_PROPERTY(TrainParam, PatchHeight)
	TE_REGISTER_PROPERTY(TrainParam, receptiveField)
	TE_REGISTER_PROPERTY(TrainParam, trainIterCnt)
	TE_REGISTER_PROPERTY(TrainParam, saveFrequency)
	TE_REGISTER_PROPERTY(TrainParam, eLocateType)
	TE_REGISTER_PROPERTY(TrainParam, DeviceID)
	TE_REGISTER_PROPERTY(TrainParam, AutomaticStop)
	TE_REGISTER_PROPERTY(TrainParam, locateSide)
	TE_REGISTER_PROPERTY(TrainParam, sampleDesc)
	TE_REGISTER_PROPERTY(TrainParam, netName)
	TE_REGISTER_END
}

TrainParam::TrainParam()
{
	TrainBatchSize = 6;
	PatchWidth = 0;
	PatchHeight = 0;
	receptiveField = 64;
	trainIterCnt = 128;
	saveFrequency = 2;
	//eToolType = ToolType::E_PixelDetect_Tool;
	//eTrainMode = TrainMode::E_TE_RESET;
	eLocateType = LocateType::E_LocateRectangle;
	locateSide = 0;
	netName = "defalut";
	sampleDesc.resize(1);
	sampleDesc[0].dtype = te::BaseType::E_UInt8;
	sampleDesc[0].channel = 3;
	sampleDesc[0].heapId = 0;
	DeviceID = 0;
	AutomaticStop = false;
}

TE_END_NAMESPACE