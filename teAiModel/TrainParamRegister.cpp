#include "TrainParamRegister.h"
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

	TE_REGISTER_TYPE(TrainParamRegister)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(TrainParamRegister, TrainBatchSize)
	TE_REGISTER_PROPERTY(TrainParamRegister, PatchWidth)
	TE_REGISTER_PROPERTY(TrainParamRegister, PatchHeight)
	TE_REGISTER_PROPERTY(TrainParamRegister, receptiveField)
	TE_REGISTER_PROPERTY(TrainParamRegister, trainIterCnt)
	TE_REGISTER_PROPERTY(TrainParamRegister, saveFrequency)
	TE_REGISTER_PROPERTY(TrainParamRegister, eLocateType)
	TE_REGISTER_PROPERTY(TrainParamRegister, DeviceID)
	TE_REGISTER_PROPERTY(TrainParamRegister, AutomaticStop)
	TE_REGISTER_PROPERTY(TrainParamRegister, locateSide)
	TE_REGISTER_PROPERTY(TrainParamRegister, sampleDesc)
	TE_REGISTER_PROPERTY(TrainParamRegister, netName)
	TE_REGISTER_END
}

TrainParamRegister::TrainParamRegister()
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
	sampleDesc[0].dtype = te::BaseType::E_Float32;
	sampleDesc[0].channel = 1;
	sampleDesc[0].heapId = 0;
	DeviceID = 0;
	AutomaticStop = false;
}

TE_END_NAMESPACE