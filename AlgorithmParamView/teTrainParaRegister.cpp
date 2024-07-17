#include "teTrainParaRegister.h"


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
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(ImageDesc, dtype)
	TE_REGISTER_PROPERTY(ImageDesc, heapId)
	TE_REGISTER_PROPERTY(ImageDesc, channel)
	TE_REGISTER_END

	TE_REGISTER_TYPE(TrainParaRegister)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(TrainParaRegister, TrainBatchSize)
	TE_REGISTER_PROPERTY(TrainParaRegister, PatchWidth)
	TE_REGISTER_PROPERTY(TrainParaRegister, PatchHeight)
	TE_REGISTER_PROPERTY(TrainParaRegister, receptiveField)
	TE_REGISTER_PROPERTY(TrainParaRegister, trainIterCnt)
	TE_REGISTER_PROPERTY(TrainParaRegister, saveFrequency)
	TE_REGISTER_PROPERTY(TrainParaRegister, eLocateType)
	TE_REGISTER_PROPERTY(TrainParaRegister, DeviceID)
	TE_REGISTER_PROPERTY(TrainParaRegister, AutomaticStop)
	TE_REGISTER_PROPERTY(TrainParaRegister, locateSide)
	TE_REGISTER_PROPERTY(TrainParaRegister, sampleDesc)
	TE_REGISTER_PROPERTY(TrainParaRegister, netName)
	TE_REGISTER_END
}

TrainParaRegister::TrainParaRegister()
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
	sampleDesc[0].dtype = te::BaseType::Float32;
	sampleDesc[0].channel = 1;
	sampleDesc[0].heapId = 0;
	DeviceID = 0;
	AutomaticStop = false;
}

TE_END_NAMESPACE