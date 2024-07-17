#include "teTestParaRegister.h"

TE_BEGIN_NAMESPACE

RTTR_REGISTRATION
{
	TE_REGISTER_TYPE(ContourDesc)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(ContourDesc, maxContourCount)
	TE_REGISTER_PROPERTY(ContourDesc, maxInnerContourCount)
	TE_REGISTER_PROPERTY(ContourDesc, maxContourPointCount)
	TE_REGISTER_END

	TE_REGISTER_ENUM_TYPE(BaseType)
	(
		TE_REGISTER_ENUM_VALUE(BaseType::FP16),
		TE_REGISTER_ENUM_VALUE(BaseType::TF32)
	)
	TE_REGISTER_END

	TE_REGISTER_ENUM_TYPE(DeviceType)
	(
		TE_REGISTER_ENUM_VALUE(DeviceType::E_CPU),
		TE_REGISTER_ENUM_VALUE(DeviceType::E_NVIDIA),
		TE_REGISTER_ENUM_VALUE(DeviceType::E_DENGLIN),
		TE_REGISTER_ENUM_VALUE(DeviceType::E_ALL)
	)
	TE_REGISTER_END

	TE_REGISTER_TYPE(DeviceInfo)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(DeviceInfo, deviceType)
	TE_REGISTER_PROPERTY(DeviceInfo, deviceID)
	TE_REGISTER_END

	TE_REGISTER_TYPE(TestParaRegister)
	TE_REGISTER_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(TestParaRegister, maxbatchsize)
	TE_REGISTER_PROPERTY(TestParaRegister, batchsize)
	TE_REGISTER_PROPERTY(TestParaRegister, contourdesc)
	TE_REGISTER_PROPERTY(TestParaRegister, deviceinfo)
	TE_REGISTER_PROPERTY(TestParaRegister, precision)
	TE_REGISTER_END
}

TestParaRegister::TestParaRegister()
{
	maxbatchsize = 1;
	batchsize = 1;
	contourdesc = { 12,5,256 };
	deviceinfo = { E_NVIDIA,0 };
	precision = BaseType::TF32;
}

TE_END_NAMESPACE