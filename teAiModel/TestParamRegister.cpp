#include "TestParamRegister.h"

TE_BEGIN_NAMESPACE

RTTR_REGISTRATION
{
	TE_REGISTER_TYPE(ContourDesc)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(ContourDesc, maxContourCount)
	TE_REGISTER_PROPERTY(ContourDesc, maxInnerContourCount)
	TE_REGISTER_PROPERTY(ContourDesc, maxContourPointCount)
	TE_REGISTER_END

	TE_REGISTER_ENUM_TYPE(ComputePrecision)
	(
		TE_REGISTER_ENUM_VALUE(E_FP16),
		TE_REGISTER_ENUM_VALUE(E_FP32)
	)
	TE_REGISTER_END

	TE_REGISTER_ENUM_TYPE(DeviceType)
	(
		TE_REGISTER_ENUM_VALUE(E_CPU),
		TE_REGISTER_ENUM_VALUE(E_GPU)
	)
	TE_REGISTER_END

	TE_REGISTER_TYPE(DeviceInfo)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(DeviceInfo, deviceType)
	TE_REGISTER_PROPERTY(DeviceInfo, deviceID)
	TE_REGISTER_END

	TE_REGISTER_TYPE(TestParamRegister)
	TE_REGISTER_TYPE_CONSTRUCTOR()
	TE_REGISTER_PROPERTY(TestParamRegister, maxbatchsize)
	TE_REGISTER_PROPERTY(TestParamRegister, batchsize)
	TE_REGISTER_PROPERTY(TestParamRegister, contourdesc)
	TE_REGISTER_PROPERTY(TestParamRegister, deviceinfo)
	TE_REGISTER_PROPERTY(TestParamRegister, precision)
	TE_REGISTER_END
}

TestParamRegister::TestParamRegister()
{
	maxbatchsize = 1;
	batchsize = 1;
	contourdesc = { 12,5,256 };
	deviceinfo = { E_GPU,0 };
	precision = E_FP32;
}

TE_END_NAMESPACE