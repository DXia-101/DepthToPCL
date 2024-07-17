#pragma once

#include "teBaseDevelopmentKit.h"
#include "teTraining.h"
#include "teAiExTypes.h"

#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

TE_BEGIN_NAMESPACE

class TestParaRegister
{
public:
	TestParaRegister();

public:
	int maxbatchsize;
	int batchsize;
	ContourDesc contourdesc;
	DeviceInfo deviceinfo;
	BaseType precision;
};

TE_END_NAMESPACE