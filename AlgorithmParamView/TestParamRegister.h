#pragma once

#include "teBaseDevelopmentKit.h"
#include "teTraining.h"
#include "teAiExTypes.h"

#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

#include <QObject>

TE_BEGIN_NAMESPACE

class TestParamRegister
{
public:
	TestParamRegister();

public:
	int maxbatchsize;
	int batchsize;
	ContourDesc contourdesc;
	DeviceInfo deviceinfo;
	ComputePrecision precision;
};

TE_END_NAMESPACE