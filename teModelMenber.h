#pragma once

#include <vector>
#include <string>
#include "teTraining.h"
#include "tePrediction.h"

namespace te
{
	class ModelMenber
	{
	public:
		std::vector<double> InvalidPointThresholds;
		std::vector<double> ValidPointThresholds;

		std::string modelPath;

		TrainConfig config;
		int DeviceID;
		AiStatus status;
		ModelInfer infer_;
		Training train_;
	};
}