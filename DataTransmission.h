#pragma once

#include <QObject>

#include "teTraining.h"
#include "tePrediction.h"
#include "teAugmentation.h"
#include "teImage.h"
#include "teRapidjsonObjectTree.h"
#include "teTimer.h"

class DataTransmission  : public QObject
{
	Q_OBJECT
public:
	static DataTransmission* GetInstance();
	static void Destory();

	void setData(int iteration, float fAvgLoss, float fPosAcc);

	void Filtered();
	bool GetIsFilter();

	void InitTrainSamples(int size);

	std::vector<te::SampleInfo> trainSamples;
signals:
	void DataChanged(int iteration, float fAvgLoss, float fPosAcc);

private:
	DataTransmission(QObject *parent = nullptr);
	DataTransmission(const DataTransmission&);
	DataTransmission& operator=(const DataTransmission&);
	~DataTransmission();

	static DataTransmission* instance;
	bool IsFilter = false;
};


