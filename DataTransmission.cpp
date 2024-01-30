#include "DataTransmission.h"

DataTransmission* DataTransmission::instance = new DataTransmission();

DataTransmission* DataTransmission::GetInstance()
{
	return instance;
}

void DataTransmission::Destory()
{
    if (NULL != DataTransmission::instance)
    {
        delete DataTransmission::instance;
        DataTransmission::instance = nullptr;
    }
}

void DataTransmission::setData(int iteration, float fAvgLoss, float fPosAcc)
{
    emit DataChanged(iteration,fAvgLoss,fPosAcc);
}

DataTransmission::DataTransmission(QObject *parent)
	: QObject(parent)
{}

DataTransmission::DataTransmission(const DataTransmission&)
{}

DataTransmission& DataTransmission::operator=(const DataTransmission&)
{
	return *this;
}

DataTransmission::~DataTransmission()
{}
