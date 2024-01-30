#pragma once

#include <QObject>

class DataTransmission  : public QObject
{
	Q_OBJECT
public:
	static DataTransmission* GetInstance();
	static void Destory();

	void setData(int iteration, float fAvgLoss, float fPosAcc);
signals:
	void DataChanged(int iteration, float fAvgLoss, float fPosAcc);

private:
	DataTransmission(QObject *parent = nullptr);
	DataTransmission(const DataTransmission&);
	DataTransmission& operator=(const DataTransmission&);
	~DataTransmission();

	static DataTransmission* instance;
};
