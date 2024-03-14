#pragma once

#include <QObject>
#include "tesampwidget.h"
#include "pcl_function.h"

class teImageBrowserWorkThread  : public QObject
{
	Q_OBJECT

public:
	teImageBrowserWorkThread(QObject*parent = nullptr);
	~teImageBrowserWorkThread();

public:
	void setImageBrowser(TeSampWidget* browser);

public slots:
	void UpdateItem(int* pIndex, int len);
	void ItemActive(int* pIndex, int len);

public slots:
	void teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void InvalidPointThresholdChange(int threshold);
	void ValidPointThresholdChange(int threshold);

signals:
	void sig_SavePointCloud(QString filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr pcr);
	void sig_showAll2DItem();

private:
	TeSampWidget* ImageBrowser;
	int InvalidPointThreshold;
	int ValidPointThreshold;
	bool GTShowFlag;
	bool RSTShowFlag;

};
