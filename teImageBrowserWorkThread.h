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
    bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud);

public slots:
	void ItemActive(int* pIndex, int len);

private:
	TeSampWidget* ImageBrowser;
	bool GTShowFlag;
	bool RSTShowFlag;
};
