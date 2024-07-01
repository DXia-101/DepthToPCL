#pragma once

#include <QObject>
#include "tesampwidget.h"
#include "pcl_function.h"
class teAiModel;
class teDataBrowserWorkThread  : public QObject
{
	Q_OBJECT

public:
	teDataBrowserWorkThread(QObject*parent = nullptr);
	~teDataBrowserWorkThread();

	void setteAiModel(teAiModel* aiModel);
public:
	void setImageBrowser(TeSampWidget* browser);
    bool SavePointCloud(QString fileName, pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveCloud);

public slots:
	void ItemActive(int* pIndex, int len);

private:
	TeSampWidget* ImageBrowser;
	bool GTShowFlag;
	bool RSTShowFlag;

	teAiModel* m_teAiModel;
};
