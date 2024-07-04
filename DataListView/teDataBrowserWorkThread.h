#pragma once

#include <QObject>
#include "tesampwidget.h"
#include "pcl_function.h"

#define _CC_

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
#ifdef _CC_
	void UpdateItem(int* pIndex, int len);
#else
	void SwitchImg(int pIndex, int len);

signals:
	void sig_NeedReload();
	void sig_LoadPointCloud(QString);
	void sig_LoadOriginImage(QString);
	void sig_updateTrainWidget();
	void sig_updateResultWidget();
	void sig_IndexChanged();

public slots:
	void ChangeCurrentState();
private:
	bool CurrentState;
#endif

private:
	TeSampWidget* ImageBrowser;
	bool GTShowFlag;
	bool RSTShowFlag;

	teAiModel* m_teAiModel;
};
