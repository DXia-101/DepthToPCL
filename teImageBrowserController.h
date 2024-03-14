#pragma once

#include <QObject>
#include "tesampwidget.h"
#include "teImageBrowserWorkThread.h"
#include <QThread>

class QVBoxLayout;
class QThread;
class teImageBrowserController  : public QObject
{
	Q_OBJECT

public:
	static teImageBrowserController* getInstance();
	static void destroy();

	void displayUIInWidget(QVBoxLayout* layout);

public slots:
	void ChangeCurrentState();
	void UpdateItem(int* pIndex, int len);
	void SwitchImg(int pIndex, int len);
	void InvalidPointThresholdChange(int threshold);
	void ValidPointThresholdChange(int threshold);

private:
	TeSampWidget* ImageBrowser;
	teImageBrowserWorkThread* worker;
	QThread* thread;

signals:
	void sig_showAll2DItem();
	void sig_HeightTransform();
	void sig_ChangeCurrentState();
	void sig_teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void sig_InvalidPointThresholdChange(int threshold);
	void sig_ValidPointThresholdChange(int threshold);
	void sig_SavePointCloud(QString filepath, pcl::PointCloud<pcl::PointXYZ>::Ptr pcr);

private:
	bool CurrentState;
	int InvalidPointThreshold;
	int ValidPointThreshold;

private:
	static teImageBrowserController* instance;

	teImageBrowserController(QObject* parent = nullptr);
	~teImageBrowserController();
	teImageBrowserController(const teImageBrowserController&);
	teImageBrowserController& operator=(const teImageBrowserController&);

	class Garbo
	{
	public:
		~Garbo()
		{
			teImageBrowserController::destroy();
		}
	};

	static Garbo tmp;
};
