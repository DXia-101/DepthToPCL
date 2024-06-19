#pragma once

#include <QThread>
#include "tesampwidget.h"
#include "pcl_function.h"

class teImageBrowserWorkThread  : public QThread
{
	Q_OBJECT

public:
	teImageBrowserWorkThread(QThread*parent = nullptr);
	~teImageBrowserWorkThread();

public:
	void setImageBrowser(TeSampWidget* browser);
	void setItemActive(int* pIndex, int len);
	void run();

public slots:
	void teUpDataSet(int iNum, int iLayerNum, bool bReset);

private:
	TeSampWidget* ImageBrowser;
	bool GTShowFlag;
	bool RSTShowFlag;
	int* pIndex;
	int len;
};
