#pragma once

#include <QObject>
#include "tesampwidget.h"
#include "teDataBrowserWorkThread.h"
#include "teImage.h"
#include <QThread>

class QVBoxLayout;
class QThread;
class teAiModel;

class teDataBrowserController  : public QObject
{
	Q_OBJECT

public:
	teDataBrowserController(QObject* parent = nullptr);
	~teDataBrowserController();

	void displayUIInWidget(QVBoxLayout* layout);
	void setteAiModel(teAiModel* aiModel);

public slots:
#ifndef _CC_
	
	void UpdateItem(int* pIndex, int len);
	
#endif // !_CC_
	void ChangeCurrentState();
	void SwitchImg(int pIndex, int len);
	void teUpDataSet(int iNum, int iLayerNum, bool bReset);

signals:
	void sig_ChangeCurrentState();
	void sig_teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void sig_NeedReload();
	void sig_LoadPointCloud(QString);
	void sig_LoadOriginImage(QString);
	void sig_updateTrainWidget();
	void sig_updateResultWidget();
	void sig_IndexChanged();

private:
	TeSampWidget* ImageBrowser;
	teDataBrowserWorkThread* worker;
	QThread* thread;
	teAiModel* m_teAiModel;

	bool CurrentState;
};
