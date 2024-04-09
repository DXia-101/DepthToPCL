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
	void ItemActive(int* pIndex, int len);


private:
	TeSampWidget* ImageBrowser;
	teImageBrowserWorkThread* worker;

signals:
	void sig_showAll2DItem();
	void sig_ChangeCurrentState();
	void sig_teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void sig_GenerateCurrentData();

private:
	bool CurrentState;

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
