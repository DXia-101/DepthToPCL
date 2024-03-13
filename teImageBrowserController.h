#pragma once

#include <QObject>
#include "tesampwidget.h"


class QVBoxLayout;
class QThread;
class teImageBrowserController  : public QObject
{
	Q_OBJECT

public:
	static teImageBrowserController* getInstance();
	static void destroy();

	void displayUIInWidget(QVBoxLayout* layout);

private slots:
	void UpdateItem(int* pIndex, int len);
	void ItemActive(int* pIndex, int len);
	void SwitchImg(int pIndex, int len);

public slots:
	void ChangeCurrentState();
	void teUpDataSet(int iNum, int iLayerNum, bool bReset);
	void InvalidPointThresholdChange(int threshold);

signals:
	void sig_showAll2DItem();
	void sig_HeightTransform();

private:
	TeSampWidget* ImageBrowser;
	int InvalidPointThreshold;
	bool GTShowFlag;
	bool RSTShowFlag;
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
