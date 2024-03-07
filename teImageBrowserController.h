#pragma once

#include <QObject>
#include "tesampwidget.h"


class QVBoxLayout;

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
	void ChangeGTShowFlag(int index);
	void ChangeRSTShowFlag(int index);
	void ChangeCurrentState();
	void teUpDataSet(int iNum, int iLayerNum, bool bReset);

private:

	TeSampWidget* ImageBrowser;
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
