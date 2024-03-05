#pragma once

#include <QObject>
#include "tesampwidget.h"


class QVBoxLayout;

class teImageBrowserController  : public QObject
{
	Q_OBJECT

public:
	teImageBrowserController(QObject *parent = nullptr);
	~teImageBrowserController();

	void displayUIInWidget(QVBoxLayout* layout);

private slots:
	void UpdateItem(int* pIndex, int len);
	void ItemActive(int* pIndex, int len);
	void SwitchImg(int pIndex, int len);

public slots:
	void ChangeGTShowFlag(int index);
	void ChangeRSTShowFlag(int index);
	void ChangeCurrentState();

private:
	TeSampWidget* ImageBrowser;
	bool GTShowFlag;
	bool RSTShowFlag;
	bool CurrentState;
};
