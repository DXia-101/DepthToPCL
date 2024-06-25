#pragma once

#include <QWidget>
#include "ui_teTestParameterDisplay.h"

#include "TestParamRegister.h"

QT_BEGIN_NAMESPACE
namespace Ui { class teTestParameterDisplayClass; };
QT_END_NAMESPACE

class teTestParameterDisplay : public QWidget
{
	Q_OBJECT

public:
	teTestParameterDisplay(QWidget *parent = nullptr);
	~teTestParameterDisplay();

public slots:
	void SaveteTestParameter();

public:
	void getTestParam(te::TestParamRegister* test);

private:
	Ui::teTestParameterDisplayClass *ui;
};
