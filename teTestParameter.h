#pragma once

#include <QWidget>
#include "ui_teTestParameter.h"

#include "TestParam.h"

QT_BEGIN_NAMESPACE
namespace Ui { class teTestParameterClass; };
QT_END_NAMESPACE

class teTestParameter : public QWidget
{
	Q_OBJECT

public:
	teTestParameter(QWidget *parent = nullptr);
	~teTestParameter();

public slots:
	void SaveteTestParameter();

public:
	void getTestParam(te::TestParam* test);

private:
	Ui::teTestParameterClass *ui;
};
