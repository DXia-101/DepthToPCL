#include "teTestParameter.h"
#include "teRapidjsonObjectTree.h"
#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

teTestParameter::teTestParameter(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTestParameterClass())
{
	ui->setupUi(this);

	ui->treeView->setWriter(std::make_shared<te::ObjectTreeWidgetWriter>());
	te::TestParam param;
	te::deserializeJsonFromIFStream("./TestParaconfig.ini", &param);

	ui->treeView->writeObject_t(param);

	ui->treeView->setExpandedRecursive(true);
}

teTestParameter::~teTestParameter()
{
	delete ui;
}

void teTestParameter::SaveteTestParameter() 
{
	te::TestParam param;
	ui->treeView->readObject_t(&param);

	te::serializeJsonToOFStream("./TestParaconfig.ini", param);
}

void teTestParameter::getTestParam(te::TestParam* test)
{
	ui->treeView->readObject_t(test);
}