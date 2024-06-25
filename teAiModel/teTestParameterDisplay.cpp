#include "teTestParameterDisplay.h"
#include "teRapidjsonObjectTree.h"
#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

teTestParameterDisplay::teTestParameterDisplay(QWidget *parent)
	: QWidget(parent)
	, ui(new Ui::teTestParameterDisplayClass())
{
	ui->setupUi(this);

	ui->treeView->setWriter(std::make_shared<te::ObjectTreeWidgetWriter>());
	te::TestParamRegister param;
	te::deserializeJsonFromIFStream("./TestParaconfig.ini", &param);

	ui->treeView->writeObject_t(param);

	ui->treeView->setExpandedRecursive(true);
}

teTestParameterDisplay::~teTestParameterDisplay()
{
	delete ui;
}

void teTestParameterDisplay::SaveteTestParameter()
{
	te::TestParamRegister param;
	ui->treeView->readObject_t(&param);

	te::serializeJsonToOFStream("./TestParaconfig.ini", param);
	ui->treeView->checkItemChange();
}

void teTestParameterDisplay::getTestParam(te::TestParamRegister* test)
{
	ui->treeView->readObject_t(test);
}