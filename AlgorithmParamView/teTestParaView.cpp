#include "teTestParaView.h"
#include "teRapidjsonObjectTree.h"
#include "teObjectTreeWidget.h"
#include "teObjectTreeWidgetItem.h"

using namespace te;
TestParaView::TestParaView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::TestParaViewClass())
{
	ui->setupUi(this);

	ui->treeView->setWriter(std::make_shared<te::ObjectTreeWidgetWriter>());
	te::TestParaRegister param;
	te::deserializeJsonFromIFStream("./workspace/TestParaconfig.ini", &param);

	ui->treeView->writeObject_t(param);

	ui->treeView->setExpandedRecursive(true);
}

TestParaView::~TestParaView()
{
	delete ui;
}

void TestParaView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::TestPara)
	{
		saveTestParameter();
	}
}

void TestParaView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &TestParaView::refresh);
	}
}

void te::TestParaView::saveTestParameter()
{
	te::TestParaRegister param;
	ui->treeView->readObject_t(&param);

	te::serializeJsonToOFStream("./workspace/TestParaconfig.ini", param);
	ui->treeView->checkItemChange();

	viewModel.lock()->setTestPara(param);
}
