#include "teTrainParaView.h"

#include "teRapidjsonObjectTree.h"

using namespace te;
TrainParaView::TrainParaView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::TrainParaViewClass())
{
	ui->setupUi(this);

	ui->treeView->setWriter(std::make_shared<te::TrainParamWriter>());

	connect(ui->treeView->getWriter(), &te::ObjectTreeWidgetWriter::sig_ItemChange, this, [this](te::ObjectTreeWidgetItem* pItem)
		{
			if (pItem->key() == "receptiveField") {

				int value = 0;
				ui->treeView->readObject_t(&value, pItem);
				viewModel.lock()->setReceptiveField(value);
				viewModel.lock()->notified(ViewModel::ReceptiveField);
			}
		});

	te::TrainParaRegister param;
	te::deserializeJsonFromIFStream("./TrainParaconfig.ini", &param);

	ui->treeView->writeObject_t(param);

	ui->treeView->setExpandedRecursive(true);
}

TrainParaView::~TrainParaView()
{
	delete ui;
}

void TrainParaView::saveTrainPara()
{
	te::TrainParaRegister param;
	ui->treeView->readObject_t(&param);

	te::serializeJsonToOFStream("./TrainParaconfig.ini", param);
	ui->treeView->checkItemChange();

	viewModel.lock()->setTrainPara(&param);
}

void TrainParaView::on_TrainingCurveCBox_stateChanged(int arg)
{
	viewModel.lock()->setTrainStatisticsState(arg);
	viewModel.lock()->notified(ViewModel::ShowTrainStateChart);
}

void TrainParaView::unCheckTrainCurveBox()
{
	ui->TrainingCurveCBox->setChecked(false);
}

void TrainParaView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &TrainParaView::refresh);
	}
}

void TrainParaView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::TrainPara)
	{
		saveTrainPara();
	}
	else if (mode == ViewModel::UnCheckTrainBox)
	{
		unCheckTrainCurveBox();
	}
}
