#include "teViewModel.h"
#include "teMainView.h"
#include "teThreeDView.h"
#include "teThreeDToolbarView.h"
#include "teThreeDMenuView.h"
#include "teModel.h"
#include "teAi3DStorage.h"
#include "teLabelBrowserView.h"
#include "teTrainParaView.h"
#include "teTestParaView.h"
#include "teThreeDMarkView.h"
#include "teReceptiveFieldView.h"
#include "teTrainStatisticsView.h"
#include <QtWidgets/QApplication>

#include <memory>

using namespace te;

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	MainView* mainView = new MainView();

	auto dbStore = std::make_unique<Ai3DStorage>();
	Model* AiModel = new Model(std::move(dbStore));
	ThreeDView* threeDView = new ThreeDView();
	ThreeDToolbarView* threeDToolbar = new ThreeDToolbarView();
	ThreeDMenuView* threeDViewMenu = new ThreeDMenuView();
	LabelBrowserView* teLabelBrowser = new LabelBrowserView();
	ThreeDMarkView* threeDMark = new ThreeDMarkView();
	ReceptiveFieldView* teReceptiveField = new ReceptiveFieldView();
	TrainParaView* trainPara = new TrainParaView();
	TestParaView* testPara = new TestParaView();
	TrainStatisticsView* trainStatistics = new TrainStatisticsView();

	std::shared_ptr<ViewModel> viewModel = std::make_shared<ViewModel>();

	mainView->bindViewModel(viewModel);
	threeDView->bindViewModel(viewModel);
	threeDView->bindViewModel(viewModel);
	threeDToolbar->bindViewModel(viewModel);
	threeDViewMenu->bindViewModel(viewModel);
	teLabelBrowser->bindViewModel(viewModel);
	threeDMark->bindViewModel(viewModel);
	teReceptiveField->bindViewModel(viewModel);
	trainPara->bindViewModel(viewModel);
	testPara->bindViewModel(viewModel);
	trainStatistics->bindViewModel(viewModel);
	viewModel->setModel(AiModel);

	mainView->addWidgetToStackLayout(teReceptiveField, ViewModel::ReceptiveFieldView);
	mainView->addWidgetToStackLayout(threeDView, ViewModel::ThreeDView);
	mainView->addWidgetToStackLayout(threeDMark, ViewModel::ThreeDMarkView);
	mainView->addWidgetToLabelLayout(teLabelBrowser);
	mainView->addWidgetToToolBarLayout(threeDToolbar);
	mainView->addWidgetToToolBarLayout(threeDViewMenu);
	mainView->addWidgetToLabelLayout(trainPara);
	mainView->addWidgetToLabelLayout(testPara);

	viewModel->setCurrentWidgetType(ViewModel::ReceptiveFieldView);
	viewModel->notified(ViewModel::StartMark);
	//threeDView->move(teReceptiveField->pos());
	threeDView->show();

	mainView->show();

	return a.exec();
}
