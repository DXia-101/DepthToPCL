#include "teApp.h"

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

using namespace te;

App::App(QObject* parent)
	: QObject(parent)
{
	mainView = std::make_unique < MainView>();

	viewModel = std::make_shared<ViewModel>();

	auto dbStore = std::make_unique<Ai3DStorage>();
	AiModel = std::make_unique < Model>(std::move(dbStore));
	threeDView = std::make_unique < ThreeDView>();
	threeDToolbar = std::make_unique < ThreeDToolbarView>();
	threeDViewMenu = std::make_unique < ThreeDMenuView>();
	teLabelBrowser = std::make_unique < LabelBrowserView>();
	threeDMark = std::make_unique < ThreeDMarkView>();
	teReceptiveField = std::make_unique < ReceptiveFieldView>();
	trainPara = std::make_unique < TrainParaView>();
	testPara = std::make_unique < TestParaView>();
	trainStatistics = std::make_unique < TrainStatisticsView>();

	viewModel->setModel(AiModel.get());
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

	mainView->addWidgetToStackLayout(teReceptiveField.get(), ViewModel::ReceptiveFieldView);
	mainView->addWidgetToStackLayout(threeDView.get(), ViewModel::ThreeDView);
	mainView->addWidgetToStackLayout(threeDMark.get(), ViewModel::ThreeDMarkView);
	mainView->addWidgetToLabelLayout(teLabelBrowser.get());
	mainView->addWidgetToToolBarLayout(threeDToolbar.get());
	mainView->addWidgetToToolBarLayout(threeDViewMenu.get());
	mainView->addWidgetToLabelLayout(trainPara.get());
	mainView->addWidgetToLabelLayout(testPara.get());

	viewModel->setCurrentWidgetType(ViewModel::ReceptiveFieldView);
	viewModel->notified(ViewModel::StartMark);
	//threeDView->move(teReceptiveField->pos());
	threeDView->show();

	mainView->show();
}

App::~App()
{
	//delete threeDView;
	//delete threeDToolbar;
	//delete threeDViewMenu;
	//delete teLabelBrowser;
	//delete threeDMark;
	//delete teReceptiveField;
	//delete trainPara;
	//delete testPara;
	//delete trainStatistics;
	//delete mainView;
	//delete AiModel;
}
