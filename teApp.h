#pragma once

#include <QObject>
#include <memory>

namespace te {
	class MainView;
	class ViewModel;
	class Model;
	class ThreeDView;
	class ThreeDToolbarView;
	class ThreeDMenuView;
	class LabelBrowserView;
	class ThreeDMarkView;
	class ReceptiveFieldView;
	class TrainParaView;
	class TestParaView;
	class TrainStatisticsView;
	class App : public QObject
	{
		Q_OBJECT

	public:
		App(QObject* parent = nullptr);
		~App();

	private:
		//MainView* mainView;
		//std::shared_ptr<ViewModel> viewModel;
		//Model* AiModel;
		//ThreeDView* threeDView;
		//ThreeDToolbarView* threeDToolbar;
		//ThreeDMenuView* threeDViewMenu;
		//LabelBrowserView* teLabelBrowser;
		//ThreeDMarkView* threeDMark;
		//ReceptiveFieldView* teReceptiveField;
		//TrainParaView* trainPara;
		//TestParaView* testPara;
		//TrainStatisticsView* trainStatistics;

		std::shared_ptr<MainView> mainView;
		std::shared_ptr<ViewModel> viewModel;
		std::shared_ptr<Model> AiModel;
		std::shared_ptr<ThreeDView> threeDView;
		std::shared_ptr<ThreeDToolbarView> threeDToolbar;
		std::shared_ptr<ThreeDMenuView> threeDViewMenu;
		std::shared_ptr<LabelBrowserView> teLabelBrowser;
		std::shared_ptr<ThreeDMarkView> threeDMark;
		std::shared_ptr<ReceptiveFieldView> teReceptiveField;
		std::shared_ptr<TrainParaView> trainPara;
		std::shared_ptr<TestParaView> testPara;
		std::shared_ptr<TrainStatisticsView> trainStatistics;

	};
}