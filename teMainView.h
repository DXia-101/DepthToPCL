#pragma once

#include <QWidget>
#include "ui_teMainView.h"
#include <memory>
#include <QMap>
#include "teViewModel.h"
class QStackedLayout;

QT_BEGIN_NAMESPACE
namespace Ui { class MainViewClass; };
QT_END_NAMESPACE

namespace te {
	class MainView : public QWidget
	{
		Q_OBJECT

	public:
		MainView(QWidget* parent = nullptr);
		~MainView();

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	public:
		void addWidgetToStackLayout(QWidget*, ViewModel::TypeWidget);
		void addWidgetToLabelLayout(QWidget*);
		void addWidgetToBrowserLayout(QWidget*);
		void addWidgetToToolBarLayout(QWidget*);

	protected:
		void initToolbar();
		void setCurrentWidgetForStackLayout();

	protected slots:
		void setThresholdSpinBox();
		void loadTrainingImages();
		void prepareTrain();
		void stopTrain();
		void prepareTest();
		void saveParameter();

	private:
		QStackedLayout* stackLayout;
		QMap<ViewModel::TypeWidget, QWidget*> widgetMap;

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;

	private:
		Ui::MainViewClass* ui;
	};
}