#pragma once

#include <QWidget>
#include <QCloseEvent>
#include <QMouseEvent>
#include "ui_teTrainStatisticsView.h"
#include "teViewModel.h"
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui { class TrainStatisticsViewClass; };
QT_END_NAMESPACE

class QStatusBar;
class QCustomPlot;
class QCPGraph;

namespace te {
	class TrainStatisticsView : public QWidget
	{
		Q_OBJECT

	public:
		TrainStatisticsView(QWidget* parent = nullptr);
		~TrainStatisticsView();
	protected:
		void ReceiveData();

	protected slots:
		void on_iteration_checkBox_stateChanged(int arg);
		void on_fAvgLoss_checkBox_stateChanged(int arg);
		void on_fPosAcc_checkBox_stateChanged(int arg);

	protected:
		void isShow(int arg);
		void restoreInitialState();

	protected:
		void initQPlot(QCustomPlot* customPlot);

		void showPlot(QCustomPlot* customPlot, int iterationNum, float fAvgLossNum, float fPosAccNum);
		void closeEvent(QCloseEvent* event) override;
		void handlePositionToolTip(QMouseEvent* event);
		bool eventFilter(QObject* watched, QEvent* event);

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;

	private:
		QCustomPlot* customPlot;
		QStatusBar* customBar;
		QCPGraph* iteration;
		QCPGraph* fAvgLoss;
		QCPGraph* fPosAcc;

	private:
		Ui::TrainStatisticsViewClass* ui;
	};

}

