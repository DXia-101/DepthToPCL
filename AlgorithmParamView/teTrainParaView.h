#pragma once

#include <QWidget>
#include "ui_teTrainParaView.h"
#include "teViewModel.h"
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui { class TrainParaViewClass; };
QT_END_NAMESPACE

namespace te {
	class TrainParaView : public QWidget
	{
		Q_OBJECT

	public:
		TrainParaView(QWidget* parent = nullptr);
		~TrainParaView();

	protected:
		void saveTrainPara();

		void unCheckTrainCurveBox();

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected slots:
		void refresh(ViewModel::updateMode);
		void on_TrainingCurveCBox_stateChanged(int arg);

	private:
		std::weak_ptr<ViewModel> viewModel;

	private:
		Ui::TrainParaViewClass* ui;
	};

}

