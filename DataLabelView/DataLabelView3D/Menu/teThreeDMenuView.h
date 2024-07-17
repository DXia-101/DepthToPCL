#pragma once

#include <QWidget>
#include "ui_teThreeDMenuView.h"
#include "teViewModel.h"
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui { class ThreeDMenuViewClass; };
QT_END_NAMESPACE
namespace te {

	class ThreeDMenuView : public QWidget
	{
		Q_OBJECT

	public:
		ThreeDMenuView(QWidget* parent = nullptr);
		~ThreeDMenuView();
	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected:
		void setHeightCoefficientSpinBoxValue();

	private slots:
		void on_reductionBtn_clicked();
		void on_ViewYBtn_clicked();
		void on_ViewXBtn_clicked();
		void on_ViewZBtn_clicked();
		void on_startMarkBtn_clicked();
		void on_showGTCheckBox_stateChanged(int arg);
		void on_showRSTcheckBox_stateChanged(int arg);
		void on_HighTransformBtn_clicked();

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;

	private:
		Ui::ThreeDMenuViewClass* ui;
	};

}