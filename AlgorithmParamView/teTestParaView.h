#pragma once

#include <QWidget>
#include "ui_teTestParaView.h"
#include <memory>
#include "teViewModel.h"

QT_BEGIN_NAMESPACE
namespace Ui { class TestParaViewClass; };
QT_END_NAMESPACE

namespace te {
	class TestParaView : public QWidget
	{
		Q_OBJECT

	public:
		TestParaView(QWidget* parent = nullptr);
		~TestParaView();

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected:
		void saveTestParameter();

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;

	private:
		Ui::TestParaViewClass* ui;
	};

}

