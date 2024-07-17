#pragma once

#include <QWidget>
#include "teViewModel.h"
#include <memory>

namespace te {
	class ThreeDMarkView : public QWidget
	{
		Q_OBJECT

	public:
		ThreeDMarkView(QWidget* parent = nullptr);
		~ThreeDMarkView();

	protected:
		void setDrawState();
		void initWidget();
		void enterEvent(QEvent* event) override;
		void leaveEvent(QEvent* event) override;
		void paintEvent(QPaintEvent* e) override;
		void mousePressEvent(QMouseEvent* e) override;
		void mouseMoveEvent(QMouseEvent* e) override;
		void mouseReleaseEvent(QMouseEvent* e) override;

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;
	};
}