#pragma once

#include <QWidget>
#include "teViewModel.h"

#include <memory>

namespace te {
	class ReceptiveFieldViewMenber;
	class ReceptiveFieldView : public QWidget
	{
		Q_OBJECT

	public:
		ReceptiveFieldView(QWidget* parent = nullptr);
		~ReceptiveFieldView();

	protected:
		void initStateMachine();
		void receptiveFieldChange();

	protected:
		void paintEvent(QPaintEvent* event) override;
		void mouseMoveEvent(QMouseEvent* event) override;
		void mousePressEvent(QMouseEvent* event) override;
		void mouseReleaseEvent(QMouseEvent* event) override;
		void wheelEvent(QWheelEvent* event) override;
		void enterEvent(QEvent* event) override;
		void leaveEvent(QEvent* event) override;
		void transMouseEvents(QMouseEvent* event);
		void transWheelEvents(QWheelEvent* event);
	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected slots:
		void refresh(ViewModel::updateMode);
		void outOfBounds();

	signals:
		void sig_enterThrD();
		void sig_enterTwoD();
	private:
		std::weak_ptr<ViewModel> viewModel;
		ReceptiveFieldViewMenber* menber;
	};

}