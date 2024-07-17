#pragma once

#include <QWidget>
#include "ui_teThreeDToolbarView.h"
#include "teViewModel.h"
#include <memory>

QT_BEGIN_NAMESPACE
namespace Ui { class ThreeDToolbarViewClass; };
QT_END_NAMESPACE
namespace te {
	class CoordinateAxisRenderSetView;
	class PointCloudPointSizeSetView;
	class CrossSectionSetView;

	class ThreeDToolbarView : public QWidget
	{
		Q_OBJECT

	public:
		ThreeDToolbarView(QWidget* parent = nullptr);
		~ThreeDToolbarView();

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected:
		void initToolbar();

	private slots:
		void setBackgroundColor();
		void setAxisRender();
		void setPointCloudColor();
		void setPointSize();
		void setCrossSection();
		void setAABB();
		void setOBB();

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;

	private:
		Ui::ThreeDToolbarViewClass* ui;

		CoordinateAxisRenderSetView* axisRender;
		PointCloudPointSizeSetView* pointSize;
		CrossSectionSetView* crossSection;
	};

}