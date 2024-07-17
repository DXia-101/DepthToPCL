#pragma once

#include <QDialog>
#include "ui_tePointCloudPointSizeSetView.h"

QT_BEGIN_NAMESPACE
namespace Ui { class PointCloudPointSizeSetViewClass; };
QT_END_NAMESPACE
namespace te {
	class PointCloudPointSizeSetView : public QDialog
	{
		Q_OBJECT

	public:
		PointCloudPointSizeSetView(QWidget* parent = nullptr);
		~PointCloudPointSizeSetView();

	public:
		int getSize();

	private:
		void on_buttonBox_accepted();
		void on_PointSize_SpinBox_valueChanged(int value);
		void on_PointSizeHSlider_valueChanged(int value);

	private:
		Ui::PointCloudPointSizeSetViewClass* ui;

		int pointCloudSize;
	};
}