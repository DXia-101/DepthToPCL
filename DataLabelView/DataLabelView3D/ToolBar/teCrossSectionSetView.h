#pragma once

#include <QDialog>
#include "ui_teCrossSectionSetView.h"

QT_BEGIN_NAMESPACE
namespace Ui { class CrossSectionSetViewClass; };
QT_END_NAMESPACE
namespace te {
	class CrossSectionSetView : public QDialog
	{
		Q_OBJECT

	private slots:
		void on_cancelButton_clicked();
		void on_okButton_clicked();
		void on_XradioButton_clicked();
		void on_YradioButton_clicked();
		void on_ZradioButton_clicked();
		void on_YesRadioButton_clicked();
		void on_NoRadioButton_clicked();

	public:
		CrossSectionSetView(QWidget* parent = nullptr);
		~CrossSectionSetView();

	private:
		Ui::CrossSectionSetViewClass* ui;

		int startRenge, endRenge;
		QString axis, isSave;
		QString configFilePath;
	};
}