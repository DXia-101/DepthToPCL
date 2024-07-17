#pragma once

#include <QDialog>
#include "ui_teCoordinateAxisRenderSetView.h"

QT_BEGIN_NAMESPACE
namespace Ui { class CoordinateAxisRenderSetViewClass; };
QT_END_NAMESPACE

namespace te {

	class CoordinateAxisRenderSetView : public QDialog
	{
		Q_OBJECT

	public:
		CoordinateAxisRenderSetView(QWidget* parent = nullptr);
		~CoordinateAxisRenderSetView();

	private:
		Ui::CoordinateAxisRenderSetViewClass* ui;

	public:
		QString getAxis();

	private slots:
		void on_XRadioButton_clicked();
		void on_YRadioButton_clicked();
		void on_ZRadioButton_clicked();
		void on_cancelButton_clicked();
		void on_okButton_clicked();

	private:
		QString axis;
	};
}