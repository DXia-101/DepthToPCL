#pragma once

#include <QWidget>
#include "ui_teLabelBrowserView.h"
#include "teViewModel.h"
#include <memory>

class QTableWidgetItem;
class QTableWidget;

QT_BEGIN_NAMESPACE
namespace Ui { class LabelBrowserViewClass; };
QT_END_NAMESPACE

namespace te {

	class LabelBrowserView : public QWidget
	{
		Q_OBJECT

	public:
		LabelBrowserView(QWidget* parent = nullptr);
		~LabelBrowserView();

	protected:
		void initInterface();
		void sendCurrentItemInfo(QTableWidgetItem* item);
		void saveTableWidget();
		void loadTableWidget();
		void addRowToTable(const QString& content, const QColor& fontColor);
		bool checkFirstColumn(const QString& searchString);

		void selectCurrentLabel();
		void selectLabelColor();

		QColor getSelectedRowFontColor();
		QColor getFontColorByFirstColumnValue(const QString&);
		QString getSelectedRowCategory();

		void updateTrainCount();
		void updateResultCount();


	protected slots:
		void on_addLabelButton_clicked();
		void on_deleteLabelButton_clicked();
		void addLabel(QString category);

	public:
		void bindViewModel(std::shared_ptr<ViewModel>);

	protected slots:
		void refresh(ViewModel::updateMode);

	private:
		std::weak_ptr<ViewModel> viewModel;

		QTableWidget* LabelWidget;

	private:
		Ui::LabelBrowserViewClass* ui;
	};

}
