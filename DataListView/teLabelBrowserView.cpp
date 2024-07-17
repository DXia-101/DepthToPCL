#include "teLabelBrowserView.h"
#include "teViewModel.h"
using namespace te;
LabelBrowserView::LabelBrowserView(QWidget* parent)
	: QWidget(parent)
	, ui(new Ui::LabelBrowserViewClass())
{
	ui->setupUi(this);
}

LabelBrowserView::~LabelBrowserView()
{
	delete ui;
}

void LabelBrowserView::refresh(ViewModel::updateMode mode)
{
	if (mode == ViewModel::LabelBrowser)
	{
		updateTrainCount();
		updateResultCount();
	}
}

void LabelBrowserView::initInterface()
{
}

void LabelBrowserView::sendCurrentItemInfo(QTableWidgetItem* item)
{
}

void LabelBrowserView::saveTableWidget()
{
}

void LabelBrowserView::loadTableWidget()
{
}

void LabelBrowserView::addRowToTable(const QString& content, const QColor& fontColor)
{
}

bool LabelBrowserView::checkFirstColumn(const QString& searchString)
{
	return false;
}

void LabelBrowserView::selectCurrentLabel()
{
}

void LabelBrowserView::selectLabelColor()
{
}

void LabelBrowserView::on_addLabelButton_clicked()
{
}

void LabelBrowserView::on_deleteLabelButton_clicked()
{
}

void LabelBrowserView::addLabel(QString category)
{
}

void te::LabelBrowserView::bindViewModel(std::shared_ptr<ViewModel> vm)
{
	viewModel = vm;
	if (viewModel.lock())
	{
		connect(viewModel.lock().get(), &ViewModel::notified, this, &LabelBrowserView::refresh);
	}
}

QColor LabelBrowserView::getSelectedRowFontColor()
{
	return QColor();
}

QColor LabelBrowserView::getFontColorByFirstColumnValue(const QString&)
{
	return QColor();
}

QString LabelBrowserView::getSelectedRowCategory()
{
	return QString();
}

void LabelBrowserView::updateTrainCount()
{
}

void LabelBrowserView::updateResultCount()
{
}

