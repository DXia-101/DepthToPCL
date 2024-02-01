#include "tesampwidget.h"
#include "ui_tesampwidget.h"

#include<QHeaderView>
#include<QMenu>

#include<QFile>

#include<algorithm>

static QIcon imageToIcon(const QImage& qImg, const QSize& iconSize)
{
	if (qImg.isNull()) {
		return QIcon();
	}

	QImage iconImage = qImg.scaled(iconSize);
	return QPixmap::fromImage(iconImage);
}


TeSampWidget::TeSampWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::TeSampWidget)
{
    ui->setupUi(this);

	qApp->installEventFilter(this);

    ui->m_NumBox->setRange(0,0);
    ui->m_NumLabel->setNum(0);
	//先这么着吧

	ui->m_LayerWidget->setIconSize({ 80, 80 });
    ui->m_LayerWidget->setVisible(false);

	connect(ui->scrollBar, &QScrollBar::valueChanged, this, [this](int value)
	{
		if (m_eChangeReason != E_User) {
			return;
		}
		m_eChangeReason = E_ScrollBar;
		ui->m_Browse->teSwitchItemByIndex(value);
		m_eChangeReason = E_User;
	});

    connect(ui->m_Browse,&TeTableWidget::teUpDateItem,this,[this](int* piIndex,int iLength)
    {
        emit sig_UpDateItem(piIndex,iLength);
    });

    connect(ui->m_Browse,&TeTableWidget::teSwitchItem,this,[this](int iIndex)
    {
		if (m_eChangeReason != E_Box) {
			ui->m_NumBox->setValue(iIndex + 1);
		}
		if (m_eChangeReason != E_ScrollBar) {//abstract
			ChangeReason tmp = m_eChangeReason;
			m_eChangeReason = E_ScrollBar;
			ui->scrollBar->setValue(iIndex);
			m_eChangeReason = tmp;
		}

		int iCurrentLayer = currentLayerIndex();
        emit sig_SwitchImg(iIndex, iCurrentLayer);

		int iRow = ui->m_Browse->teGetRowByIndex(iIndex);
		if (iRow < 0) {
			return;
		}

		QStringList qlstThumbnailPath = ui->m_Browse->item(iRow, 1)->data(Qt::UserRole + 10).toStringList();
		for (int i = 0; i < qlstThumbnailPath.size(); i++) {
			QImage thumb(qlstThumbnailPath[i]);
			ui->m_LayerWidget->item(i)->setIcon(imageToIcon(thumb, QSize(80, 80)));
		}
    });

    connect(ui->m_Browse,&QTableWidget::cellClicked,this,[this](int row, int column)
    {
        if(row==-1||column==-1)return ;

        if(column>1){
            int iIndex=ui->m_Browse->teGetItemIndex(row);
            emit sig_SamplePropChange(iIndex,column-2);
        }
    });

	ui->m_Browse->horizontalHeader()->setContextMenuPolicy(Qt::CustomContextMenu);

	connect(ui->m_Browse->horizontalHeader(), &QHeaderView::customContextMenuRequested, this, [this](const QPoint &pos)
	{
		QMenu menu;		for (int i = 1; i < ui->m_Browse->columnCount(); i++) {			menu.addAction(ui->m_Browse->horizontalHeaderItem(i)->text());			menu.actions().back()->setCheckable(true);			menu.actions().back()->setChecked(!ui->m_Browse->isColumnHidden(i));		}		menu.exec(QCursor::pos());		for (int i = 0; i < menu.actions().size(); i++) {			ui->m_Browse->setColumnHidden(i + 1, !menu.actions()[i]->isChecked());		}
	});

    connect(ui->m_Browse->horizontalHeader(),&QHeaderView::sectionClicked,this,[this](int column)
    {
		if (column > 1)emit sig_BatchSamplePropChange(column - 2);
    });

	connect(ui->m_LayerWidget, &QListWidget::currentRowChanged, this, [this](int iCurRow) 
	{
		if (iCurRow < 0) {
			return;
		}
		emit sig_SwitchImg(teCurSampleIndex(), iCurRow);
	});

	connect(&m_ActiveTimer, &QTimer::timeout, this, &TeSampWidget::activeTimeout);

	m_ActiveTimer.start(200);
}

TeSampWidget::~TeSampWidget()
{
	m_ActiveTimer.stop();

    delete ui;
}


void TeSampWidget::teSetField(FieldNameTable table, int iLength)
{
	m_NameTable = table;

	int iFieldNum = ui->m_Browse->columnCount() - 2;

	if (iFieldNum > 0) {//清除原来字段
		for (int i = 0; i < iFieldNum; i++) {
			ui->m_Browse->removeColumn(2);
		}
	}

	for (int i = 0; i < iLength; i++) {
		ui->m_Browse->teAddField(m_NameTable(i));
	}
}

void TeSampWidget::teShowField()
{
	int iFieldNum = ui->m_Browse->columnCount() - 2;

	for (int i = 0; i < iFieldNum; i++) {
		ui->m_Browse->teShowField(i);
	}
}

void TeSampWidget::teShowField(int iField)
{
	ui->m_Browse->teShowField(iField);
}

void TeSampWidget::teHideField()
{
	int iFieldNum = ui->m_Browse->columnCount() - 2;

	for (int i = 0; i < iFieldNum; i++) {
		ui->m_Browse->teHideField(i);
	}
}

void TeSampWidget::teUpDateSet(int iNum,int iLayerNum,bool bReset)
{
	int iLastIndex = qMax(0, teCurSampleIndex());

	m_eChangeReason = E_Update;
	ui->m_NumBox->setRange(0, iNum);
	ui->m_NumLabel->setNum(iNum);

	ui->m_LayerLabel->setNum(iLayerNum);

	ui->m_LayerWidget->clear();
	for (int i = 0; i < iLayerNum; i++) {
		ui->m_LayerWidget->addItem(new QListWidgetItem);
	}

	ui->m_Browse->teSetArray(iNum);

	ui->scrollBar->setRange(0, std::max(0, iNum - 1));
	m_eChangeReason = E_User;

	if (iNum == 0)return;
	if (bReset) {
	}
	else {
		ui->m_Browse->teSwitchItemByIndex(qMin(iLastIndex, iNum - 1));
	}

}


void TeSampWidget::teUpDateImg(int iIndex, const QStringList& qlstThumbnailPath, QSize& size, const QString& qstrImgName)
{
    int iRow=ui->m_Browse->teGetRowByIndex(iIndex);
	if (iRow < 0) {
		return;
	}

	QImage qImg(qlstThumbnailPath[0]);
	ui->m_Browse->teSetThumb(iRow, qImg, E_FORMAT_RGB);
	ui->m_Browse->teSetThumbInfo(iRow, size.width(), size.height(), qstrImgName);

	ui->m_Browse->item(iRow, 1)->setData(Qt::UserRole + 10, qlstThumbnailPath);
}


void TeSampWidget::teSetSampleField(int iIndex,int iField,QString qstrValue,
                                    QColor qClolor)
{
    int iRow=ui->m_Browse->teGetRowByIndex(iIndex);
    if(iRow<0)return ;

    ui->m_Browse->teSetFieldValue(iRow,iField,qstrValue,qClolor);

}


void TeSampWidget::teUpdateThumb(int iIndex, int iLayer,const QImage& qImg, TeImgFormat eFormat)
{
	int iRow = ui->m_Browse->teGetRowByIndex(iIndex);
	if (iRow < 0)return;

	if (iRow == ui->m_Browse->currentRow()) {
		QImage qThumbnail = (eFormat == E_FORMAT_BGR) ? qImg.rgbSwapped() : qImg;
		QIcon thumbnailIcon = imageToIcon(qThumbnail, {80,80});
		ui->m_LayerWidget->item(iLayer)->setIcon(thumbnailIcon);
	}

	if (iLayer == 0) {
		ui->m_Browse->teSetThumb(iRow, qImg, eFormat);
	}
}

bool TeSampWidget::teIsThumbNull(int iIndex, int iLayer)
{
	int iRow = ui->m_Browse->teGetRowByIndex(iIndex);
	if (iRow < 0)return true;

	QStringList qlstThumbnailPath = ui->m_Browse->item(iRow, 1)->data(Qt::UserRole + 10).toStringList();

	return !QFile::exists(qlstThumbnailPath[iLayer]);
}

TeTableWidget* TeSampWidget::teBrowserTable()
{
	return ui->m_Browse;
}

int TeSampWidget::teCurSampleIndex()
{
    int iIndex=-1;
    if(ui->m_Browse->currentRow()>=0){
        iIndex=ui->m_Browse->teGetItemIndex(ui->m_Browse->currentRow());
    }

    return iIndex;
}


int TeSampWidget::currentLayerIndex()
{
	return std::max(ui->m_LayerWidget->currentRow(), 0);
}





bool TeSampWidget::eventFilter(QObject *watched, QEvent *event)
{

	switch (event->type()) {
	case(QEvent::KeyPress): {
		QKeyEvent* keyEvent = static_cast<QKeyEvent *>(event);
		switch (keyEvent->key()) {
		case (Qt::Key_Up):case(Qt::Key_Down): {
			if (!qApp->activeWindow())break;//无活跃窗口，不处理
			QWidget* pActiveBrowser = qApp->activeWindow()->findChild<QWidget*>("m_Browse");
			if (ui->m_Browse != pActiveBrowser)break;//活跃控件(可能为空)不是本控件，不处理
			if (qApp->focusWidget() != ui->m_Browse && qApp->focusWidget() != ui->m_LayerWidget) {//俩控件一起抢焦点，太恶心了
				if (m_iFocusWidget == 0) {
					ui->m_Browse->setFocus();
				}
				else {
					ui->m_LayerWidget->setFocus();
				}
			}
			break;
		}
		default:break;
		}
		break;
	}
	case(QEvent::UpdateLater): {
		if (watched == this) {
			activateWindow();
			ui->m_Browse->setFocus();
			if (ui->m_Browse->rowCount() > 0 && ui->m_Browse->currentRow() == -1) {				ui->m_Browse->teSetCurrentRow(0);			}
		}
		break;
	}
	case(QEvent::FocusIn): {
		if (watched == ui->m_Browse) {
			m_iFocusWidget = 0;
		}
		else if (watched == ui->m_LayerWidget) {
			m_iFocusWidget = 1;
		}
	}
	default:break;
	}
	
	return QWidget::eventFilter(watched, event);
}

void TeSampWidget::on_toolButton_clicked()
{
    ui->m_LayerWidget->setVisible(!ui->m_LayerWidget->isVisible());
}


void TeSampWidget::on_m_NumBox_editingFinished()
{
	if (ui->m_NumBox->hasFocus()) {
		ui->m_NumBox->clearFocus();
		return;
	}

	m_eChangeReason = E_Box;
	ui->m_Browse->teSwitchItemByIndex(std::max(0, ui->m_NumBox->value() - 1));
	m_eChangeReason = E_User;
}


void TeSampWidget::activeTimeout()
{
	if (!qApp->activeWindow()) {//无活跃窗口
		return;
	}
	QWidget* pActiveBrowser = qApp->activeWindow()->findChild<QWidget*>("m_Browse");
	if (ui->m_Browse != pActiveBrowser){ //活跃控件(可能为空)不是本控件，不处理
		return;	
	}
	int iVisibleNum = 0;
	int* pArray=ui->m_Browse->teGetVisibleIndex(iVisibleNum);

	if (iVisibleNum == 0)return;

	emit sig_ItemActive(pArray,iVisibleNum);

}
