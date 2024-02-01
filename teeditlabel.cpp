#include"teeditlabel.h"

#include<QDebug>

TeEditLabel::TeEditLabel(QWidget *parent):QWidget(parent)
{
    m_pStackedWidget=new QStackedWidget(this);

    setLayout(new QVBoxLayout(this));
    layout()->setMargin(0);
    layout()->setSpacing(0);
    layout()->addWidget(m_pStackedWidget);

    m_pShowLabel=new QLabel(this);
    m_pLineEdit=new QLineEdit(this);
    m_pLineEdit->setReadOnly(false);

    m_pStackedWidget->insertWidget(E_EDIT_LABEL,m_pShowLabel);
    m_pStackedWidget->insertWidget(E_EDIT_TEXT,m_pLineEdit);


    m_pShowLabel->installEventFilter(this);
    m_pLineEdit->installEventFilter(this);
    setSizePolicy(m_pLineEdit->sizePolicy());


}

TeEditLabel::~TeEditLabel()
{
//    delete m_pShowLabel;
//    delete m_pLineEdit;
//    delete m_pStackedWidget;

}

void TeEditLabel::teSwitchLabel()
{
    m_pStackedWidget->setCurrentIndex(E_EDIT_LABEL);

    if(!m_bReadOnly)
    {
        if(m_pLineEdit->text()!=m_pShowLabel->text())
        {
            m_pShowLabel->setText(m_pLineEdit->text());
            emit teTextChanged(m_pLineEdit->text());

        }

    }

}


void TeEditLabel::teSwitchText()
{
    m_pStackedWidget->setCurrentIndex(E_EDIT_TEXT);

    m_pLineEdit->setText(m_pShowLabel->text());//Text肯定来自Label

    m_pLineEdit->setFocus();
    m_pLineEdit->selectAll();

}


void TeEditLabel::teSetText(const QString &qstrText)
{
    m_pShowLabel->setText(qstrText);
    m_pLineEdit->setText(qstrText);
}


void TeEditLabel::teClearText()
{
    m_pShowLabel->clear();
    m_pLineEdit->clear();
}

/*
编辑文本框时：
1.回车键切回Label
2.文本框失去焦点切回Label

显示Label时：
1.双击编辑文本框

其他处理策略放到父控件

*/

bool TeEditLabel::eventFilter(QObject *watched, QEvent *event)
{

    if (watched == m_pLineEdit)
    {

        if(event->type() == QEvent::KeyPress)
        {
            QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

            if(keyEvent->key() == Qt::Key_Return)
            {
                teSwitchLabel();
            }

        }
        else if (event->type() == QEvent::FocusOut)
        {
            if(Qt::PopupFocusReason!=static_cast<QFocusEvent*>(event)->reason())
            {
                teSwitchLabel();
            }

        }
    }
    else if (watched == m_pShowLabel)
    {

        if(event->type()==QEvent::MouseButtonDblClick)
        {
            teSwitchText();

        }
    }

    return QWidget::eventFilter(watched, event);


}
