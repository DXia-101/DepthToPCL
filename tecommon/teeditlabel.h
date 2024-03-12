#ifndef TEEDITLABEL_H
#define TEEDITLABEL_H


#include <QFormLayout>
#include <QKeyEvent>
#include <QLabel>
#include <QLineEdit>
#include <QStackedWidget>
#include <QVBoxLayout>

#include"tewidget_global.h"

class TeEditLabel: public QWidget
{
    Q_OBJECT

public:
    enum TeEditType
    {
        E_EDIT_LABEL=0,
        E_EDIT_TEXT=E_EDIT_LABEL+1,
    };

    TeEditLabel(QWidget *parent=nullptr);

    ~TeEditLabel()override;

    QString teGetText() const{return m_pShowLabel->text();}


    void teSetText(const QString &qstrText);
    void teClearText();

    TeEditType teGetEditType()
    {
        return static_cast<TeEditType>(m_pStackedWidget->currentIndex());
    }

    void teSwitchLabel();
    void teSwitchText();

    void teSetReadOnly(bool b)
    {
        m_bReadOnly=b;
    }

    void teSetKeyIgnore(bool b)
    {
        m_bIgnoreKey=b;
    }


signals:
    void teTextChanged(QString qstr);


protected:
     bool eventFilter(QObject *watched, QEvent *event)override;

    //调父类事件会默认为ignore，继续传递给父控件，
    void keyPressEvent(QKeyEvent * qevent)override
    {
        if(m_bIgnoreKey)
            QWidget::keyPressEvent(qevent);
        else
            qevent->accept();
    }


private:
    QLabel *m_pShowLabel;
    QLineEdit *m_pLineEdit;
    QStackedWidget* m_pStackedWidget;

    bool m_bReadOnly=true;
    bool m_bIgnoreKey=true;


};







#endif // TEEDITLABEL_H
