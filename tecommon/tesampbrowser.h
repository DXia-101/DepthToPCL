#ifndef TESAMPBROWSER_H
#define TESAMPBROWSER_H

#include"tetablewidget.h"
#include"teeditlabel.h"

#include"tewidget_global.h"

class TeSampBrowser:public TeTableWidget
{
    Q_OBJECT

public:
    TeSampBrowser(QWidget* parent=nullptr,
                  TeBrowseMode eBackMode=E_BROWSE_PAGE,
                  TeBrowseMode eForwardMode=E_BROWSE_PAGE);

    ~TeSampBrowser()override;

    void teSetThumb(int iRow,const QImage& qImg,TeImgFormat eFormat);

    void teSetThumbInfo(int iRow,int iWidth,int iHeight,const QString& qstr);

	bool teIsThumbNull(int iRow);

    int teFieldNum()
    {

        return columnCount()-2;
    }

    void teAddField(QString qstrName);

    void teSetFieldValue(int iRow,int iFieldIndex,QString qstrValue,
                         QColor qClolor);

    QString teGetFieldValue(int iRow,int iFieldIndex);

    void teHideField(int iFieldIndex)
    {
        hideColumn(2+iFieldIndex);
    }

    void teShowField(int iFieldIndex)
    {
        showColumn(2+iFieldIndex);
    }


protected:
    void keyPressEvent(QKeyEvent * qevent)override;

    void mousePressEvent(QMouseEvent *event)override;


private:
    QWidget* teCreateWidget(QTableWidgetItem* pItem);


};


#endif // TESAMPBROWSER_H
