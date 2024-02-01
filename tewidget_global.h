#ifndef TEWIDGET_GLOBAL_H
#define TEWIDGET_GLOBAL_H


#include<QString>
#include<QImage>
#include <QMetaType>
#include<QDebug>
#include <QtCore/qglobal.h>




enum TeImgFormat
{
    E_FORMAT_BGR=0,
    E_FORMAT_RGB=E_FORMAT_BGR+1,

};




inline QString Char2QString(const char* pstr,bool bIsLocal=false)
{
    return (bIsLocal)?QString(pstr):QString::fromLocal8Bit(pstr);
}



#endif // TETRAINWIDGET_GLOBAL_H
