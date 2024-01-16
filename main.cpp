#include "DepthToPCL.h"
#include <QtWidgets/QApplication>
//#include <QSurfaceFormat>
//#include "QVTKOpenGLNativeWidget.h"

int main(int argc, char *argv[])
{
    //QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
    QApplication a(argc, argv);
    DepthToPCL w;
    w.show();
    return a.exec();
}
