#include "DepthToPCL.h"
#include <QtWidgets/QApplication>
//#include <QSurfaceFormat>
//#include "QVTKOpenGLNativeWidget.h"

#include <opencv.hpp>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    DepthToPCL w;
    w.show();
    return a.exec();
}
