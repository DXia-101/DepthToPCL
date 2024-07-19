#include <QtWidgets/QApplication>
#include "teApp.h"
using namespace te;

int main(int argc, char* argv[])
{
	QApplication a(argc, argv);
	App w;

	return a.exec();
}
