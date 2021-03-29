#include "project_qt.h"
#include <QtWidgets/QApplication>
#include <vtkOutputWindow.h>


int main(int argc, char *argv[])
{
	vtkOutputWindow::SetGlobalWarningDisplay(0);
	QApplication a(argc, argv);
	project_qt w;
	w.show();
	w.PoinCloudShow();
	return a.exec();
}
