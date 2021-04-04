#include "subCalibWindow.h"

subCalibWindow::subCalibWindow(QWidget *parent)
	: QDialog(parent)
{
	ui.setupUi(this);
	connect(ui.pushButton_manul, &QPushButton::clicked, this, &subCalibWindow::on_pushbutton_manul);
	connect(ui.pushButton_auto, &QPushButton::clicked, this, &subCalibWindow::on_pushbutton_auto);
}

subCalibWindow::~subCalibWindow()
{
}

void subCalibWindow::on_pushbutton_manul()
{
	emit ManulEvent();
}

void subCalibWindow::on_pushbutton_auto()
{
	emit AutoEvent();
}



