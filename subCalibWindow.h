#pragma once

#include <QDialog>
#include "ui_subCalibWindow.h"

class subCalibWindow : public QDialog
{
	Q_OBJECT

public:
	subCalibWindow(QWidget *parent = Q_NULLPTR);
	~subCalibWindow();
	void on_pushbutton_manul();
	void on_pushbutton_auto();

signals:
	void ManulEvent();
	void AutoEvent();

private:
	Ui::subCalibWindow ui;
};
