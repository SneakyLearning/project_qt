#include "mylabel.h"
#include <QMouseEvent>

MyLabel::MyLabel(QWidget *parent)
	: QLabel(parent)
{
	this->setMouseTracking(true);
}

MyLabel::~MyLabel()
{
}

void MyLabel::mouseMoveEvent(QMouseEvent* ev)
{
	QString text = QString("<center><h1>Mouse Press:(%1,%2)</center>").arg(ev->x()).arg(ev->y());
	this->setText(text);
}
