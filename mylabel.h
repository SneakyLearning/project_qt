#pragma once

#include <QLabel>

class MyLabel : public QLabel
{
	Q_OBJECT

public:
	MyLabel(QWidget *parent);
	~MyLabel();

protected:
	void mouseMoveEvent(QMouseEvent *ev);
};
