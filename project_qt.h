#pragma once

#include <QtWidgets/QWidget>
#include "ui_project_qt.h"
#include <vtkRenderWindow.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string>

using namespace pcl;
using namespace std;

class project_qt : public QWidget
{
	Q_OBJECT

public:
	project_qt(QWidget *parent = Q_NULLPTR);
	void PoinCloudShow();

protected:
	
private:
	Ui::project_qtClass ui;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};
