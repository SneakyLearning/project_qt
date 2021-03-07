#pragma once

#include "KinectCamera.h"

#include <QtWidgets/QWidget>
#include "ui_project_qt.h"
#include <vtkRenderWindow.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

#include <string>

using namespace pcl;
using namespace std;
using namespace cv;

class project_qt : public QWidget
{
	Q_OBJECT

public:
	project_qt(QWidget *parent = Q_NULLPTR);
	void PoinCloudShow();
	void ImageShow();
	void pushbutton_2_slot();
	void pushbutton_3_slot();

protected:
	
private:
	Ui::project_qtClass ui;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};
