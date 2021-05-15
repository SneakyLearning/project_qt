#pragma once

#include <QtWidgets/QMainWindow>
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

class project_qt : public QMainWindow
{
	Q_OBJECT

public:
	project_qt(QWidget *parent = Q_NULLPTR);
	void savePointCloud();
	void PoinCloudShow();
	void ImageShow();
	void pushbutton_init_slot();
	void pushbutton_getdata_slot();
	void pushbutton_calibrate_slot();
	void pushbutton_showcalibrate_slot();
	void pushbutton_loadcalibrate_slot();
	void pushbutton_pass_slot();
	void pushbutton_voxel_slot();
	void pushbutton_outlier_slot();
	void pushbutton_background_slot();
	void pushbutton_center_slot();
	void pushbutton_line_slot();
	void pushbutton_getpath_slot();
	void lineEdit_receiveData();
	void save_image();
	void load_pointcloud();
	int choose_xyz;
	float PtData[3];

	int viewname_Index;
	int line_Index;
protected:
	
private:
	Ui::project_qtClass ui;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
};
