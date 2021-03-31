#include "KinectCamera.h"
#include "project_qt.h"
#include "pointCloudProcess.h"
#include "transform.h"
#include <vtkAutoInit.h>		//³õÊ¼»¯VTKÄ£¿é

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

kinectCamera kinect;
pointCloudProcess process;
transformer trans;

project_qt::project_qt(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	connect(ui.pushButton_init, &QPushButton::pressed, this, &project_qt::pushbutton_init_slot);
	connect(ui.pushButton_getdata, &QPushButton::pressed, this, &project_qt::pushbutton_getdata_slot);
	connect(ui.pushButton_calibrate, &QPushButton::pressed, this, &project_qt::pushbutton_calibrate_slot);
	connect(ui.pushButton_pass, &QPushButton::pressed, this, &project_qt::pushbutton_pass_slot);
	connect(ui.pushButton_voxel, &QPushButton::pressed, this, &project_qt::pushbutton_voxel_slot);
	connect(ui.pushButton_outlier, &QPushButton::pressed, this, &project_qt::pushbutton_outlier_slot);
	connect(ui.pushButton_background, &QPushButton::pressed, this, &project_qt::pushbutton_background_slot);
	connect(ui.pushButton_center, &QPushButton::pressed, this, &project_qt::pushbutton_center_slot);
	connect(ui.pushButton_line, &QPushButton::pressed, this, &project_qt::pushbutton_line_slot);
	connect(ui.pushButton_getpath, &QPushButton::pressed, this, &project_qt::pushbutton_getpath_slot);
}

void project_qt::PoinCloudShow()
{
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	//viewer->addPointCloud(cloud);
	viewer->initCameraParameters();
	viewer->addCoordinateSystem(1.0);

	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
	viewer->setupInteractor(ui.qvtkWidget->GetInteractor(), ui.qvtkWidget->GetRenderWindow());
	//setCentralWidget(ui.qvtkWidget);

	ui.qvtkWidget->update();
}

void project_qt::ImageShow()
{
	cv::Mat Rgb;
	QImage Img;
	cvtColor(image_rgb, Rgb, CV_BGR2RGB);
	Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGB888);
	int width = ui.label->width();
	int height = ui.label->height();
	Img.scaled(width, height, Qt::KeepAspectRatio);
	ui.label->setPixmap(QPixmap::fromImage(Img));
}

void project_qt::update_cloud()
{
	cout << "update" << endl;
}

void project_qt::pushbutton_init_slot()
{
	kinect.initKinect();
}

void project_qt::pushbutton_getdata_slot()
{
	kinect.getData();
	this->ImageShow();
	viewer->addPointCloud(cloud,"cloud1");
	viewer->updatePointCloud(cloud,"cloud1");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_calibrate_slot()
{

}

void project_qt::pushbutton_pass_slot()
{
	process.passfilter();
	viewer->removePointCloud("cloud1");
	viewer->addPointCloud(cloud, "cloud2");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_voxel_slot()
{
	process.voxelfilter();
	viewer->removePointCloud("cloud2");
	viewer->addPointCloud(cloud, "cloud3");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_outlier_slot()
{
	process.removeOutlier(800, 0.15);
	viewer->removePointCloud("cloud3");
	viewer->addPointCloud(cloud, "cloud4");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_background_slot()
{
	process.drawWeldCloud(1000, 0.010);
	viewer->removePointCloud("cloud4");
	viewer->addPointCloud(cloud, "cloud5");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_center_slot()
{
	//process.removeOutlier(500, 0.1);
	process.drawWeldCloud(1000, 0.002);
	viewer->removePointCloud("cloud5");
	viewer->addPointCloud(cloud,"cloud6");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_line_slot()
{
	vector<PointXYZ> points = process.drawWeldLine(cloud, 0.002);
	viewer->addLine(points[0],points[1],0, 1, 0, "line", 0);
	ui.qvtkWidget->update();
}


void project_qt::pushbutton_getpath_slot()
{

}
