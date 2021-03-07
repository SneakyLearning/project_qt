#include "project_qt.h"
#include <vtkAutoInit.h>		//初始化VTK模块

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

kinectCamera kinect;

project_qt::project_qt(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	connect(ui.pushButton_2, &QPushButton::pressed, this, &project_qt::pushbutton_2_slot);
	connect(ui.pushButton_3, &QPushButton::pressed, this, &project_qt::pushbutton_3_slot);
}

void project_qt::PoinCloudShow()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	string filepath = "stl2pcd.pcd";
	if (-1 == pcl::io::loadPCDFile(filepath, *cloud)) //打开点云文件
	{
		cout << "error input!" << endl;
		return;  
	}
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
	viewer->addPointCloud(cloud);
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
	Mat demo_img = imread("D:\\c++_projects\\project_v5\\template.jpg");
	QImage Img;
	cvtColor(demo_img, Rgb, CV_BGR2RGB);
	Img = QImage((const uchar*)(Rgb.data), Rgb.cols, Rgb.rows, Rgb.cols * Rgb.channels(), QImage::Format_RGB888);
	ui.label->setPixmap(QPixmap::fromImage(Img));
}

void project_qt::pushbutton_2_slot()
{
	kinect.initKinect();
}

void project_qt::pushbutton_3_slot()
{
	kinect.getData();
}
