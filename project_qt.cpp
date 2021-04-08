#include "KinectCamera.h"
#include "project_qt.h"
#include "pointCloudProcess.h"
#include "transform.h"
#include <vtkAutoInit.h>		//初始化VTK模块
#include <fstream>
#include "subCalibWindow.h"

#pragma execution_character_set("utf-8")

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

kinectCamera kinect;
pointCloudProcess process;
transformer trans;

project_qt::project_qt(QWidget *parent)
	: QMainWindow(parent), choose_xyz(0), viewname_Index(0)
{
	ui.setupUi(this);
	ui.label->setText(QString("图像显示区域"));
	ui.label->setAlignment(Qt::AlignCenter);
	ui.label_hint->setText("输入框");
	ui.input->setEnabled(false);
	connect(ui.pushButton_init, &QPushButton::pressed, this, &project_qt::pushbutton_init_slot);
	connect(ui.pushButton_getdata, &QPushButton::pressed, this, &project_qt::pushbutton_getdata_slot);
	connect(ui.pushButton_calibrate, &QPushButton::pressed, this, &project_qt::pushbutton_showcalibrate_slot);
	connect(ui.pushButton_pass, &QPushButton::pressed, this, &project_qt::pushbutton_pass_slot);
	connect(ui.pushButton_voxel, &QPushButton::pressed, this, &project_qt::pushbutton_voxel_slot);
	connect(ui.pushButton_outlier, &QPushButton::pressed, this, &project_qt::pushbutton_outlier_slot);
	connect(ui.pushButton_background, &QPushButton::pressed, this, &project_qt::pushbutton_background_slot);
	connect(ui.pushButton_center, &QPushButton::pressed, this, &project_qt::pushbutton_center_slot);
	connect(ui.pushButton_line, &QPushButton::pressed, this, &project_qt::pushbutton_line_slot);
	connect(ui.pushButton_getpath, &QPushButton::pressed, this, &project_qt::pushbutton_getpath_slot);
	connect(ui.input, &QLineEdit::returnPressed, this, &project_qt::lineEdit_receiveData);
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
	if (kinect.initKinect() == 0)
	{
		ui.statusBar->showMessage("相机初始化失败", 3000);
	}
	else{ ui.statusBar->showMessage("相机初始化成功", 3000); }
}

void project_qt::pushbutton_getdata_slot()
{
	kinect.getData();
	this->ImageShow();
	string indexS = to_string(viewname_Index);
	viewer->addPointCloud(cloud, indexS);
	viewer->updatePointCloud(cloud, indexS);
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_calibrate_slot()
{
	if (nine_points_xyz.size()!=0)
	{
		ui.label_hint->setText(QString("输入点1的x坐标"));
		ui.input->setEnabled(true);
		ui.statusBar->showMessage(QString("请标定%1个点").arg(nine_points_xyz.size()),3000);
	}
	else  ui.statusBar->showMessage("未找到标定点", 3000); 
	for (vector<vector<float>>::const_iterator iter = nine_points_xyz.begin(); iter != nine_points_xyz.end(); iter++)
	{
		trans.addSourcePoints((*iter)[0], (*iter)[1], (*iter)[2]);
	}
}

void project_qt::pushbutton_showcalibrate_slot()
{
	subCalibWindow* subcalibwin = new subCalibWindow(this);
	subcalibwin->show();
	connect(subcalibwin, &subCalibWindow::ManulEvent, this, &project_qt::pushbutton_calibrate_slot);
	connect(subcalibwin, &subCalibWindow::AutoEvent, this, &project_qt::pushbutton_loadcalibrate_slot);
}

void project_qt::pushbutton_loadcalibrate_slot()
{
	ifstream ifs;
	ifs.open("calibrate.txt", ios::in);
	if (!ifs.is_open())
	{
		ui.statusBar->showMessage("读取失败！", 3000);
		return;
	}
	double tempvalue;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			ifs >> tempvalue;
			trans.mat->SetElement(i,j,tempvalue);
		}
	}
	ui.statusBar->showMessage("标定结果已读取", 3000);
	std::cout << "Matrix: " << *trans.mat << std::endl;
	ifs.close();
}

void project_qt::pushbutton_pass_slot()
{
	process.passfilter();
	string indexS = to_string(viewname_Index);
	viewer->removePointCloud(indexS);
	indexS = to_string(viewname_Index++);
	viewer->addPointCloud(cloud, indexS);
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
	viewer->removePointCloud("cloud3");
	viewer->addPointCloud(cloud, "cloud4");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_center_slot()
{
	process.drawWeldCloud(1000, 0.002);
	process.removeOutlier(10, 0.01);
	viewer->removePointCloud("cloud4");
	viewer->addPointCloud(cloud,"cloud5");
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_line_slot()
{
	vector<PointXYZ> points = process.drawWeldLine(cloud, 0.002,0);
	viewer->addLine(points[0],points[1],0, 1, 0, "line", 0);
	ui.qvtkWidget->update();
	for (size_t i = 0; i < points.size(); i++)
	{
		trans.convert_coordinate_to_robot(points[i].x, points[i].y, points[i].z);
	}
}

void project_qt::pushbutton_getpath_slot()
{
	
}

void project_qt::lineEdit_receiveData()
{
	QString s = ui.input->text();
	PtData[choose_xyz % 3] = s.toFloat();
	choose_xyz++;
	if (choose_xyz % 3 == 0)
	{
		trans.addTargetPoints(PtData[0], PtData[1], PtData[2]);
	}
	if (choose_xyz == 3*nine_points_xyz.size())
	{
		trans.computerTranform();
		ofstream ofs;
		ofs.open("calibrate.txt", ios::trunc);
		for (int i = 0; i < 4; i++)
		{
			for (int j = 0; j < 4; j++)
			{
				ofs << trans.mat->Element[i][j] << endl;
			}
		}
		ofs.close();
		ui.statusBar->showMessage("标定结果已保存", 3000);
		ui.label_hint->setText("输入框");
		ui.input->setEnabled(false);
		return;
	}
	switch (choose_xyz % 3) 
	{
	case 0:ui.label_hint->setText(QString("输入点%1的x坐标").arg(floor(choose_xyz / 3 )));
	case 1:ui.label_hint->setText(QString("输入点%1的y坐标").arg(floor(choose_xyz / 3 )));
	case 2:ui.label_hint->setText(QString("输入点%1的z坐标").arg(floor(choose_xyz / 3 )));
	}
}
