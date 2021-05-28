#include "KinectCamera.h"
#include "project_qt.h"
#include "pointCloudProcess.h"
#include "transform.h"
#include <vtkAutoInit.h>		//初始化VTK模块
#include <fstream>
#include "subCalibWindow.h"
#include <QTime>
#include <QFileDialog>

#pragma execution_character_set("utf-8")

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

kinectCamera kinect;
pointCloudProcess process;
transformer trans;

project_qt::project_qt(QWidget *parent)
	: QMainWindow(parent), choose_xyz(0), viewname_Index(0),line_Index(0),arrow_Index(0)
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
	connect(ui.actionsave, &QAction::triggered, this, &project_qt::savePointCloud);
	connect(ui.actionsave_temp, &QAction::triggered, this, [&]() {copyPointCloud(*cloud, *process.temp_cloud); });
	connect(ui.actionload_temp, &QAction::triggered, this, [&]() {copyPointCloud(*process.temp_cloud, *cloud); });
	connect(ui.actionsave_image, &QAction::triggered, this, &project_qt::save_image);
	connect(ui.actionload_pointcloud, &QAction::triggered, this, &project_qt::load_pointcloud);
}

void project_qt::savePointCloud()
{
	io::savePCDFile("points.pcd", *cloud);
}

void project_qt::PoinCloudShow()
{
	viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
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
	copyPointCloud(*cloud, *process.origin_cloud);
	copyPointCloud(*cloud, *process.temp_cloud);
	this->ImageShow();
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_calibrate_slot()
{
	if (nine_points_xyz.size()!=0)
	{
		ui.label_hint->setText(QString("输入点0的x坐标"));
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
	process.passfilter(ui.x_pass_min_value->value(),ui.x_pass_max_value->value(),ui.y_pass_min_value->value(),ui.y_pass_max_value->value());
	ui.statusBar->showMessage(QString("降采样后有%1个点").arg(cloud->points.size()), 3000);
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_voxel_slot()
{
	process.voxelfilter(ui.voxe_value->value());
	ui.statusBar->showMessage(QString("降采样后有%1个点").arg(cloud->points.size()), 3000);
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_outlier_slot()
{
	ui.statusBar->showMessage("开始过滤离群点", 3000);
	QTime timer;
	timer.start();
	process.removeOutlier(ui.outlier_meank_value->value(), ui.outlier_thres_value->value());
	ui.statusBar->showMessage(QString("耗时%1ms").arg(timer.elapsed()), 3000);
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_background_slot()
{
	process.drawWeldCloud(ui.back_iter_value->value(), ui.back_thres_value->value());
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_center_slot()
{
	process.removeOutlier(ui.mid_outlier_meank_value->value(), ui.mid_outlier_meank_value->value());
	process.drawWeldCloud(ui.mid_back_iter_value->value(), ui.mid_back_thres_value->value());
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}

void project_qt::pushbutton_line_slot()
{
	vector<PointXYZ> points = process.drawWeldLine(cloud, ui.line_thres_value->value());
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(process.origin_cloud, "cloud" + to_string(viewname_Index));
	viewer->removeShape("line" + to_string(line_Index++));
	viewer->removeShape("arrow" + to_string(arrow_Index++));
	viewer->addLine(points[0],points[1],0, 1, 0, "line"+to_string(line_Index), 0);
	PointXYZ middle_of_twopoints((points[0].x + points[1].x) / 2, (points[0].y + points[1].y) / 2, (points[0].z + points[1].z) / 2);
	PointXYZ end_of_arrow(middle_of_twopoints.x + process.normal[0] / 10, middle_of_twopoints.y + process.normal[1] / 10, middle_of_twopoints.z + process.normal[2] / 10);
	viewer->addArrow(middle_of_twopoints, end_of_arrow,0,255,0,"arrow"+ to_string(arrow_Index), 0);
	ui.qvtkWidget->update();
	ofstream ofile;
	ofile.open("twopoints.txt", ios::app);
	cout << "output two points in robot" << endl;
	for (size_t i = 0; i < points.size(); i++)
	{	
		vector<float> temp_point=trans.convert_coordinate_to_robot(points[i].x, points[i].y, points[i].z);
		ofile << temp_point[0] << endl << temp_point[1] << endl << temp_point[2] << endl;
	}
	ofile.close();
	cout << "output normal in robot" << endl;
	vector<float> temp_normal = trans.convert_coordinate_to_robot(process.normal[0], process.normal[1], process.normal[2]);
	ofstream vfile;
	vfile.open("normal.txt", ios::app);
	vfile << temp_normal[0] << endl << temp_normal[1] << endl << temp_normal[2] << endl;
	vfile.close();
}

void project_qt::pushbutton_getpath_slot()
{
	
}

void project_qt::lineEdit_receiveData()
{
	QString s = ui.input->text();
	ui.input->clear();
	PtData[choose_xyz % 3] = s.toFloat();
	choose_xyz++;
	if (choose_xyz % 3 == 0)
	{
		trans.addTargetPoints(PtData[0], PtData[1], PtData[2]);
	}
	if (choose_xyz == 3 * nine_points_xyz.size())
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
		ui.input->clear();
		ui.input->setEnabled(false);
		return;
	}
	switch (choose_xyz % 3)
	{
	case (0):
	{
		ui.label_hint->setText(QString("输入点%1的x坐标").arg(floor(choose_xyz / 3)));
		break;
	};
	case (1):
	{
		ui.label_hint->setText(QString("输入点%1的y坐标").arg(floor(choose_xyz / 3)));
		break;
	}
	case (2):
	{
		ui.label_hint->setText(QString("输入点%1的z坐标").arg(floor(choose_xyz / 3)));
		break;
	}
	}
}

void project_qt::save_image()
{
	imwrite("image_rgb.jpg", image_rgb);
}

void project_qt::load_pointcloud()
{
	QString file_name = QFileDialog::getOpenFileName();
	PCDReader reader;
	reader.read<PointXYZ>(file_name.toStdString(), *cloud);
	copyPointCloud(*cloud, *process.origin_cloud);
	viewer->removePointCloud("cloud" + to_string(viewname_Index++));
	viewer->addPointCloud(cloud, "cloud" + to_string(viewname_Index));
	ui.qvtkWidget->update();
}
