#include "project_qt.h"
#include <vtkAutoInit.h>		//初始化VTK模块

VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);


project_qt::project_qt(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
}

void project_qt::PoinCloudShow()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

	string filepath = "D:\\c++_projects\\project_v4\\project_v4\\weld_cloud.pcd";
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
