#pragma once
#pragma warning(disable:4996)

#include <iostream>
#include <Windows.h>
#include <NuiApi.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;
using namespace pcl;

extern PointCloud<PointXYZ>::Ptr cloud;
extern vector<vector<int>> nine_points;
extern vector<vector<float>> nine_points_xyz;

class kinectCamera
{
public:

	bool initKinect();
	void getData();
	void findPointsArea();
	void find_nine_circles(Mat src_img, int bias_x, int bias_y, int bias_width, int bias_height);
	void find_point_xyz(vector<vector<int>> nine_points);

protected:
	HANDLE depthStream;
	HANDLE rgbStream;
	INuiSensor* sensor;
};