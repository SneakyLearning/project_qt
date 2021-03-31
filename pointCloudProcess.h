#pragma once

#include "KinectCamera.h"

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/common/common.h>

#include <pcl/kdtree/kdtree.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/don.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

using namespace std;
using namespace pcl;

class pointCloudProcess
{
public:
	void passfilter();
	void voxelfilter();
	void removeOutlier(int meank , double threshold );
	void drawWeldCloud(int maxiterations, double threshold);
	void donFilter(float smallsize,float largesize);
	vector<PointXYZ> drawWeldLine(float threshold);
	vector<PointXYZ> drawWeldLine(PointCloud<PointXYZ>::Ptr source, float threshold,int direction);
};