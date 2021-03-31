#include "pointCloudProcess.h"

PointCloud<PointNormal>::Ptr doncloud_filtered(new PointCloud<PointNormal>);

void pointCloudProcess::passfilter()
{
	PassThrough<PointXYZ> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	pass.setFilterLimits(-0.2, 0.15);
	pass.filter(*cloud);
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-0.28, 0.10);
	pass.filter(*cloud);
	return;
}

void pointCloudProcess::voxelfilter()
{
	VoxelGrid<PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setLeafSize(0.005f, 0.005f, 0.005f);
	sor.filter(*cloud);
	/*pcl::visualization::PCLVisualizer viewer("draw weld cloud");
	viewer.addPointCloud(cloud, "cloud");
	cout << "after process:" << cloud->points.size() << "points" << endl;
	cout << "showing result of drawing weld cloud" << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}*/
	return;

}

void pointCloudProcess::removeOutlier(int meank,double threshold)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
	filter.setInputCloud(cloud);
	//cout << "before process:" << cloud->points.size() << "points" << endl;
	filter.setMeanK(meank);
	filter.setStddevMulThresh(threshold);
	filter.filter(*cloud);
	/*pcl::visualization::PCLVisualizer viewer2("remove outlier");
	viewer2.addPointCloud(cloud,"cloud");
	cout << "after process:" << cloud->points.size() << "points" <<endl;
	cout << "showing result of remove outlier" << endl;
	while (!viewer2.wasStopped())
	{
		viewer2.spinOnce(100);
	}*/
	return;
}

void pointCloudProcess::drawWeldCloud(int maxiterations, double threshold)
{
	ModelCoefficients::Ptr cofficients(new ModelCoefficients());
	PointIndices::Ptr inliers(new PointIndices());
	SACSegmentation<PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_PLANE);
	seg.setMethodType(SAC_RANSAC);
	seg.setMaxIterations(maxiterations);
	seg.setDistanceThreshold(threshold);
	ExtractIndices<PointXYZ> extract;
	seg.setInputCloud(cloud);
	seg.segment(*inliers, *cofficients);
	extract.setInputCloud(cloud);
	//cout << "before process:" << cloud->points.size() << "points" << endl;
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloud);
	/*pcl::visualization::PCLVisualizer viewer("draw weld cloud");
	viewer.addPointCloud(cloud, "cloud");
	cout << "after process:" << cloud->points.size() << "points" << endl;
	cout << "showing result of drawing weld cloud" << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}*/
	return;
}

void pointCloudProcess::donFilter(float smallsize=0.005f, float largesize=1.0f)
{
	NormalEstimationOMP<PointXYZ, PointNormal> ne;
	search::KdTree<PointXYZ>::Ptr tree(new search::KdTree<PointXYZ>);
	tree->setInputCloud(cloud);
	ne.setInputCloud(cloud);
	cout << "before process:" << cloud->points.size() << "points" << endl;
	ne.setSearchMethod(tree);
	ne.setViewPoint(numeric_limits<float>::max(), numeric_limits<float>::max(), numeric_limits<float>::max());
	PointCloud<PointNormal>::Ptr normals_small_scale(new PointCloud<PointNormal>), normals_large_scale(new PointCloud<PointNormal>);
	ne.setRadiusSearch(smallsize);
	cout << "computing normals of small size" << endl;
	ne.compute(*normals_small_scale);
	ne.setRadiusSearch(largesize);
	cout << "computing normals of large size" << endl;
	ne.compute(*normals_large_scale);
	PointCloud<PointNormal>::Ptr doncloud(new pcl::PointCloud<PointNormal>);
	copyPointCloud<PointXYZ, PointNormal>(*cloud, *doncloud);
	DifferenceOfNormalsEstimation<PointXYZ, PointNormal, PointNormal> don;
	don.setInputCloud(cloud);
	don.setNormalScaleLarge(normals_large_scale);
	don.setNormalScaleSmall(normals_small_scale);
	if (!don.initCompute())
	{
		std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
		exit(EXIT_FAILURE);
	}
	cout << "computing don" << endl;
	don.computeFeature(*doncloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView(new pcl::visualization::PCLVisualizer("Showing the difference of curvature of two scale"));
	visualization::PointCloudColorHandlerGenericField<PointNormal> handler_k(doncloud, "curvature");
	MView->setBackgroundColor(1, 1, 1);
	MView->addPointCloud(doncloud, handler_k);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
	MView->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
	MView->spin();
	ConditionOr<PointNormal>::Ptr range_cond(new ConditionOr<PointNormal>());
	range_cond->addComparison(FieldComparison<PointNormal>::ConstPtr(new FieldComparison<PointNormal>("curvature", ComparisonOps::LT, 0.1)));
	ConditionalRemoval<PointNormal> condrem;
	condrem.setCondition(range_cond);
	condrem.setInputCloud(doncloud);
	condrem.filter(*doncloud_filtered);
	doncloud = doncloud_filtered;
	//pcl::io::savePCDFileASCII("???", *doncloud);
	boost::shared_ptr<pcl::visualization::PCLVisualizer> MView2(new pcl::visualization::PCLVisualizer("Showing the results of keeping relative small curvature points"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k2(doncloud, "curvature");
	MView2->setBackgroundColor(1, 1, 1);
	MView2->addPointCloud(doncloud, handler_k2);
	MView2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
	MView2->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5);
	MView2->spin();
}

vector<PointXYZ> pointCloudProcess::drawWeldLine(float threshold=0.005f)
{
	PointCloud<PointXYZ>::Ptr doncloud_filtered_duplic(new PointCloud<PointXYZ>());
	PointCloud<PointXYZ>::Ptr cloud_line(new PointCloud<PointXYZ>());
	copyPointCloud(*doncloud_filtered, *cloud_line);
	copyPointCloud(*doncloud_filtered, *doncloud_filtered_duplic);
	ModelCoefficients::Ptr coefficents(new ModelCoefficients);
	PointIndices::Ptr inliers(new PointIndices);
	SACSegmentation<PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_LINE);
	seg.setMethodType(SAC_RANSAC);
	seg.setDistanceThreshold(threshold);
	seg.setInputCloud(cloud_line);
	seg.segment(*inliers, *coefficents);
	ExtractIndices<PointXYZ> extract;
	extract.setInputCloud(cloud_line);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_line);
	PointXYZ min, max;
	getMinMax3D(*doncloud_filtered_duplic, min, max);
	PointXYZ p1(((min.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[3]) + coefficents->values[0], min.y, ((min.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[5]) + coefficents->values[2]);
	PointXYZ p2(((max.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[3]) + coefficents->values[0], max.y, ((max.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[5]) + coefficents->values[2]);
	pcl::visualization::PCLVisualizer viewer("draw weld line");
	viewer.addPointCloud(doncloud_filtered_duplic);
	viewer.addLine<PointXYZ>(p1, p2, 0, 1, 0, "line", 0);
	//viewer.addLine<PointXYZ>(min, max, 1, 0, 0, "line2", 0);
	cout << "showing result of draw weld line" << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
	}
	vector<PointXYZ> result = { p1,p2 };
	return result;
}

vector<PointXYZ> pointCloudProcess::drawWeldLine(PointCloud<PointXYZ>::Ptr source, float threshold)
{
	PointCloud<PointXYZ>::Ptr cloud_line(new PointCloud<PointXYZ>());
	ModelCoefficients::Ptr coefficents(new ModelCoefficients);
	PointIndices::Ptr inliers(new PointIndices);
	SACSegmentation<PointXYZ> seg;
	seg.setOptimizeCoefficients(true);
	seg.setModelType(SACMODEL_LINE);
	seg.setMethodType(SAC_RANSAC);
	seg.setDistanceThreshold(threshold);
	seg.setInputCloud(source);
	seg.segment(*inliers, *coefficents);
	ExtractIndices<PointXYZ> extract;
	extract.setInputCloud(source);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(*cloud_line);
	PointXYZ min, max;
	getMinMax3D(*cloud, min, max);
	PointXYZ p1(((min.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[3]) + coefficents->values[0], min.y, ((min.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[5]) + coefficents->values[2]);
	PointXYZ p2(((max.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[3]) + coefficents->values[0], max.y, ((max.y - coefficents->values[1]) / coefficents->values[4] * coefficents->values[5]) + coefficents->values[2]);
	//pcl::visualization::PCLVisualizer viewer("draw weld line");
	//viewer.addPointCloud(cloud);
	//viewer.addLine<PointXYZ>(p1, p2, 0, 1, 0, "line", 0);
	////viewer.addLine<PointXYZ>(min, max, 1, 0, 0, "line2", 0);
	//cout << "showing result of draw weld line" << endl;
	//while (!viewer.wasStopped())
	//{
	//	viewer.spinOnce(100);
	//}
	vector<PointXYZ> result = { p1,p2 };
	return result;
}
