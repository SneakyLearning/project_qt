#pragma warning(disable:4996)
#include "KinectCamera.h"
#include "project_qt.h"

const int width = 640;
const int height = 480;

Mat image_rgb(480, 640, CV_8UC3);
PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
vector<vector<int>> nine_points;
vector<vector<float>> nine_points_xyz;

bool kinectCamera::initKinect()
{
	int numSensors;
	if (NuiGetSensorCount(&numSensors) < 0 || numSensors < 0) return false;
	if (NuiCreateSensorByIndex(0, &this->sensor) < 0) return false;
	this->sensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_DEPTH | NUI_INITIALIZE_FLAG_USES_COLOR);
	this->sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_DEPTH, NUI_IMAGE_RESOLUTION_640x480, 0, 2, NULL, &this->depthStream);
	this->sensor->NuiImageStreamOpen(NUI_IMAGE_TYPE_COLOR, NUI_IMAGE_RESOLUTION_640x480, 0, 2, NULL, &this->rgbStream);
	return this->sensor;
}

void kinectCamera::getData()
{
	NUI_IMAGE_FRAME imageFrame;
	NUI_LOCKED_RECT LockedRect;
	if (this->sensor->NuiImageStreamGetNextFrame(this->rgbStream, 500, &imageFrame) < 0)
	{
		cout << "getting frame failed" << endl;
		return;
	}
	INuiFrameTexture* texture = imageFrame.pFrameTexture;
	texture->LockRect(0, &LockedRect, NULL, 0);
	for (int i = 0; i < image_rgb.rows; i++)
	{
		uchar* prt = image_rgb.ptr(i);
		uchar* pBuffer = (uchar*)(LockedRect.pBits) + i * LockedRect.Pitch;
		for (int j = 0; j < image_rgb.cols; j++)
		{
			prt[3 * j] = pBuffer[4 * j];
			prt[3 * j + 1] = pBuffer[4 * j + 1];
			prt[3 * j + 2] = pBuffer[4 * j + 2];
		}
	}
	flip(image_rgb, image_rgb, 1);
	/*Mat roi = image_rgb(Rect(270, 200, 125, 125));
	namedWindow("roi", CV_WINDOW_NORMAL);
	imshow("roi", roi);
	waitKey(0);
	imwrite("template.jpg", roi);*/
	texture->UnlockRect(0);
	if (sensor->NuiImageStreamGetNextFrame(depthStream, 1, &imageFrame) < 0) return;
	INuiFrameTexture* texture2 = imageFrame.pFrameTexture;
	texture2->LockRect(0, &LockedRect, NULL, 0);
	const USHORT* curr = (const USHORT*)LockedRect.pBits;
	cloud->width = 640;
	cloud->height = 480;
	cloud->points.resize(640 * 480);
	for (int j = 0; j < height; j++)
	{
		for (int i = 0; i < width; i++)
		{
			USHORT depth = NuiDepthPixelToDepth(*curr++);
			Vector4 pos = NuiTransformDepthImageToSkeleton(i, j, depth << 3, NUI_IMAGE_RESOLUTION_640x480);
			cloud->points[j * 640 + i].x = pos.x / pos.w;
			cloud->points[j * 640 + i].y = pos.y / pos.w;
			cloud->points[j * 640 + i].z = pos.z / pos.w;
			/*cloud->points[j * 640 + i].r = image_rgb.at<Vec3b>(j, 639-i)[2];
			cloud->points[j * 640 + i].g = image_rgb.at<Vec3b>(j, 639-i)[1];
			cloud->points[j * 640 + i].b = image_rgb.at<Vec3b>(j, 639-i)[0];*/
		}
	}
	this->findPointsArea();
	texture2->UnlockRect(0);
	NuiShutdown();
	return;
}

void kinectCamera::findPointsArea()
{
	Mat template_img = imread("template.jpg");
	Mat result(image_rgb.cols - template_img.cols + 1, image_rgb.rows - template_img.rows + 1, CV_32FC1);
	matchTemplate(image_rgb, template_img,result,5);
	double minVal, maxVal;
	Point minLoc, maxLoc, matchLoc;
	minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
	matchLoc = maxLoc;
	rectangle(image_rgb, Rect(matchLoc.x, matchLoc.y, template_img.cols, template_img.rows), Scalar(255, 0, 0));
	/*namedWindow("roi", CV_WINDOW_NORMAL);
	imshow("roi", image_rgb);*/
	this->find_nine_circles(image_rgb, matchLoc.x, matchLoc.y, template_img.cols, template_img.rows);
	//waitKey(0);
}

void kinectCamera::find_nine_circles(Mat src_img,int bias_x,int bias_y,int bias_width,int bias_height)
{
	Mat binary_img, dst_img;
	src_img = image_rgb(Rect(bias_x, bias_y, bias_width,bias_height));
	Mat image_rgb = src_img.clone();
	cvtColor(src_img, src_img, COLOR_BGR2GRAY);
	threshold(src_img, binary_img, 200
		, 255, THRESH_BINARY);
	/*namedWindow("binary", 0);
	imshow("binary", binary_img);*/
	vector<vector<Point>> contours;
	vector<Vec4i> hireachy;
	findContours(binary_img, contours, hireachy, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
	drawContours(image_rgb, contours, -1, Scalar(0, 0, 255));
	int text = 0;
	for (int i = 0; i < hireachy.size(); i++)
	{
		if (hireachy[i][3] != 0) continue;
		if (contours[i].size() < 5)continue;
		double area = contourArea(contours[i]);
		if (area < 9)continue;
		double arc_length = arcLength(contours[i], true);
		double radius = arc_length / (2 * M_PI);
		if (!(17 < radius && radius < 25))
		{
			continue;
		}
		RotatedRect rect = fitEllipse(contours[i]);
		float ratio = float(rect.size.width) / float(rect.size.height);
		if (ratio < 1.1 && ratio > 0.9)
		{
			printf("X: %f\n", rect.center.x);
			printf("Y: %f\n", rect.center.y);
			printf("圆的面积: %f\n", area);
			printf("圆的半径: %f\n", radius);

			nine_points.push_back({ (int)(rect.center.x) + bias_x, (int)(rect.center.y) + bias_y });
			putText(image_rgb, to_string(text++), rect.center, FONT_HERSHEY_COMPLEX, 1, Scalar(0, 255, 0));
			ellipse(image_rgb, rect, Scalar(0, 255, 255), 1);
			circle(image_rgb, rect.center, 1, Scalar(0, 255, 0), 2, 8, 0);
		}

	}
	//drawContours(image_rgb, contours, -1, Scalar(0, 255, 0), 1, 8, hireachy, 4);
	namedWindow("result", 0);
	imshow("result", image_rgb);
	waitKey(0);
	this->find_point_xyz(nine_points);
	return;
}

void kinectCamera::find_point_xyz(vector<vector<int>> nine_points)
{
	for (vector<vector<int>>::const_iterator iter = nine_points.begin(); iter != nine_points.end(); iter++)
	{
		cout << "here3" << endl;
		cout << cloud->points[0].x << endl;
		cout << "x:" << cloud->points[(*iter)[1] * 640 + (639 - (*iter)[0])].x << endl;
		cout << "y:" << cloud->points[(*iter)[1] * 640 + (639 - (*iter)[0])].y << endl;
			cout << "z:" << cloud->points[(*iter)[1] * 640 + (639 - (*iter)[0])].z << endl;
		nine_points_xyz.push_back({
			cloud->points[(*iter)[1] * 640 + (639 - (*iter)[0])].x,
			cloud->points[(*iter)[1] * 640 + (639 - (*iter)[0])].y,
			cloud->points[(*iter)[1] * 640 + (639 - (*iter)[0])].z });
		
		
	}
	return;
}
