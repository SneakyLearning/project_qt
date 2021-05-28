#pragma once

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>
#include <vector>
using namespace std;

class transformer
{
public:
	transformer();
	vtkMatrix4x4* mat;
	vtkSmartPointer<vtkLandmarkTransform> landmarkTransform;
	vtkSmartPointer<vtkPoints> sourcePoints;
	vtkSmartPointer<vtkPoints> targetPoints;
	void computerTranform();
	void addSourcePoints(float x, float y, float z);
	void addTargetPoints(float x, float y, float z);
	vector<float> convert_coordinate_to_robot(float x, float y, float z);
};