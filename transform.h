#pragma once

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkLandmarkTransform.h>
#include <vtkMatrix4x4.h>

class transformer
{
public:

	void computerTranform();
	void addSourcePoints(float x, float y, float z);
	void addTargetPoints(float x, float y, float z);
	void convert_coordinate_to_robot(float x, float y, float z);
};