#include "transform.h"

vtkSmartPointer<vtkPoints> sourcePoints = vtkSmartPointer<vtkPoints>::New();
vtkSmartPointer<vtkPoints> targetPoints = vtkSmartPointer<vtkPoints>::New();
vtkSmartPointer<vtkLandmarkTransform> landmarkTransform =
vtkSmartPointer<vtkLandmarkTransform>::New();
vtkMatrix4x4* mat;

void transformer::computerTranform()
{
	landmarkTransform->SetSourceLandmarks(sourcePoints);
	landmarkTransform->SetTargetLandmarks(targetPoints);
	landmarkTransform->SetModeToRigidBody();
	landmarkTransform->Update();
	mat = landmarkTransform->GetMatrix();
	std::cout << "Matrix: " << *mat << std::endl;
}

void transformer::addSourcePoints(float x,float y,float z)
{
	float sourcePoint[3] = {x,y,z};
	sourcePoints->InsertNextPoint(sourcePoint);
}

void transformer::addTargetPoints(float x, float y, float z)
{
	float targetPoint[3] = { x,y,z };
	targetPoints->InsertNextPoint(targetPoint);
}

void transformer::convert_coordinate_to_robot(float x, float y, float z)
{
	float temp[4] = { x,y,z,1.0f };
	float *result= mat->MultiplyPoint(temp);
	cout << result[0] << " " << result[1] << " " << result[2] << " " << result[3] << endl;
}