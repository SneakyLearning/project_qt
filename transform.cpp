#include "transform.h"

transformer::transformer()
{
	this->mat=vtkMatrix4x4::New();
	this->landmarkTransform = vtkLandmarkTransform::New();
	this->sourcePoints = vtkSmartPointer<vtkPoints>::New();;
	this->targetPoints = vtkSmartPointer<vtkPoints>::New();;
}

void transformer::computerTranform()
{
	landmarkTransform->SetSourceLandmarks(sourcePoints);
	landmarkTransform->SetTargetLandmarks(targetPoints);
	landmarkTransform->SetModeToRigidBody();
	landmarkTransform->Update();
	vtkMatrix4x4* temp_mat = landmarkTransform->GetMatrix();
	mat->DeepCopy(temp_mat);
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

vector<float> transformer::convert_coordinate_to_robot(float x, float y, float z)
{
	float temp[4] = { x,y,z,1.0f };
	float *result= mat->MultiplyPoint(temp);
	cout << result[0] << " " << result[1] << " " << result[2] << " " << result[3] << endl;
	vector<float> res = { result[0] ,result[1],result[2] };
	return res;
}