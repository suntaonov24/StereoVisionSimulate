#pragma once
#include <iostream>
#include "CameraParams.h"
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkRenderer.h>
#include <vtkGlyph3D.h>
#include <vtkActor.h>
#include <vtkNamedColors.h>
#include <vtkProperty.h>
#include <vtkPolyDataMapper.h>
#include <vtkSphereSource.h>
#include <vtkAxesActor.h>
#include <vtkRenderWindow.h>
#include <vtkSmartPointer.h>
struct ReconActor
{
	vtkNew<vtkSphereSource> sphere;
	vtkNew<vtkPoints> points;
	vtkNew<vtkPolyData> polydata;
	vtkNew<vtkGlyph3D> glyph3D;
	vtkNew<vtkPolyDataMapper> mapper;
	vtkNew<vtkActor> actor;
	vtkNew<vtkNamedColors> colors;
	vtkSmartPointer<vtkRenderer> render;
	vtkSmartPointer<vtkRenderWindow> renWin;
};
#define ARRAY_TO_VTK4X4MATRIX(a,b) for (unsigned int i = 0; i < 4; ++i)\
{																				\
	for (unsigned int j = 0; j < 4; ++j)\
	{																			\
		a->SetElement(i, j, b[4 * i + j]);\
	}																			\
}
class StereoVisionImpl;
class __EXPORT_API__ StereoVision
{
public:
	StereoVision();
	~StereoVision();
	void IsDebug(bool debug);
	void LoadActor(const char* imagePath);
	void SetLeftCamera(CameraManager* camera);
	void SetRightCamera(CameraManager* camera);
	//Registrate call back function for calculating disparity map
	void RegisterCallback(void(*func)(unsigned char* imageLeft, CameraManager* left,unsigned char* imageRight, CameraManager* right,ReconActor* actor,bool debug));
	void Update();
	CameraManager* mLeft = nullptr;
	CameraManager* mRight = nullptr;
private:
	StereoVisionImpl* mPimpl = nullptr;
	void (*mFunc)(unsigned char* imageLeft, CameraManager* left, unsigned char* imageRight, CameraManager* right,ReconActor* actor, bool debug) = nullptr;
};