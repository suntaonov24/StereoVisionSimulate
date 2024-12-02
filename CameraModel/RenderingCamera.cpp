#include "RenderingCamera.h"
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkProperty.h>
#include <vtkSphereSource.h>
#include <vtkAxesActor.h>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkSTLReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkWindowToImageFilter.h>
#include <vtkImageLuminance.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

struct ModelObject
{
	vtkNew<vtkSTLReader> reader;
	vtkNew<vtkSphereSource> sphere;
	vtkNew<vtkPolyDataMapper> mapper;
	vtkNew<vtkActor> actor;
};

class StereoVisionImpl 
{
public:
	StereoVisionImpl() {}
	~StereoVisionImpl() {}
	ModelObject mObject;
};

StereoVision::StereoVision()
{
	mPimpl = new StereoVisionImpl;
}
StereoVision::~StereoVision()
{
	if (mPimpl != nullptr)
	{
		delete mPimpl;
	}
}
void StereoVision::LoadActor(const char* path)
{
	if (std::string(path) != "")
	{
		mPimpl->mObject.reader->SetFileName(path);
		mPimpl->mObject.reader->Update();
		mPimpl->mObject.mapper->SetInputData(mPimpl->mObject.reader->GetOutput());
		mPimpl->mObject.mapper->Update();
	}
	else
	{
		mPimpl->mObject.sphere->SetRadius(10);
		mPimpl->mObject.sphere->SetCenter(50, 10, 400);
		mPimpl->mObject.mapper->SetInputConnection(mPimpl->mObject.sphere->GetOutputPort());
	}
	//mPimpl->mObject.mapper->ScalarVisibilityOff();
	mPimpl->mObject.actor->SetMapper(mPimpl->mObject.mapper);
}
void StereoVision::SetLeftCamera(CameraManager* camera)
{
	mLeft = camera;
}
void StereoVision::SetRightCamera(CameraManager* camera)
{
	mRight = camera;
}
void StereoVision::RegisterLeftImage(unsigned char* image)
{
	mLeftImage = image;
}
void StereoVision::RegisterRightImage(unsigned char* image)
{
	mRightImage = image;
}
void StereoVision::Update()
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkCamera> cameraLeft;
	vtkNew<vtkCameraActor> cameraActorLeft;
	//Set Left camera parameters
	float* externalMatrix_l = mLeft->GetExternalMatrix();
	cameraLeft->SetPosition(0, 0, 0);
	cameraLeft->SetViewUp(0, 1, 0);
	cameraLeft->SetFocalPoint(0, 0, mLeft->mParams->FocalLength);
	float* internalMatrix_l = mLeft->GetInternalMatrix();
	double viewAngle_l = 2.0*atan(0.5*mLeft->mParams->ImageSize[0]/ internalMatrix_l[0])*180.0/PI;
	cameraLeft->SetViewAngle(viewAngle_l);
	cameraLeft->SetClippingRange(mLeft->mParams->ClippingRange[0], mLeft->mParams->ClippingRange[1]);
	vtkNew<vtkMatrix4x4> cameraLeftMatrix;
	for (unsigned int i = 0; i < 4; ++i)
	{
		for (unsigned int j = 0; j < 4; ++j)
		{
			cameraLeftMatrix->SetElement(i, j, externalMatrix_l[4 * i + j]);
		}
	}
	vtkNew<vtkTransform> cameraLeftTransform;
	cameraLeftTransform->SetMatrix(cameraLeftMatrix);
	cameraLeft->ApplyTransform(cameraLeftTransform);
	cameraActorLeft->SetCamera(cameraLeft);
	vtkNew<vtkRenderWindow> renWinLeft;
	renWinLeft->SetWindowName("Left side camera image");
	renWinLeft->SetSize(mLeft->mParams->ImageSize[0],mLeft->mParams->ImageSize[1]);
	vtkNew<vtkRenderer> renderLeft;
	renWinLeft->AddRenderer(renderLeft);
	renderLeft->SetActiveCamera(cameraLeft);
	renderLeft->AddActor(mPimpl->mObject.actor);
	vtkNew<vtkCamera> cameraRight;
	vtkNew<vtkCameraActor> cameraActorRight;
	//Set Right camera parameters
	float* externalMatrix_r = mRight->GetExternalMatrix();
	cameraRight->SetPosition(0, 0, 0);
	cameraRight->SetViewUp(0, 1, 0);
	cameraRight->SetFocalPoint(0, 0, mRight->mParams->FocalLength);
	cameraRight->SetClippingRange(mRight->mParams->ClippingRange[0], mRight->mParams->ClippingRange[1]);
	float* internalMatrix_r = mRight->GetInternalMatrix();
	double viewAngle_r = 2.0*atan(0.5* mRight->mParams->ImageSize[0]/ internalMatrix_r[0])*180.0/PI;
	cameraRight->SetViewAngle(viewAngle_r);
	vtkNew<vtkMatrix4x4> cameraRightMatrix;
	for (unsigned int i = 0; i < 4; ++i)
	{
		for (unsigned int j = 0; j < 4; ++j)
		{
			cameraRightMatrix->SetElement(i, j, externalMatrix_r[4 * i + j]);
		}
	}
	vtkNew<vtkTransform> cameraRightTransform;
	cameraRightTransform->SetMatrix(cameraRightMatrix);
	cameraRight->ApplyTransform(cameraRightTransform);
	cameraActorRight->SetCamera(cameraRight);
	vtkNew<vtkRenderWindow> renWinRight;
	renWinRight->SetWindowName("Right side camera image");
	renWinRight->SetSize(mRight->mParams->ImageSize[0],mRight->mParams->ImageSize[1]);
	vtkNew<vtkRenderer> renderRight;
	renWinRight->AddRenderer(renderRight);
	renderRight->SetActiveCamera(cameraRight);
	renderRight->AddActor(mPimpl->mObject.actor);
	//Renderer world coordinate
	vtkNew<vtkMatrix4x4> worldMatrix;
	worldMatrix->Identity();
	vtkNew<vtkTransform> worldTransform;
	worldTransform->SetMatrix(worldMatrix);
	vtkNew<vtkAxesActor> worldAxesActor;
	worldAxesActor->SetOrigin(worldTransform->GetPosition());
	worldAxesActor->SetOrientation(worldTransform->GetOrientation());
	worldAxesActor->SetTotalLength(50,50,50);
	//Renderer left and right camera actor
	vtkNew<vtkRenderer> renderer;
	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->AddRenderer(renderer);
	renderWindow->SetWindowName("StereoVision Demo");
	renderWindow->SetSize(1000,500);
	vtkNew<vtkRenderWindowInteractor> iren;
	iren->SetRenderWindow(renderWindow);
	renderer->AddActor(cameraActorLeft);
	renderer->AddActor(cameraActorRight);
	renderer->AddActor(worldAxesActor);
	mPimpl->mObject.actor->SetPosition(-132,-85,-200);
	mPimpl->mObject.actor->GetMatrix()->Print(std::cout);
	renderer->AddActor(mPimpl->mObject.actor);
	renderer->ResetCamera();
	renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());
	renderWindow->Render();
	renWinLeft->Render();
	renWinRight->Render();
	//Get renderer window image
	vtkNew<vtkWindowToImageFilter> windowImageLeft;
	windowImageLeft->SetInput(renWinLeft);
	windowImageLeft->SetScale(1);
	windowImageLeft->SetInputBufferTypeToRGB();
	windowImageLeft->ReadFrontBufferOff();
	windowImageLeft->Update();
	vtkNew<vtkImageLuminance> luminanceLeft;
	luminanceLeft->SetInputData(windowImageLeft->GetOutput());
	luminanceLeft->Update();
	vtkNew<vtkWindowToImageFilter> windowImageRight;
	windowImageRight->SetInput(renWinRight);
	windowImageRight->SetScale(1);
	windowImageRight->SetInputBufferTypeToRGB();
	windowImageRight->ReadFrontBufferOff();
	windowImageRight->Update();
	vtkNew<vtkImageLuminance> luminanceRight;
	luminanceRight->SetInputData(windowImageRight->GetOutput());
	luminanceRight->Update();
	if (mLeftImage != nullptr)
	{
		memcpy(mLeftImage,luminanceLeft->GetOutput()->GetScalarPointer(),mLeft->mParams->ImageSize[0]* mLeft->mParams->ImageSize[1]*sizeof(unsigned char));
	}
	if (mRightImage != nullptr)
		memcpy(mRightImage,luminanceRight->GetOutput()->GetScalarPointer(),mRight->mParams->ImageSize[0]*mRight->mParams->ImageSize[1]*sizeof(unsigned char));
	iren->Start();
}