#include "RenderingCamera.h"
#include <semaphore>
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
#include <vtkOBJReader.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkWindowToImageFilter.h>
#include <vtkImageLuminance.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkAppendFilter.h>
#include <vtkDataSetMapper.h>
#include <vtkLODActor.h>
#include <vtkLight.h>
#include <vtkPlaneSource.h>
#include <vtkTexture.h>
#include <vtkTextureMapToPlane.h>
#include <vtkImageFlip.h>
#include <vtkPlanes.h>
#include <vtkFrustumSource.h>
#include <vtkShrinkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkLightActor.h>
#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkRenderingOpenGL2);
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);

#define RENDER_CAMERA_FRUSTUM(CameraID) double planesArray##CameraID[24];		\
	camera##CameraID->GetFrustumPlanes(1.0, planesArray##CameraID);				\
	vtkNew<vtkPlanes> planes##CameraID;									\
	planes##CameraID->SetFrustumPlanes(planesArray##CameraID);					\
	vtkNew<vtkFrustumSource> frustum##CameraID;							\
	frustum##CameraID->ShowLinesOff();									\
	frustum##CameraID->SetPlanes(planes##CameraID);								\
	vtkNew<vtkShrinkPolyData> shrink##CameraID;							\
	shrink##CameraID->SetInputConnection(frustum##CameraID->GetOutputPort());	\
	shrink##CameraID->SetShrinkFactor(0.9);								\
	vtkNew<vtkPolyDataMapper> frustumMapper##CameraID;					\
	frustumMapper##CameraID->SetInputConnection(shrink##CameraID->GetOutputPort());\
	vtkNew<vtkProperty> back##CameraID;									\
	back##CameraID->SetColor(colors->GetColor3d("Tomato").GetData());\
	back##CameraID->SetOpacity(0.1);								\
	vtkNew<vtkActor> frustumActor##CameraID;							\
	frustumActor##CameraID->SetMapper(frustumMapper##CameraID);				\
	frustumActor##CameraID->GetProperty()->EdgeVisibilityOff();\
	frustumActor##CameraID->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());\
	frustumActor##CameraID->SetBackfaceProperty(back##CameraID);		\
	frustumActor##CameraID->GetProperty()->SetOpacity(0.1);	

#define RENDER_TEXTURE_PLANE(CameraID) vtkNew<vtkPlaneSource> plane##CameraID;					\
	plane##CameraID->SetOrigin(-m##CameraID->mParams->ImageBoardSize[0] * 0.5, -m##CameraID->mParams->ImageBoardSize[0] * 0.5, m##CameraID->mParams->FocalLength);\
	plane##CameraID->SetPoint1(-m##CameraID->mParams->ImageBoardSize[0] * 0.5, m##CameraID->mParams->ImageBoardSize[1] * 0.5, m##CameraID->mParams->FocalLength);\
	plane##CameraID->SetPoint2(m##CameraID->mParams->ImageBoardSize[0] * 0.5, -m##CameraID->mParams->ImageBoardSize[1] * 0.5, m##CameraID->mParams->FocalLength);\
	vtkNew<vtkImageFlip> flip##CameraID##Image;												\
	flip##CameraID##Image->SetInputData(windowImage##CameraID->GetOutput());		\
	flip##CameraID##Image->SetFilteredAxis(0);									\
	vtkNew<vtkTexture> texture##CameraID;										\
	texture##CameraID->SetInputConnection(flip##CameraID##Image->GetOutputPort());\
	vtkNew<vtkTextureMapToPlane> texturePlane##CameraID;						\
	texturePlane##CameraID->SetInputConnection(plane##CameraID->GetOutputPort());\
	vtkNew<vtkPolyDataMapper> planeMapper##CameraID;								\
	planeMapper##CameraID->SetInputConnection(texturePlane##CameraID->GetOutputPort());\
	vtkNew<vtkActor> texturedPlane##CameraID;												\
	texturedPlane##CameraID->SetMapper(planeMapper##CameraID);								\
	texturedPlane##CameraID->SetTexture(texture##CameraID);								\
	texturedPlane##CameraID->GetProperty()->SetOpacity(0.5);							\
	texturedPlane##CameraID->AddPosition(camera##CameraID##Transform->GetPosition());		\
	texturedPlane##CameraID->AddOrientation(camera##CameraID##Transform->GetOrientation());

struct ModelObject
{
	vtkNew<vtkOBJReader> reader;
	vtkNew<vtkPolyDataMapper> mapper;
	vtkNew<vtkActor> actor;
};
struct CameraImage
{
	vtkNew<vtkConeSource> cameraCone;
	vtkNew<vtkCubeSource> cameraCube;
	vtkNew<vtkAppendFilter> cameraAppend;
	vtkNew<vtkDataSetMapper> cameraMapper;
	vtkNew<vtkLODActor> cameraActor;
};
class StereoVisionImpl 
{
public:
	StereoVisionImpl() {}
	~StereoVisionImpl() {}
	ModelObject mObject;
	ReconActor mReconActor;
	CameraImage mLeft;
	CameraImage mRight;
	bool mDebug = false;
	void RenderCamera(vtkTransform* transform,float scale, CameraImage* camera)
	{
		camera->cameraCone->SetHeight(1.5);
		camera->cameraCone->SetResolution(12);
		camera->cameraCone->SetRadius(0.4);
		camera->cameraCube->SetXLength(1.5);
		camera->cameraCube->SetZLength(0.8);
		camera->cameraCube->SetCenter(0.4,0,0);
		camera->cameraAppend->AddInputConnection(camera->cameraCube->GetOutputPort());
		camera->cameraAppend->AddInputConnection(camera->cameraCone->GetOutputPort());
		camera->cameraMapper->SetInputConnection(camera->cameraAppend->GetOutputPort());
		camera->cameraActor->SetMapper(camera->cameraMapper);
		camera->cameraActor->SetScale(scale, scale, scale);
		camera->cameraActor->SetOrientation(0,0,90);
		camera->cameraActor->AddPosition(transform->GetPosition());
		camera->cameraActor->AddOrientation(transform->GetOrientation());
	}
	void ConfigureCameraParams(vtkCamera* camera, float* internalMatrix, float focalLength, unsigned int* imageSize, float* clippingRange)
	{
		camera->SetPosition(0, 0, 0);
		camera->SetViewUp(0, 1, 0);
		camera->SetFocalPoint(0,0,focalLength);
		double view_angle = 2.0 * atan(0.5 * imageSize[0] / internalMatrix[0]) * 180.0 / PI;
		camera->SetViewAngle(view_angle);
		camera->SetClippingRange(clippingRange[0],clippingRange[1]);
	}
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
void StereoVision::IsDebug(bool debug)
{
	mPimpl->mDebug = debug;
}
void StereoVision::LoadActor(const char* path)
{
	mPimpl->mObject.reader->SetFileName(path);
	mPimpl->mObject.reader->Update();
	mPimpl->mObject.mapper->SetInputData(mPimpl->mObject.reader->GetOutput());
	mPimpl->mObject.mapper->Update();
	mPimpl->mObject.actor->SetMapper(mPimpl->mObject.mapper);
	double* boundary = mPimpl->mObject.reader->GetOutput()->GetBounds();
	double x = boundary[0] + boundary[1];
	double y = boundary[2] + boundary[3];
	double z = boundary[4] + boundary[5];
	mPimpl->mObject.actor->AddPosition(-x*0.5,-y*0.5,-z*0.1);
}
void StereoVision::SetLeftCamera(CameraManager* camera)
{
	mLeft = camera;
	mLeft->IsDebug(mPimpl->mDebug);
}
void StereoVision::SetRightCamera(CameraManager* camera)
{
	mRight = camera;
	mRight->IsDebug(mPimpl->mDebug);
}
void StereoVision::RegisterCallback(void(*func)(unsigned char* imageLeft, CameraManager* left, unsigned char* imageRight, CameraManager* right, ReconActor* actor,bool debug))
{
	mFunc = func;
}
void StereoVision::Update()
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkCamera> cameraLeft;
	//Set Left camera parameters
	float* externalMatrix_l = mLeft->GetExternalMatrix();
	mPimpl->ConfigureCameraParams(cameraLeft,mLeft->GetInternalMatrix(),mLeft->mParams->FocalLength,mLeft->mParams->ImageSize,mLeft->mParams->ClippingRange);
	vtkNew<vtkMatrix4x4> cameraLeftMatrix;
	ARRAY_TO_VTK4X4MATRIX(cameraLeftMatrix, externalMatrix_l);
	cameraLeftMatrix->Invert();
	vtkNew<vtkTransform> cameraLeftTransform;
	cameraLeftTransform->SetMatrix(cameraLeftMatrix);
	cameraLeft->ApplyTransform(cameraLeftTransform);
	mPimpl->RenderCamera(cameraLeftTransform,2,&mPimpl->mLeft);
	RENDER_CAMERA_FRUSTUM(Left)

	vtkNew<vtkRenderWindow> renWinLeft;
	renWinLeft->SetWindowName("Left side camera image");
	renWinLeft->SetSize(mLeft->mParams->ImageSize[0],mLeft->mParams->ImageSize[1]);
	vtkNew<vtkRenderer> renderLeft;
	renWinLeft->AddRenderer(renderLeft);
	renderLeft->SetActiveCamera(cameraLeft);
	renderLeft->AddActor(mPimpl->mObject.actor);

	vtkNew<vtkCamera> cameraRight;
	//Set Right camera parameters
	float* externalMatrix_r = mRight->GetExternalMatrix();
	mPimpl->ConfigureCameraParams(cameraRight,mRight->GetInternalMatrix(),mRight->mParams->FocalLength,mRight->mParams->ImageSize,mRight->mParams->ClippingRange);
	vtkNew<vtkMatrix4x4> cameraRightMatrix;
	ARRAY_TO_VTK4X4MATRIX(cameraRightMatrix, externalMatrix_r);
	cameraRightMatrix->Invert();
	vtkNew<vtkTransform> cameraRightTransform;
	cameraRightTransform->SetMatrix(cameraRightMatrix);
	cameraRight->ApplyTransform(cameraRightTransform);
	mPimpl->RenderCamera(cameraRightTransform, 2, &mPimpl->mRight);

	RENDER_CAMERA_FRUSTUM(Right);
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
	worldAxesActor->SetTotalLength(10,10,10);
	//Renderer left and right camera actor
	vtkNew<vtkRenderer> renderer;
	vtkNew<vtkRenderWindow> renderWindow;
	renderWindow->AddRenderer(renderer);
	renderWindow->SetWindowName("StereoVision Demo");
	renderWindow->SetSize(1000,500);
	vtkNew<vtkRenderWindowInteractor> iren;
	iren->SetRenderWindow(renderWindow);
	renderer->AddActor(frustumActorLeft);
	renderer->AddActor(frustumActorRight);
	renderer->AddActor(worldAxesActor);
	renderer->AddActor(mPimpl->mLeft.cameraActor);
	renderer->AddActor(mPimpl->mRight.cameraActor);

	renderer->AddActor(mPimpl->mObject.actor);
	renderer->ResetCamera();
	renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());
	mPimpl->mReconActor.render = renderer;
	renderWindow->Render();
	mPimpl->mReconActor.renWin = renderWindow;
	renWinLeft->Render();
	renWinRight->Render();
	//Get renderer window image
	vtkNew<vtkWindowToImageFilter> windowImageLeft;
	windowImageLeft->SetInput(renWinLeft);
	windowImageLeft->SetScale(1);
	windowImageLeft->SetInputBufferTypeToRGB();
	windowImageLeft->ReadFrontBufferOff();
	windowImageLeft->Update();
	//Get virtual image plane of left camera
	RENDER_TEXTURE_PLANE(Left);

	renderer->AddActor(texturedPlaneLeft);

	vtkNew<vtkImageLuminance> luminanceLeft;
	luminanceLeft->SetInputData(windowImageLeft->GetOutput());
	luminanceLeft->Update();
	vtkNew<vtkWindowToImageFilter> windowImageRight;
	windowImageRight->SetInput(renWinRight);
	windowImageRight->SetScale(1);
	windowImageRight->SetInputBufferTypeToRGB();
	windowImageRight->ReadFrontBufferOff();
	windowImageRight->Update();
	//Get virtual image plane of right camera
	RENDER_TEXTURE_PLANE(Right);

	renderer->AddActor(texturedPlaneRight);

	vtkNew<vtkImageLuminance> luminanceRight;
	luminanceRight->SetInputData(windowImageRight->GetOutput());
	luminanceRight->Update();
	if (mFunc != nullptr)
	{
		mFunc((unsigned char*)luminanceLeft->GetOutput()->GetScalarPointer(),mLeft,(unsigned char*)luminanceRight->GetOutput()->GetScalarPointer(),mRight,&mPimpl->mReconActor,mPimpl->mDebug);
	}
	iren->Start();
}