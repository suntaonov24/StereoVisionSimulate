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
#include <vtkPanoramicProjectionPass.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkAppendFilter.h>
#include <vtkDataSetMapper.h>
#include <vtkLODActor.h>
#include <vtkLight.h>
#include <vtkLightActor.h>
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

#define RENDER_TEXTURE_PLANE(CameraID, idx) vtkNew<vtkPlaneSource> plane##CameraID;					\
	plane##CameraID->SetOrigin(-mCameraArray[idx]->mParams->ImageBoardSize[0] * 0.5, -mCameraArray[idx]->mParams->ImageBoardSize[0] * 0.5, mCameraArray[idx]->mParams->FocalLength);\
	plane##CameraID->SetPoint1(-mCameraArray[idx]->mParams->ImageBoardSize[0] * 0.5, mCameraArray[idx]->mParams->ImageBoardSize[1] * 0.5, mCameraArray[idx]->mParams->FocalLength);\
	plane##CameraID->SetPoint2(mCameraArray[idx]->mParams->ImageBoardSize[0] * 0.5, -mCameraArray[idx]->mParams->ImageBoardSize[1] * 0.5, mCameraArray[idx]->mParams->FocalLength);\
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

#define GET_RENDER_WINDOW_IMAGE(CameraID) vtkNew<vtkWindowToImageFilter> windowImage##CameraID; \
	windowImage##CameraID->SetInput(renWin##CameraID);						\
	windowImage##CameraID->SetScale(1);									\
	windowImage##CameraID->SetInputBufferTypeToRGB();							\
	windowImage##CameraID->ReadFrontBufferOff();							\
	windowImage##CameraID->Update();

struct ObjectModel
{
	vtkNew<vtkOBJReader> reader;
	vtkNew<vtkPolyDataMapper> mapper;
	vtkNew<vtkActor> actor;
};
struct CameraImage
{
	vtkNew<vtkNamedColors> colors;
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
	ObjectModel mObject;
	ReconActor mReconActor;
	CameraImage mLeft;
	CameraImage mRight;
	unsigned int mLeftIdx;
	unsigned int mRightIdx;
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
		camera->cameraActor->SetOrientation(0,90,0);
		camera->cameraActor->AddPosition(transform->GetPosition());
		camera->cameraActor->AddOrientation(transform->GetOrientation());
		camera->cameraActor->GetProperty()->SetColor(camera->colors->GetColor3d("LightSalmon").GetData());
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
	mCameraArray.push_back(camera);
	mCameraArray.back()->IsDebug(mPimpl->mDebug);
	mPimpl->mLeftIdx = mCameraArray.size() - 1;
}
void StereoVision::SetRightCamera(CameraManager* camera)
{
	mCameraArray.push_back(camera);
	mCameraArray.back()->IsDebug(mPimpl->mDebug);
	mPimpl->mRightIdx = mCameraArray.size() - 1;
}
void StereoVision::RegisterCallback(void(*func)(unsigned char* imageLeft, unsigned char* imageRight, std::vector<CameraManager*>* cameraArray, unsigned int* cameraIdx, ReconActor* actor,bool debug))
{
	mFunc = func;
}
void StereoVision::Update()
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkCamera> cameraLeft;
	//Set Left camera parameters
	float* externalMatrix_l = mCameraArray[mPimpl->mLeftIdx]->GetExternalMatrix();
	mPimpl->ConfigureCameraParams(cameraLeft, mCameraArray[mPimpl->mLeftIdx]->GetInternalMatrix(), mCameraArray[mPimpl->mLeftIdx]->mParams->FocalLength, mCameraArray[mPimpl->mLeftIdx]->mParams->ImageSize, mCameraArray[mPimpl->mLeftIdx]->mParams->ClippingRange);
	vtkNew<vtkMatrix4x4> cameraLeftMatrix;
	ARRAY_TO_VTK4X4MATRIX(cameraLeftMatrix, externalMatrix_l,4,4);
	cameraLeftMatrix->Invert();
	vtkNew<vtkTransform> cameraLeftTransform;
	cameraLeftTransform->SetMatrix(cameraLeftMatrix);
	cameraLeft->ApplyTransform(cameraLeftTransform);
	mPimpl->RenderCamera(cameraLeftTransform,2,&mPimpl->mLeft);
	RENDER_CAMERA_FRUSTUM(Left)

	vtkNew<vtkRenderWindow> renWinLeft;
	renWinLeft->SetWindowName("Left side camera image");
	renWinLeft->SetSize(mCameraArray[mPimpl->mLeftIdx]->mParams->ImageSize[0], mCameraArray[mPimpl->mLeftIdx]->mParams->ImageSize[1]);
	vtkNew<vtkRenderer> renderLeft;
	renWinLeft->AddRenderer(renderLeft);
	renderLeft->SetActiveCamera(cameraLeft);
	renderLeft->AddActor(mPimpl->mObject.actor);

	vtkNew<vtkCamera> cameraRight;
	//Set Right camera parameters
	float* externalMatrix_r = mCameraArray[mPimpl->mRightIdx]->GetExternalMatrix();
	mPimpl->ConfigureCameraParams(cameraRight, mCameraArray[mPimpl->mRightIdx]->GetInternalMatrix(), mCameraArray[mPimpl->mRightIdx]->mParams->FocalLength, mCameraArray[mPimpl->mRightIdx]->mParams->ImageSize, mCameraArray[mPimpl->mRightIdx]->mParams->ClippingRange);
	vtkNew<vtkMatrix4x4> cameraRightMatrix;
	ARRAY_TO_VTK4X4MATRIX(cameraRightMatrix, externalMatrix_r,4,4);
	cameraRightMatrix->Invert();
	vtkNew<vtkTransform> cameraRightTransform;
	cameraRightTransform->SetMatrix(cameraRightMatrix);
	cameraRight->ApplyTransform(cameraRightTransform);
	mPimpl->RenderCamera(cameraRightTransform, 2, &mPimpl->mRight);

	RENDER_CAMERA_FRUSTUM(Right);
	vtkNew<vtkRenderWindow> renWinRight;
	renWinRight->SetWindowName("Right side camera image");
	renWinRight->SetSize(mCameraArray[mPimpl->mRightIdx]->mParams->ImageSize[0], mCameraArray[mPimpl->mRightIdx]->mParams->ImageSize[1]);
	vtkNew<vtkRenderer> renderRight;
	renWinRight->AddRenderer(renderRight);
	renderRight->SetActiveCamera(cameraRight);
	renderRight->AddActor(mPimpl->mObject.actor);
	//Renderer world coordinate
	vtkNew<vtkMatrix4x4> worldMatrix;
	worldMatrix->Identity();
	vtkNew<vtkTransform> worldTransform;
	worldTransform->SetMatrix(worldMatrix);
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
	//Get left renderer window image
	GET_RENDER_WINDOW_IMAGE(Left);
	//Get virtual image plane of left camera
	RENDER_TEXTURE_PLANE(Left,0);
	renderer->AddActor(texturedPlaneLeft);
	//Get right renderer window image
	GET_RENDER_WINDOW_IMAGE(Right);
	//Get virtual image plane of right camera
	RENDER_TEXTURE_PLANE(Right,1);
	renderer->AddActor(texturedPlaneRight);
	unsigned int cameraIdx[] = {mPimpl->mLeftIdx,mPimpl->mRightIdx};
	if (mFunc != nullptr)
	{
		mFunc((unsigned char*)windowImageLeft->GetOutput()->GetScalarPointer(),(unsigned char*)windowImageRight->GetOutput()->GetScalarPointer(),&mCameraArray,cameraIdx,&mPimpl->mReconActor,mPimpl->mDebug);
	}
	iren->Start();
}