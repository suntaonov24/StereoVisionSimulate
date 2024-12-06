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

struct ModelObject
{
	vtkNew<vtkOBJReader> reader;
	vtkNew<vtkSphereSource> sphere;
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
	void RenderCamera(float centerX,float centerY,float centerZ,float scale, CameraImage* camera)
	{
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
	if (std::string(path) != "")
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
	else
	{
		mPimpl->mObject.sphere->SetRadius(10);
		mPimpl->mObject.sphere->SetCenter(0, 0, 100);
		mPimpl->mObject.mapper->SetInputConnection(mPimpl->mObject.sphere->GetOutputPort());
		mPimpl->mObject.actor->SetMapper(mPimpl->mObject.mapper);
	}
	//mPimpl->mObject.mapper->ScalarVisibilityOff();
	
	
}
void StereoVision::SetLeftCamera(CameraManager* camera)
{
	mLeft = camera;
}
void StereoVision::SetRightCamera(CameraManager* camera)
{
	mRight = camera;
}
void StereoVision::RegisterCallback(void(*func)(unsigned char* imageLeft, CameraManager* left, unsigned char* imageRight, CameraManager* right, ReconActor* actor,bool debug))
{
	mFunc = func;
}
void StereoVision::Update()
{
	vtkNew<vtkNamedColors> colors;
	vtkNew<vtkCamera> cameraLeft;
	//vtkNew<vtkCameraActor> cameraActorLeft;
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
	double planesArrayLeft[24];
	cameraLeft->GetFrustumPlanes(1.0,planesArrayLeft);
	vtkNew<vtkPlanes> planesLeft;
	planesLeft->SetFrustumPlanes(planesArrayLeft);
	vtkNew<vtkFrustumSource> frustumLeft;
	frustumLeft->ShowLinesOff();
	frustumLeft->SetPlanes(planesLeft);
	vtkNew<vtkShrinkPolyData> shrinkLeft;
	shrinkLeft->SetInputConnection(frustumLeft->GetOutputPort());
	shrinkLeft->SetShrinkFactor(0.9);
	vtkNew<vtkPolyDataMapper> frustumMapperLeft;
	frustumMapperLeft->SetInputConnection(shrinkLeft->GetOutputPort());
	vtkNew<vtkProperty> backLeft;
	backLeft->SetColor(colors->GetColor3d("Tomato").GetData());
	backLeft->SetOpacity(0.1);
	vtkNew<vtkActor> frustumActorLeft;
	frustumActorLeft->SetMapper(frustumMapperLeft);
	frustumActorLeft->GetProperty()->EdgeVisibilityOff();
	frustumActorLeft->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());
	frustumActorLeft->SetBackfaceProperty(backLeft);
	frustumActorLeft->GetProperty()->SetOpacity(0.1);

	//cameraActorLeft->SetCamera(cameraLeft);
	vtkNew<vtkRenderWindow> renWinLeft;
	renWinLeft->SetWindowName("Left side camera image");
	renWinLeft->SetSize(mLeft->mParams->ImageSize[0],mLeft->mParams->ImageSize[1]);
	vtkNew<vtkRenderer> renderLeft;
	renWinLeft->AddRenderer(renderLeft);
	renderLeft->SetActiveCamera(cameraLeft);
	renderLeft->AddActor(mPimpl->mObject.actor);
	//renderLeft->AddViewProp(lightActor);

	vtkNew<vtkCamera> cameraRight;
	//vtkNew<vtkCameraActor> cameraActorRight;
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
	double planesArrayRight[24];
	cameraRight->GetFrustumPlanes(1.0, planesArrayRight);
	vtkNew<vtkPlanes> planesRight;
	planesRight->SetFrustumPlanes(planesArrayRight);
	vtkNew<vtkFrustumSource> frustumRight;
	frustumRight->ShowLinesOff();
	frustumRight->SetPlanes(planesRight);
	vtkNew<vtkShrinkPolyData> shrinkRight;
	shrinkRight->SetInputConnection(frustumRight->GetOutputPort());
	shrinkRight->SetShrinkFactor(0.9);
	vtkNew<vtkPolyDataMapper> frustumMapperRight;
	frustumMapperRight->SetInputConnection(shrinkRight->GetOutputPort());
	vtkNew<vtkProperty> backRight;
	backRight->SetColor(colors->GetColor3d("Tomato").GetData());
	backRight->SetOpacity(0.1);
	vtkNew<vtkActor> frustumActorRight;
	frustumActorRight->SetMapper(frustumMapperRight);
	frustumActorRight->GetProperty()->EdgeVisibilityOff();
	frustumActorRight->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());
	frustumActorRight->SetBackfaceProperty(backLeft);
	frustumActorRight->GetProperty()->SetOpacity(0.1);
	//cameraActorRight->SetCamera(cameraRight);
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
	//renderer->AddActor(cameraActorLeft);
	renderer->AddActor(frustumActorLeft);
	renderer->AddActor(frustumActorRight);
	//renderer->AddActor(cameraActorRight);
	renderer->AddActor(worldAxesActor);
	
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
	vtkNew<vtkPlaneSource> planeLeft;
	planeLeft->SetOrigin(-mLeft->mParams->ImageBoardSize[0] * 0.5, -mLeft->mParams->ImageBoardSize[0] * 0.5,mLeft->mParams->FocalLength);
	planeLeft->SetPoint1(-mLeft->mParams->ImageBoardSize[0]*0.5, mLeft->mParams->ImageBoardSize[1]*0.5, mLeft->mParams->FocalLength);
	planeLeft->SetPoint2(mLeft->mParams->ImageBoardSize[0]*0.5, -mLeft->mParams->ImageBoardSize[1]*0.5, mLeft->mParams->FocalLength);
	vtkNew<vtkImageFlip> flipLeftImage;
	flipLeftImage->SetInputData(windowImageLeft->GetOutput());
	flipLeftImage->SetFilteredAxis(0);
	vtkNew<vtkTexture> textureLeft;
	textureLeft->SetInputConnection(flipLeftImage->GetOutputPort());
	vtkNew<vtkTextureMapToPlane> texturePlaneLeft;
	texturePlaneLeft->SetInputConnection(planeLeft->GetOutputPort());
	vtkNew<vtkPolyDataMapper> planeMapperLeft;
	planeMapperLeft->SetInputConnection(texturePlaneLeft->GetOutputPort());
	vtkNew<vtkActor> texturedPlaneLeft;
	texturedPlaneLeft->SetMapper(planeMapperLeft);
	texturedPlaneLeft->SetTexture(textureLeft);
	texturedPlaneLeft->GetProperty()->SetOpacity(0.5);
	texturedPlaneLeft->AddPosition(cameraLeftTransform->GetPosition());
	texturedPlaneLeft->AddOrientation(cameraLeftTransform->GetOrientation());
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
	vtkNew<vtkPlaneSource> planeRight;
	planeRight->SetOrigin(-mRight->mParams->ImageBoardSize[0] * 0.5, -mRight->mParams->ImageBoardSize[1] * 0.5,mRight->mParams->FocalLength);
	planeRight->SetPoint1(-mRight->mParams->ImageBoardSize[0] * 0.5, mRight->mParams->ImageBoardSize[1] * 0.5, mRight->mParams->FocalLength);
	planeRight->SetPoint2(mRight->mParams->ImageBoardSize[0] * 0.5, -mRight->mParams->ImageBoardSize[1] * 0.5, mRight->mParams->FocalLength);
	vtkNew<vtkImageFlip> flipRightImage;
	flipRightImage->SetInputData(windowImageRight->GetOutput());
	flipRightImage->SetFilteredAxis(0);
	vtkNew<vtkTexture> textureRight;
	textureRight->SetInputConnection(flipRightImage->GetOutputPort());
	vtkNew<vtkTextureMapToPlane> texturePlaneRight;
	texturePlaneRight->SetInputConnection(planeRight->GetOutputPort());
	vtkNew<vtkPolyDataMapper> planeMapperRight;
	planeMapperRight->SetInputConnection(texturePlaneRight->GetOutputPort());
	vtkNew<vtkActor> texturedPlaneRight;
	texturedPlaneRight->SetMapper(planeMapperRight);
	texturedPlaneRight->SetTexture(textureRight);
	texturedPlaneRight->GetProperty()->SetOpacity(0.5);
	texturedPlaneRight->AddPosition(cameraRightTransform->GetPosition());
	texturedPlaneRight->AddOrientation(cameraRightTransform->GetOrientation());
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