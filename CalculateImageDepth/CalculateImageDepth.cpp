#include "CalculateImageDepth.h"
#include "../CameraModel/RenderingCamera.h"
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkArrowSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkNamedColors.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

#define ARRAY_TO_4X4MAT(a,b) for (unsigned int i = 0; i < 4; ++i)\
		{														\
			for (unsigned int j = 0; j < 4; ++j)						\
			{														\
			a.at<float>(i, j) = b[4 * i + j];\
			}																			\
		}
#define MAT_TO_VTKMATRIX(a,b,m,n)for (unsigned int i = 0; i < m; ++i)\
		{																		\
			for (unsigned int j = 0; j < n; ++j)									\
			{																	\
				a->SetElement(i, j, b.at<double>(i, j));\
			}																		\
		}
#define GetRotationMatrix(a,b) for (unsigned int i = 0; i < 3; ++i)\
		{\
			for (unsigned int j = 0; j < 3; ++j)\
			{\
				a.at<float>(i, j) = b.at<float>(i, j);\
			}\
		}
#define ARROW_FOR_DEBUG(transform,matrix,arrowSourceX,arrowSourceY,arrowSourceZ, mapperX,mapperY,mapperZ, arrowActor,scale,opacity,colors,colorName) vtkNew<vtkTransform> transform;\
	vtkNew<vtkNamedColors> colors;			\
	transform->SetMatrix(matrix);				\
	vtkNew<vtkArrowSource> arrowSourceX;						\
	vtkNew<vtkArrowSource> arrowSourceY;						\
	vtkNew<vtkArrowSource> arrowSourceZ;						\
	arrowSourceX->Update();									\
	arrowSourceY->Update();									\
	arrowSourceZ->Update();									\
	vtkNew<vtkPolyDataMapper> mapperX;				\
	vtkNew<vtkPolyDataMapper> mapperY;				\
	vtkNew<vtkPolyDataMapper> mapperZ;				\
	mapperX->SetInputConnection(arrowSourceX->GetOutputPort());\
	mapperY->SetInputConnection(arrowSourceY->GetOutputPort());\
	mapperZ->SetInputConnection(arrowSourceZ->GetOutputPort());\
	vtkNew<vtkActor> arrowActor##X;	\
	arrowActor##X->SetPosition(0,0,0);	\
	arrowActor##X->SetOrientation(90,0,0);	\
	arrowActor##X->AddPosition(transform->GetPosition());			\
	arrowActor##X->AddOrientation(transform->GetOrientation());		\
	arrowActor##X->SetMapper(mapperX);							\
	arrowActor##X->SetScale(scale);							\
	arrowActor##X->GetProperty()->SetOpacity(opacity);				\
	arrowActor##X->GetProperty()->SetColor(colors->GetColor3d(colorName).GetData());\
	vtkNew<vtkActor> arrowActor##Y;										\
	arrowActor##Y->SetPosition(0,0,0);			\
	arrowActor##Y->SetOrientation(0,90,0);		\
	arrowActor##Y->AddPosition(transform->GetPosition());			\
	arrowActor##Y->AddOrientation(transform->GetOrientation());		\
	arrowActor##Y->SetMapper(mapperY);							\
	arrowActor##Y->SetScale(scale);							\
	arrowActor##Y->GetProperty()->SetOpacity(opacity);				\
	arrowActor##Y->GetProperty()->SetColor(colors->GetColor3d(colorName).GetData());\
	vtkNew<vtkActor> arrowActor##Z;										\
	arrowActor##Z->SetPosition(0,0,0);			\
	arrowActor##Z->SetOrientation(0,0,90);		\
	arrowActor##Z->AddPosition(transform->GetPosition());			\
	arrowActor##Z->AddOrientation(transform->GetOrientation());		\
	arrowActor##Z->SetMapper(mapperZ);							\
	arrowActor##Z->SetScale(scale);							\
	arrowActor##Z->GetProperty()->SetOpacity(opacity);				\
	arrowActor##Z->GetProperty()->SetColor(colors->GetColor3d(colorName).GetData());

CalculateImageDepth::CalculateImageDepth()
{
	mStereoVision = new StereoVision;
	mLeft = new CameraManager;
	mRight = new CameraManager;
}
CalculateImageDepth::~CalculateImageDepth()
{
	if (mStereoVision != nullptr)
	{
		delete mStereoVision;
	}
	if (mLeft != nullptr)
	{
		delete mLeft;
	}
	if (mRight != nullptr)
	{
		delete mRight;
	}
}
void CalculateImageDepth::IsDebug(bool debug)
{
	mDebug = debug;
}
void CalculateImageDepth::LoadActor(const char* path)
{
	mStereoVision->LoadActor(path);
}
void CalculateImageDepth::SetCameraPos(float pos_l[3], float pos_r[3])
{
	mLeft->SetCameraPos(pos_l[0], pos_l[1], pos_l[2]);
	mRight->SetCameraPos(pos_r[0], pos_r[1], pos_r[2]);
}
void CalculateImageDepth::SetBoardSize(float board_l[2], float board_r[2])
{
	mLeft->SetBoardSize(board_l[0],board_l[1]);
	mRight->SetBoardSize(board_r[0],board_r[1]);
}
void CalculateImageDepth::SetCameraOrientation(float ori_l[3], float ori_r[3])
{
	mLeft->SetCameraOrientation(ori_l[0], ori_l[1], ori_l[2]);
	mRight->SetCameraOrientation(ori_r[0], ori_r[1], ori_r[2]);
}
void CalculateImageDepth::SetFocalLength(float length_l, float length_r)
{
	mLeft->SetFocalLength(length_l);
	mRight->SetFocalLength(length_r);
}
void CalculateImageDepth::SetImageSize(unsigned int sz_l[2], unsigned int sz_r[2])
{
	mLeft->SetImageSize(sz_l[0],sz_l[1]);
	mRight->SetImageSize(sz_r[0],sz_r[1]);
}
void CalculateImageDepth::SetClippingRange(float range_l[2], float range_r[2]) 
{
	mLeft->SetClippingRange(range_l[0],range_l[1]);
	mRight->SetClippingRange(range_r[0],range_r[1]);
}
void CalculateImageDepth::Update()
{
	mLeft->Update();
	mRight->Update();
	mStereoVision->IsDebug(mDebug);
	mStereoVision->SetLeftCamera(mLeft);
	mStereoVision->SetRightCamera(mRight);
	auto function = [](unsigned char* left, CameraManager* leftManager,unsigned char* right, CameraManager* rightManager,ReconActor* actor,bool debug)->void {
		cv::Mat leftImage(leftManager->mParams->ImageSize[0], leftManager->mParams->ImageSize[1], CV_8UC1,left);
		cv::Mat rightImage(rightManager->mParams->ImageSize[0], rightManager->mParams->ImageSize[1], CV_8UC1,right);
		cv::Mat leftImageFliped, rightImageFliped;
		cv::flip(leftImage,leftImageFliped,0);
		cv::flip(rightImage, rightImageFliped,0);
		if (debug)
		{
			cv::namedWindow("left", cv::WINDOW_FREERATIO);
			cv::imshow("left", leftImageFliped);
			cv::namedWindow("right", cv::WINDOW_FREERATIO);
			cv::imshow("right", rightImageFliped);
		}
		float* internalMatrix_l = leftManager->GetInternalMatrix();
		float* externalMatrix_l = leftManager->GetExternalMatrix();
		float* internalMatrix_r = rightManager->GetInternalMatrix();
		float* externalMatrix_r = rightManager->GetExternalMatrix();
		cv::Mat internalMatrix_l_(3, 3, CV_32FC1, internalMatrix_l);
		cv::Mat internalMatrix_r_(3, 3, CV_32FC1, internalMatrix_r);
		cv::Mat distCoeffs_l, distCoeffs_r;
		cv::Mat rotation_l(3, 3, CV_32FC1);
		cv::Mat rotation_r(3, 3, CV_32FC1);
		cv::Mat rotation_translation_l(4,4,CV_32FC1);
		cv::Mat rotation_translation_r(4,4,CV_32FC1);
		ARRAY_TO_4X4MAT(rotation_translation_l, externalMatrix_l);
		ARRAY_TO_4X4MAT(rotation_translation_r, externalMatrix_r);
		//Transformation matrix from camera coordinate to world coordinate (inverse of external matrix)
		rotation_translation_l = rotation_translation_l.inv();
		rotation_translation_r = rotation_translation_r.inv();
		vtkNew<vtkMatrix4x4> arrowMatrix1;
		ARRAY_TO_VTK4X4MATRIX(arrowMatrix1, externalMatrix_l, 4, 4);
		arrowMatrix1->Invert();
		ARROW_FOR_DEBUG(arrowTransform1,arrowMatrix1,arrowSource1X, arrowSource1Y, arrowSource1Z,arrowMapper1X, arrowMapper1Y, arrowMapper1Z,arrowActor1,10,0.5,colors1,"Blue");
		actor->render->AddActor(arrowActor1X);
		actor->render->AddActor(arrowActor1Y);
		actor->render->AddActor(arrowActor1Z);
		vtkNew<vtkMatrix4x4> arrowMatrix2;
		ARRAY_TO_VTK4X4MATRIX(arrowMatrix2, externalMatrix_r, 4, 4);
		arrowMatrix2->Invert();
		ARROW_FOR_DEBUG(arrowTransform2, arrowMatrix2, arrowSource2X, arrowSource2Y, arrowSource2Z, arrowMapper2X,arrowMapper2Y,arrowMapper2Z, arrowActor2,10,0.5, colors2,"Cyan");
		actor->render->AddActor(arrowActor2X);
		actor->render->AddActor(arrowActor2Y);
		actor->render->AddActor(arrowActor2Z);
		cv::Mat rotation_translation_r1 = rotation_translation_r * rotation_translation_l.inv();
		GetRotationMatrix(rotation_r, rotation_translation_r1);
		cv::Mat translation(3, 1, CV_32FC1);
		translation.at<float>(0) = rotation_translation_r1.at<float>(0,3);
		translation.at<float>(1) = rotation_translation_r1.at<float>(1,3);
		translation.at<float>(2) = rotation_translation_r1.at<float>(2,3);
		vtkNew<vtkMatrix4x4> arrowMatrix3;
		for (unsigned int i = 0; i < 4; ++i)
		{
			for (unsigned int j = 0; j < 4; ++j)
			{
				arrowMatrix3->SetElement(i, j, rotation_translation_r.at<float>(i, j));
			}
		}
		vtkMatrix4x4::Multiply4x4(arrowMatrix3,arrowMatrix1, arrowMatrix3);
		ARROW_FOR_DEBUG(arrowTransform3, arrowMatrix3, arrowSource3X,arrowSource3Y,arrowSource3Z, arrowMapper3X,arrowMapper3Y,arrowMapper3Z, arrowActor3,5,0.5, colors3,"Magenta");
		actor->render->AddActor(arrowActor3X);
		actor->render->AddActor(arrowActor3Y);
		actor->render->AddActor(arrowActor3Z);
		cv::Size imageSize(leftManager->mParams->ImageSize[0], leftManager->mParams->ImageSize[1]);
		cv::Mat R1, R2, P1, P2, Q;
		cv::stereoRectify(internalMatrix_l_, distCoeffs_l, internalMatrix_r_, distCoeffs_r, imageSize, rotation_r, translation, R1, R2, P1, P2, Q);
		vtkNew<vtkMatrix4x4> R1_,R2_;
		R1_->Identity();
		R2_->Identity();
		MAT_TO_VTKMATRIX(R1_, R1, 3, 3);
		MAT_TO_VTKMATRIX(R2_, R2, 3, 3);
		R1_->SetElement(0, 3, rotation_translation_l.at<float>(0,3));
		R1_->SetElement(1, 3, rotation_translation_l.at<float>(1,3));
		R1_->SetElement(2, 3, rotation_translation_l.at<float>(2,3));
		R2_->SetElement(0, 3, rotation_translation_r.at<float>(0,3));
		R2_->SetElement(1, 3, rotation_translation_r.at<float>(1,3));
		R2_->SetElement(2, 3, rotation_translation_r.at<float>(2,3));
		ARROW_FOR_DEBUG(arrowTransform4,R1_,arrowSource4X,arrowSource4Y,arrowSource4Z,arrowMapper4X,arrowMapper4Y,arrowMapper4Z,arrowActor4,5,1,colors4,"tomato");
		actor->render->AddActor(arrowActor4X);
		actor->render->AddActor(arrowActor4Y);
		actor->render->AddActor(arrowActor4Z);
		ARROW_FOR_DEBUG(arrowTransform5,R2_,arrowSource5X,arrowSource5Y,arrowSource5Z,arrowMapper5X,arrowMapper5Y,arrowMapper5Z,arrowActor5,5,1,colors5,"navy");
		actor->render->AddActor(arrowActor5X);
		actor->render->AddActor(arrowActor5Y);
		actor->render->AddActor(arrowActor5Z);
		cv::Mat mapLeftx, mapLefty;
		cv::initUndistortRectifyMap(internalMatrix_l_, distCoeffs_l, R1, P1, imageSize, CV_32FC1, mapLeftx, mapLefty);
		cv::Mat mapRightx, mapRighty;
		cv::initUndistortRectifyMap(internalMatrix_r_, distCoeffs_r, R2, P2, imageSize, CV_32FC1, mapRightx, mapRighty);
		cv::Mat leftImage_, rightImage_;
		cv::remap(leftImageFliped, leftImage_, mapLeftx, mapLefty, cv::INTER_LINEAR);
		cv::remap(rightImageFliped, rightImage_, mapRightx, mapRighty, cv::INTER_LINEAR);
		leftImageFliped.release();
		rightImageFliped.release();
		if (debug)
		{
			cv::namedWindow("rectifiedImageLeft");
			cv::imshow("rectifiedImageLeft", leftImage_);
			cv::namedWindow("rectifiedImageRight");
			cv::imshow("rectifiedImageRight", rightImage_);
		}
		cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(48, 9);
		cv::Mat disparity;
		stereo->compute(leftImage_, rightImage_, disparity);
		disparity = disparity / 16.0;
		if (debug)
		{
			cv::namedWindow("disparity", cv::WINDOW_FREERATIO);
			cv::imshow("disparity", disparity*128);
			cv::waitKey(0);
		}
		cv::Mat reconImage;
		cv::reprojectImageTo3D(disparity, reconImage, Q);
		for (unsigned int r = 0; r < reconImage.rows; ++r)
		{
			for (unsigned int c = 0; c < reconImage.cols; ++c)
			{
				actor->points->InsertNextPoint(reconImage.at<cv::Vec3f>(r, c)[0], reconImage.at<cv::Vec3f>(r, c)[1], -reconImage.at<cv::Vec3f>(r, c)[2]);
			}
		}
		actor->polydata->SetPoints(actor->points);
		actor->sphere->SetCenter(0, 0, 0);
		actor->sphere->SetRadius(0.1);
		actor->glyph3D->SetSourceConnection(actor->sphere->GetOutputPort());
		actor->glyph3D->SetInputData(actor->polydata);
		actor->glyph3D->Update();
		actor->mapper->SetInputConnection(actor->glyph3D->GetOutputPort());
		actor->actor->SetMapper(actor->mapper);
		actor->actor->GetProperty()->SetColor(actor->colors->GetColor3d("Salmon").GetData());
		vtkNew<vtkMatrix4x4> externalMatrix_l_;
		ARRAY_TO_VTK4X4MATRIX(externalMatrix_l_, externalMatrix_l,4,4);
		externalMatrix_l_->Invert();
		vtkNew<vtkMatrix4x4> cameraRectifiedMatrix;
		cameraRectifiedMatrix->Identity();
		MAT_TO_VTKMATRIX(cameraRectifiedMatrix, R1,3,3);
		cameraRectifiedMatrix->Invert();
		vtkMatrix4x4::Multiply4x4(cameraRectifiedMatrix, externalMatrix_l_, externalMatrix_l_);
		vtkNew<vtkTransform> externalTransform_l;
		externalTransform_l->SetMatrix(externalMatrix_l_);
		actor->actor->AddPosition(externalTransform_l->GetPosition());
		actor->actor->AddOrientation(externalTransform_l->GetOrientation());
		actor->render->AddActor(actor->actor);
		actor->renWin->Render();
	};
	mStereoVision->RegisterCallback(function);
	mStereoVision->Update();
}