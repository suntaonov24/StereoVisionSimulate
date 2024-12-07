#include "CalculateImageDepth.h"
#include "../CameraModel/RenderingCamera.h"
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

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
		for (unsigned int i = 0; i < 3; ++i)
		{
			for (unsigned int j = 0; j < 3; ++j)
			{
				rotation_l.at<float>(i, j) = externalMatrix_l[4 * i + j];
				rotation_r.at<float>(i, j) = externalMatrix_r[4 * i + j];
			}
		}
		rotation_l = rotation_l * rotation_r.inv();
		cv::Mat translation(3, 1, CV_32FC1);
		translation.at<float>(0) = externalMatrix_l[3] - externalMatrix_r[3];
		translation.at<float>(1) = externalMatrix_l[7] - externalMatrix_r[7];
		translation.at<float>(2) = externalMatrix_l[11] - externalMatrix_r[11];
		cv::Size imageSize(leftManager->mParams->ImageSize[0], leftManager->mParams->ImageSize[1]);
		cv::Mat R1(3, 3, CV_32FC1), R2(3, 3, CV_32FC1), P1(3, 4, CV_32FC1), P2(3, 4, CV_32FC1), Q(4, 4, CV_32FC1);
		try
		{
			cv::stereoRectify(internalMatrix_l_, distCoeffs_l, internalMatrix_r_, distCoeffs_r, imageSize, rotation_l, translation, R1, R2, P1, P2, Q);
		}
		catch (cv::Exception& error)
		{
			std::cout << error.what() << std::endl;
		}
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
		cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(128, 15);
		cv::Mat disparity;
		try
		{
			stereo->compute(leftImage_, rightImage_, disparity);
		}
		catch (cv::Exception& error)
		{
			std::cout << error.what() << std::endl;
		}
		disparity = disparity / 16.0;
		if (debug)
		{
			cv::namedWindow("disparity", cv::WINDOW_FREERATIO);
			cv::imshow("disparity", disparity);
		}
		//std::cout<<"disparity type: "<<disparity.type() << std::endl;
		cv::Mat reconImage;
		cv::reprojectImageTo3D(disparity, reconImage, Q);
		if (debug)
		{
			cv::namedWindow("reconstructed image");
			cv::imshow("reconstructed image", reconImage);
			cv::waitKey(0);
		}
		std::cout << reconImage.type() << std::endl;
		for (unsigned int r = 0; r < reconImage.rows; ++r)
		{
			for (unsigned int c = 0; c < reconImage.cols; ++c)
			{
				actor->points->InsertNextPoint(reconImage.at<cv::Vec3f>(r, c)[0], reconImage.at<cv::Vec3f>(r, c)[1], reconImage.at<cv::Vec3f>(r, c)[2]);
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
		for (unsigned int i = 0; i < 4; ++i)
		{
			for (unsigned int j = 0; j < 4; ++j)
			{
				externalMatrix_l_->SetElement(i, j, externalMatrix_l[4 * i + j]);
			}
		}
		externalMatrix_l_->Print(std::cout);
		externalMatrix_l_->Invert();
		externalMatrix_l_->Print(std::cout);
		vtkNew<vtkTransform> externalTransform_l;
		externalTransform_l->SetMatrix(externalMatrix_l_);
		actor->actor->AddPosition(externalTransform_l->GetPosition());
		actor->actor->AddOrientation(externalTransform_l->GetOrientation());
		vtkNew<vtkAxesActor> cameraAxes;
		cameraAxes->AddPosition(externalTransform_l->GetPosition());
		cameraAxes->AddOrientation(externalTransform_l->GetOrientation());
		std::cout<<cameraAxes->GetOrigin()[0]<<std::endl;
		cameraAxes->SetTotalLength(50, 50, 50);
		actor->render->AddActor(actor->actor);
		actor->render->AddActor(cameraAxes);
		actor->renWin->Render();
	};
	mStereoVision->RegisterCallback(function);
	mStereoVision->Update();
}