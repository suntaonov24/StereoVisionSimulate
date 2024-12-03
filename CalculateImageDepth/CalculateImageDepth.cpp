#include "CalculateImageDepth.h"
#include "../CameraModel/RenderingCamera.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <thread>
#include <semaphore>

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
	mStereoVision->SetLeftCamera(mLeft);
	mStereoVision->SetRightCamera(mRight);
	cv::Mat leftImage(mLeft->mParams->ImageSize[0], mLeft->mParams->ImageSize[1], CV_8UC1);
	cv::Mat rightImage(mRight->mParams->ImageSize[0], mRight->mParams->ImageSize[1], CV_8UC1);
	mStereoVision->RegisterLeftImage(leftImage.ptr());
	mStereoVision->RegisterRightImage(rightImage.ptr());
	mStereoVision->Update();
	if (mDebug)
	{
		cv::namedWindow("left",cv::WINDOW_FREERATIO);
		cv::imshow("left",leftImage);
		cv::namedWindow("right",cv::WINDOW_FREERATIO);
		cv::imshow("right",rightImage);
		//cv::waitKey(0);
	}
	/*float* internalMatrix_l = mStereoVision->mLeft->GetInternalMatrix();
	float* externalMatrix_l = mStereoVision->mLeft->GetExternalMatrix();
	float* internalMatrix_r = mStereoVision->mRight->GetInternalMatrix();
	float* externalMatrix_r = mStereoVision->mRight->GetExternalMatrix();
	cv::Mat internalMatrix_l_(3, 3, CV_32FC1, internalMatrix_l);
	cv::Mat internalMatrix_r_(3, 3, CV_32FC1, internalMatrix_r);
	cv::Mat distCoeffs_1, distCoeffs_r;
	cv::Mat rotation_l(3,3,CV_32FC1);
	cv::Mat rotation_r(3,3,CV_32FC1);
	for (unsigned int i = 0; i < 3; ++i)
	{
		for (unsigned int j = 0; j < 3; ++j)
		{
			rotation_l.at<float>(i, j) = externalMatrix_l[4 * i + j];
			rotation_r.at<float>(i, j) = externalMatrix_r[4 * i + j];
		}
	}
	rotation_l = rotation_l * rotation_r.inv();
	cv::Mat translation(1, 3, CV_32FC1);
	translation.at<float>(0) = externalMatrix_l[3] - externalMatrix_r[3];
	translation.at<float>(1) = externalMatrix_l[7] - externalMatrix_r[7];
	translation.at<float>(2) = externalMatrix_l[11] - externalMatrix_r[11];
	cv::Size imageSize(mStereoVision->mLeft->mParams->ImageSize[0], mStereoVision->mLeft->mParams->ImageSize[1]);
	cv::Mat R1(3,3,CV_32FC1), R2(3,3,CV_32FC1), P1(3,4,CV_32FC1), P2(3,4,CV_32FC1), Q(4,4,CV_32FC1);
	try
	{
		cv::stereoRectify(internalMatrix_l_,distCoeffs_1 ,internalMatrix_r_,distCoeffs_r, imageSize,rotation_l,translation,R1,R2,P1,P2,Q);
	}
	catch (cv::Exception& error)
	{
		std::cout << error.what() << std::endl;
	}
	cv::Ptr<cv::StereoBM> stereo = cv::StereoBM::create(128, 15);
	cv::Mat disparity;
	try
	{
		stereo->compute(leftImage,rightImage,disparity);
	}
	catch (cv::Exception& error)
	{
		std::cout << error.what() << std::endl;
	}
	if (mDebug)
	{
		cv::namedWindow("disparity",cv::WINDOW_FREERATIO);
		cv::imshow("disparity",disparity);
		cv::waitKey(0);
	}
	cv::Mat reconImage;
	cv::reprojectImageTo3D(disparity,reconImage,Q);*/
}