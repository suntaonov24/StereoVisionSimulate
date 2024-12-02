#include "CalculateImageDepth.h"
#include "../CameraModel/RenderingCamera.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

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
		cv::waitKey(0);
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
}