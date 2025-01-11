#include "CalculateImageDepth.h"
#include "../CameraModel/RenderingCamera.h"
#include <Eigen/Eigen>
#include <vtkMatrix4x4.h>
#include <vtkTransform.h>
#include <vtkArrowSource.h>
#include <vtkSphereSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkNamedColors.h>
#include <vtkAxesActor.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <vector>

#define POINT_TO_LINE_DISTANCE(a,b,c,x,y) (a*x+b*y+c)/sqrt(a*a+b*b); 

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
		{																\
			for (unsigned int j = 0; j < 3; ++j)							\
			{														\
				a.at<float>(i, j) = b.at<float>(i, j);							\
			}															\
		}

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
void CalculateImageDepth::SetOpticalPtsOffset(float* offset_l, float* offset_r)
{
	mLeft->SetOpticalPtsOffset(offset_r[0],offset_r[1]);
	mRight->SetOpticalPtsOffset(offset_l[0],offset_r[1]);
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
	mStereoVision->SetRightCamera(mRight);
	mStereoVision->SetLeftCamera(mLeft);
	auto function = [](unsigned char* left, unsigned char* right, std::vector<CameraManager*>* cameraManager,unsigned int* cameraIdx,ReconActor* actor,bool debug)->void {
		unsigned int leftIdx = cameraIdx[0], rightIdx = cameraIdx[1];
		cv::Mat leftImage_rgb((*cameraManager)[leftIdx]->mParams->ImageSize[0], (*cameraManager)[leftIdx]->mParams->ImageSize[1], CV_8UC3, left);
		cv::Mat rightImage_rgb((*cameraManager)[rightIdx]->mParams->ImageSize[0], (*cameraManager)[rightIdx]->mParams->ImageSize[1], CV_8UC3,right);
		cv::Mat leftImage;
		cv::Mat rightImage;
		cv::cvtColor(leftImage_rgb,leftImage,cv::COLOR_RGB2GRAY);
		cv::cvtColor(rightImage_rgb,rightImage,cv::COLOR_RGB2GRAY);
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
		float* internalMatrix_l = (*cameraManager)[0]->GetInternalMatrix();
		float* externalMatrix_l = (*cameraManager)[0]->GetExternalMatrix();
		float* internalMatrix_r = (*cameraManager)[1]->GetInternalMatrix();
		float* externalMatrix_r = (*cameraManager)[1]->GetExternalMatrix();
		std::vector<cv::Point2f> featuresLeft,featuresRight;
		ExtractMatchedFeatures(leftImageFliped,rightImageFliped,featuresLeft,featuresRight,debug);
		cv::Mat F;
		std::vector<cv::Vec3f> epilinesLeft, epilinesRight;
		GetEpipolarLines(featuresLeft, featuresRight, epilinesLeft, epilinesRight,F,debug);
	};
	mStereoVision->RegisterCallback(function);
	mStereoVision->Update();
}

void CalculateImageDepth::ExtractMatchedFeatures(cv::Mat& leftImage, cv::Mat& rightImage, std::vector<cv::Point2f>& featuresLeft,std::vector<cv::Point2f>& featuresRight,bool debug)
{
	//TODO. find corresponding features in left camera and right camera, both of those features are matched with each other.
	std::vector<cv::KeyPoint> keyPointsLeft, keyPointsRight;
	cv::Mat descriptorsLeft, descriptorsRight;
	cv::Ptr<cv::Feature2D> detector = cv::BRISK::create();
	detector->detectAndCompute(leftImage,cv::noArray(),keyPointsLeft,descriptorsLeft);
	detector->detectAndCompute(rightImage,cv::noArray(),keyPointsRight,descriptorsRight);
	cv::Ptr<cv::DescriptorMatcher> featuresMatcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
	std::vector<std::vector<cv::DMatch>> matches;
	featuresMatcher->knnMatch(descriptorsLeft, descriptorsRight,matches,2);
	std::vector<cv::DMatch> goodMatches;
	for (unsigned int i = 0; i < matches.size(); ++i)
	{
		if (matches[i][0].distance < 0.9 * matches[i][1].distance)
		{
			goodMatches.push_back(matches[i][0]);
		}
	}
	for (unsigned int i = 0; i < goodMatches.size(); ++i)
	{
		featuresLeft.push_back(keyPointsLeft[goodMatches[i].queryIdx].pt);
		featuresRight.push_back(keyPointsRight[goodMatches[i].trainIdx].pt);
	}
	if (debug)
	{
		cv::Mat imageMatches;
		cv::drawMatches(leftImage,keyPointsLeft,rightImage,keyPointsRight,goodMatches,imageMatches,cv::Scalar::all(-1),
			cv::Scalar::all(-1),std::vector<char>(),cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
		cv::Mat H = cv::findHomography(featuresLeft,featuresRight,cv::RANSAC);
		std::vector<cv::Point2f> imageLeftCorners(4);
		imageLeftCorners[0] = cv::Point2f(0, 0);
		imageLeftCorners[1] = cv::Point2f((float)rightImage.cols, 0);
		imageLeftCorners[2] = cv::Point2f((float)rightImage.cols, (float)rightImage.rows);
		imageLeftCorners[3] = cv::Point2f(0, (float)rightImage.rows);
		std::vector<cv::Point2f> imageRightCorners;
		cv::perspectiveTransform(imageLeftCorners,imageRightCorners,H);
		cv::line(imageMatches,imageRightCorners[0]+cv::Point2f(((float)leftImage.cols,0)),
			imageRightCorners[1]+cv::Point2f((float)leftImage.cols,0),cv::Scalar(0,255,0),4);
		cv::line(imageMatches,imageRightCorners[1]+cv::Point2f(((float)leftImage.cols,0)),
			imageRightCorners[2]+cv::Point2f((float)leftImage.cols,0),cv::Scalar(0,255,0),4);
		cv::line(imageMatches,imageRightCorners[2]+cv::Point2f(((float)leftImage.cols,0)),
			imageRightCorners[3]+cv::Point2f((float)leftImage.cols,0),cv::Scalar(0,255,0),4);
		cv::line(imageMatches,imageRightCorners[3]+cv::Point2f(((float)leftImage.cols,0)),
			imageRightCorners[0]+cv::Point2f((float)leftImage.cols,0),cv::Scalar(0,255,0),4);

		cv::namedWindow("Good Matches' features",cv::WINDOW_FREERATIO);
		cv::imshow("Good Matches' features",imageMatches);
		cv::waitKey(0);
	}
}

void CalculateImageDepth::GetEpipolarLines(std::vector<cv::Point2f>& featuresLeft, std::vector<cv::Point2f>& featuresRight,std::vector<cv::Vec3f>& epilinesLeft,std::vector<cv::Vec3f>& epilinesRight, cv::Mat& F, bool debug )
{
	//TODO. calculate fundamental matrix.
	F = cv::findFundamentalMat(featuresLeft,featuresRight);
	cv::computeCorrespondEpilines(featuresLeft,1,F,epilinesRight);
	cv::computeCorrespondEpilines(featuresRight,2,F,epilinesLeft);
	if (debug)
	{
		float totalDistance = 0.0;
		for (unsigned int i = 0; i < featuresLeft.size(); ++i)
		{
			totalDistance += POINT_TO_LINE_DISTANCE(epilinesLeft[i][0],epilinesLeft[i][1],epilinesLeft[i][2],featuresLeft[i].x,featuresLeft[i].y);
			totalDistance += POINT_TO_LINE_DISTANCE(epilinesRight[i][0], epilinesRight[i][1], epilinesRight[i][2],featuresRight[i].x,featuresRight[i].y);
		}
		std::cout << "The total error distance is: " << totalDistance <<" pixels." << std::endl;
	}
}
void CalculateImageDepth::CalculateHomographyMatrix(cv::Mat& H)
{
	//TODO. calculate homography matrix.
}