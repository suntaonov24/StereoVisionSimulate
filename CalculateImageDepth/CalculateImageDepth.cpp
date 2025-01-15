#include "CalculateImageDepth.h"
#include "../CameraModel/RenderingCamera.h"
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
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
		Eigen::Matrix3f F_;
		F_ << F.at<float>(0, 0), F.at<float>(0, 1), F.at<float>(0, 2),
			F.at<float>(1, 0), F.at<float>(1, 1), F.at<float>(1, 2),
			F.at<float>(2, 0), F.at<float>(2, 1), F.at<float>(2, 2);
		Eigen::EigenSolver<Eigen::Matrix3f> es;
		es.compute(F_,true);
		Eigen::Vector3cf values = es.eigenvalues();
		Eigen::Matrix3cf vectors = es.eigenvectors();
		if (debug)
		{
			std::cout << "Eigen values are: " << values << std::endl;
			std::cout << "Eigen vectors are: " << vectors << std::endl;
		}
		unsigned int maximalIndex = 0;
		for (unsigned int i = 1; i < 3; ++i)
		{
			if (values(i).real() > values(i - 1).real())
				maximalIndex = i;
		}
		Eigen::Vector3f epipolar;
		epipolar << vectors(0, maximalIndex).real(), vectors(1, maximalIndex).real(), vectors(2, maximalIndex).real();
		Eigen::Matrix3f epipolarMatrix;
		epipolarMatrix << 0, -epipolar(2), epipolar(1),
			epipolar(2), 0, -epipolar(1),
			-epipolar(1), epipolar(0), 0;
		if (debug)
		{
			std::cout << "The corresponding matrix of epipolar is: " << epipolarMatrix << std::endl;
		}
		cv::Mat H(3, 3, CV_32FC1);
		CalculateHomographyMatrix(leftImage,rightImage,epipolarMatrix,F_,H);
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
void CalculateImageDepth::CalculateHomographyMatrix(cv::Mat& leftImage, cv::Mat& rightImage, Eigen::Matrix3f& eMat, Eigen::Matrix3f& FMat, cv::Mat& H)
{
	//TODO. calculate homography matrix.
	auto GetABParams = [](cv::Mat& image,Eigen::Matrix3f& eMat,Eigen::Matrix3f& A, Eigen::Matrix3f& B) ->void{
		int width = image.cols;
		int height = image.rows;
		Eigen::Matrix3f PPT;
		PPT << width * width - 1, 0, 0,
			0, height* height, 0,
			0, 0, 0;
		PPT /= (width * height / 12);
		Eigen::Matrix3f PcPcT;
		PcPcT << (width - 1) * (width - 1), (width - 1)* (height - 1), 2 * (width - 1),
			(width - 1)* (height - 1), (height - 1)* (height - 1), 2 * (height - 1),
			2 * (width - 1), 2 * (height - 1), 4;
		PcPcT /= 4;
		A = eMat.transpose() * PPT * eMat;
		B = eMat.transpose() * PcPcT * eMat;
	};
	Eigen::Matrix3f A, B, Ap, Bp;
	GetABParams(leftImage, eMat, A, B);
	GetABParams(rightImage, FMat, Ap, Bp);
	auto GetMaximizeZValue = [](Eigen::Matrix3f& A, Eigen::Matrix3f& B)->Eigen::Vector3f {
		Eigen::LLT<Eigen::Matrix3f> lltOfMatA;
		Eigen::Matrix3f LOfMatA = lltOfMatA.matrixL().transpose();
		Eigen::Matrix3f D_TBD_1 = LOfMatA.inverse().transpose() * B * LOfMatA.inverse();
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
		es.compute(D_TBD_1);
		Eigen::Vector3f values = es.eigenvalues();
		Eigen::Matrix3f vectors = es.eigenvectors();
		unsigned int maxIndex = 0;
		for (unsigned i = 1; i < 3; ++i)
		{
			if (values[i] > values[i - 1])maxIndex = i;
		}
		return LOfMatA.inverse()*vectors.col(maxIndex);
	};
	Eigen::Vector3f ZValue = GetMaximizeZValue(A,B) + GetMaximizeZValue(Ap,Bp);
	auto Equation = [&](float x)->float {
		float a = B(0, 0) * x * x + (B(0, 1) + B(1, 0)) * x + B(1, 1);
		float b = (2 * x * A(0, 0) + (A(0, 1) + A(1, 0))) * a - (2 * B(0, 0) + (B(0, 1) + B(1, 0))) * (A(0, 0) * x * x + (A(0, 1) + A(1, 0) * x + A(1, 1)));
		float c = Bp(0, 0) * x * x + (Bp(0, 1) + Bp(1, 0)) * x + Bp(1, 1);
		float d = (2 * x * Ap(0, 0) + (Ap(0, 1) + Ap(1, 0))) * c - (2 * Bp(0, 0) + (Bp(0, 1) + Bp(1, 0))) * (Ap(0, 0) * x * x + (Ap(0, 1) + Ap(1, 0) * x + Ap(1, 1)));
		return b/(a*a) + d/(c*c);
	};
	auto derivative = [&](float x)->float {
		float a = B(0, 0) * x * x + (B(0, 1) + B(1, 0)) * x + B(1, 1);
		float b = (2 * A(0, 0)) * a + (2 * x * A(0, 0) + (A(0, 1) + A(1, 0))) * (2 * B(0, 0) * x + (B(0, 1) + B(1, 0))) - (2 * B(0, 0) + (B(0, 1) + B(1, 0))) * (2 * A(0, 0) * x + (A(0, 1) + A(1, 0)));
		float c = Bp(0, 0) * x * x + (Bp(0, 1) + Bp(1, 0)) * x + Bp(1, 1);
		float d = (2 * Ap(0, 0)) * a + (2 * x * Ap(0, 0) + (Ap(0, 1) + Ap(1, 0))) * (2 * Bp(0, 0) * x + (Bp(0, 1) + Bp(1, 0))) - (2 * Bp(0, 0) + (Bp(0, 1) + Bp(1, 0))) * (2 * Ap(0, 0) * x + (Ap(0, 1) + Ap(1, 0)));
	};
	auto GetOptimizedZValue = [&](Eigen::Vector3f& x, int iterationNumber)->void {
		float fx = Equation(x(0));
		float dfx = derivative(x(0));
		int iteration = 0;
		while (abs(fx) > 0.0001 && iteration < iterationNumber)
		{
			fx = Equation(x(0) - fx / dfx);
			dfx = derivative(x(0) - fx / dfx);
			++iteration;
		}
	};
	GetOptimizedZValue(ZValue, 100);
	
}