#include "CameraParams.h"
#include <Eigen/Eigen>

CameraManager::CameraManager()
{
	mParams = new CameraParams;
}
CameraManager::~CameraManager()
{
	if (mParams != nullptr)
	{
		delete mParams;
	}
}
void CameraManager::SetCameraPos(float x, float y,float z)
{
	mParams->CameraPos[0] = x;
	mParams->CameraPos[1] = y;
	mParams->CameraPos[2] = z;
}

void CameraManager::SetCameraOrientation(float x, float y, float z)
{
	mParams->CameraOri[0] = x;
	mParams->CameraOri[1] = y;
	mParams->CameraOri[2] = z;
}

void CameraManager::SetImageSize(unsigned int x, unsigned int y)
{
	mParams->ImageSize[0] = x;
	mParams->ImageSize[1] = y;
}
void CameraManager::SetBoardSize(float x, float y)
{
	mParams->ImageBoardSize[0] = x;
	mParams->ImageBoardSize[1] = y;
}

void CameraManager::SetFocalLength(float focalLength)
{
	mParams->FocalLength = focalLength;
}
void CameraManager::SetClippingRange(float near, float far)
{
	mParams->ClippingRange[0] = near;
	mParams->ClippingRange[1] = far;
}
void CameraManager::Update()
{
	Eigen::Matrix3f rotationMatrix;
	rotationMatrix = Eigen::AngleAxisf(mParams->CameraOri[2], Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(mParams->CameraOri[1], Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(mParams->CameraOri[0], Eigen::Vector3f::UnitZ());
	float spacingX = mParams->ImageBoardSize[0] / mParams->ImageSize[0];
	float spacingY = mParams->ImageBoardSize[1] / mParams->ImageSize[1];
	//Calculate camera internal matrix
	Eigen::Matrix3f internalMatrix;
	internalMatrix << mParams->FocalLength / spacingX, 0.0, mParams->ImageSize[0] * 0.5,
		0.0, mParams->FocalLength / spacingY, mParams->ImageSize[1] * 0.5,
		0.0, 0.0, 1.0;
	std::cout << "Internal matrix: " << std::endl;
	std::cout << internalMatrix << std::endl;
	Eigen::Matrix3f internalMatrix_ = internalMatrix.transpose();
	memcpy(mInternalMatrix, internalMatrix_.data(), 9 * sizeof(float));
	//Calculate camera external matrix
	Eigen::Matrix4f externalMatrix;
	externalMatrix << rotationMatrix(0,0), rotationMatrix(0,1), rotationMatrix(0,2), mParams->CameraPos[0],
		rotationMatrix(1,0), rotationMatrix(1,1), rotationMatrix(1,2), mParams->CameraPos[1],
		rotationMatrix(2,0), rotationMatrix(2,1), rotationMatrix(2,2), mParams->CameraPos[2],
		0.0, 0.0, 0.0, 1.0;
	std::cout << "External matrix: " << std::endl;
	std::cout << externalMatrix << std::endl;
	Eigen::Matrix4f externalMatrix_ = externalMatrix.transpose();
	memcpy(mExternalMatrix, externalMatrix_.data(), 16 * sizeof(float));
}
float* CameraManager::GetInternalMatrix()
{
	return mInternalMatrix;
}

float* CameraManager::GetExternalMatrix()
{
	return mExternalMatrix;
}