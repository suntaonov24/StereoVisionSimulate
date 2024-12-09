#pragma once
#include <iostream>
#define __EXPORT_API__ __declspec(dllexport)
#define PI 3.141592653
struct CameraParams
{
	//the size of captured image [mm]
	float ImageBoardSize[2];
	//the size of current image [pixel]
	unsigned int ImageSize[2];
	//the focal length of current camera [mm]
	float FocalLength;
	//the position of current camera [mm]
	// reference URL: https://learnopengl.com/Getting-started/Camera
	float CameraPos[3];
	//the orientation that camera looked at (normalized vector)
	float CameraOri[3];
	//the clipping range that camera can capture the objec
	float ClippingRange[2];
};

class __EXPORT_API__ CameraManager
{
public:
	CameraManager();
	~CameraManager();
	void IsDebug(bool debug);
	void SetCameraPos(float x, float y, float z);
	void SetCameraOrientation(float x, float y, float z);
	void SetBoardSize(float x, float y);
	void SetImageSize(unsigned int x, unsigned int y);
	void SetFocalLength(float focalLength);
	void SetClippingRange(float near, float far);
	void Update();
	float* GetInternalMatrix();
	float* GetExternalMatrix();
	CameraParams* mParams = nullptr;
private:
	float mInternalMatrix[9];
	float mExternalMatrix[16];
	bool mDebug = false;
};