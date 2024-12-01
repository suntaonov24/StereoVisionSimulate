#pragma once
#include <iostream>
#include "CameraParams.h"
class StereoVisionImpl;
class __EXPORT_API__ StereoVision
{
public:
	StereoVision();
	~StereoVision();
	void LoadActor(const char* imagePath);
	void SetLeftCamera(CameraManager* camera);
	void SetRightCamera(CameraManager* camera);
	//Registrate call back function for calculating disparity map
	void GetCameraImageCallBack(void(*func)(unsigned char* imageLeft,CameraParams* paramsLeft, unsigned char* imageRight,CameraParams* paramsRight));
	void Update();
private:
	CameraManager* mLeft = nullptr;
	CameraManager* mRight = nullptr;
	void (*mFunc)(unsigned char*, CameraParams*, unsigned char*, CameraParams*) = nullptr;
	StereoVisionImpl* mPimpl = nullptr;
};