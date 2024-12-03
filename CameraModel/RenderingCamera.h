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
	void RegisterLeftImage(unsigned char* image);
	void RegisterRightImage(unsigned char* image);
	void Update();
	CameraManager* mLeft = nullptr;
	CameraManager* mRight = nullptr;
private:
	unsigned char* mLeftImage = nullptr;
	unsigned char* mRightImage = nullptr;
	StereoVisionImpl* mPimpl = nullptr;
};