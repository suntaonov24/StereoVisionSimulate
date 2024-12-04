#pragma once
#include <iostream>
#include "CameraParams.h"
class StereoVisionImpl;
class __EXPORT_API__ StereoVision
{
public:
	StereoVision();
	~StereoVision();
	void IsDebug(bool debug);
	void LoadActor(const char* imagePath);
	void SetLeftCamera(CameraManager* camera);
	void SetRightCamera(CameraManager* camera);
	//Registrate call back function for calculating disparity map
	void RegisterCallback(void(*func)(unsigned char* imageLeft, CameraManager* left,unsigned char* imageRight, CameraManager* right,bool debug));
	void Update();
	CameraManager* mLeft = nullptr;
	CameraManager* mRight = nullptr;
private:
	StereoVisionImpl* mPimpl = nullptr;
	void (*mFunc)(unsigned char* imageLeft, CameraManager* left, unsigned char* imageRight, CameraManager* right, bool debug) = nullptr;
};