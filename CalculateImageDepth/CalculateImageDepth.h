#include <iostream>
#include "../CameraModel/RenderingCamera.h"

class __EXPORT_API__ CalculateImageDepth
{
public:
	CalculateImageDepth();
	~CalculateImageDepth();
	void IsDebug(bool debug);
	void LoadActor(const char* path);
	void SetCameraPos(float pos_l[3], float pos_r[3]);
	void SetBoardSize(float board_l[2], float board_r[2]);
	void SetCameraOrientation(float ori_l[3],float ori_r[3]);
	void SetFocalLength(float length_l,float length_r);
	void SetImageSize(unsigned int sz_l[2], unsigned int sz_r[2]);
	void SetClippingRange(float range_l[2],float range_r[2]);
	void Update();
private:
	CameraManager* mLeft = nullptr;
	CameraManager* mRight = nullptr;
	StereoVision* mStereoVision = nullptr;
	bool mDebug = false;
};
