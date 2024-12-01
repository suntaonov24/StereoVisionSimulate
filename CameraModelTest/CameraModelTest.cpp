#include "../CameraModel/RenderingCamera.h"

int main(int argc, char** argv)
{
	CameraManager manager_l;
	manager_l.SetCameraPos(0, 0, 0);
	manager_l.SetBoardSize(50,50);
	manager_l.SetCameraOrientation(0,0,0);
	manager_l.SetFocalLength(50);
	manager_l.SetImageSize(150,150);
	manager_l.SetClippingRange(0.01,500);
	manager_l.Update();
	CameraManager manager_r;
	manager_r.SetCameraPos(10,0,0);
	manager_r.SetBoardSize(50,50);
	manager_r.SetCameraOrientation(0,0,0);
	manager_r.SetFocalLength(50);
	manager_r.SetImageSize(150, 150);
	manager_r.SetClippingRange(0.01, 500);
	manager_r.Update();
	StereoVision vision;
	vision.LoadActor("C:\\VTK\\VTKData-master\\VTKData-master\\Data\\42400-IDGH.stl");
	vision.SetLeftCamera(&manager_l);
	vision.SetRightCamera(&manager_r);
	vision.Update();
	return 0;
}