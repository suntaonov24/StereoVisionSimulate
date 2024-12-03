//#include "../CameraModel/RenderingCamera.h"
#include "../CalculateImageDepth/CalculateImageDepth.h"

int main(int argc, char** argv)
{
	CalculateImageDepth calculator;
	calculator.IsDebug(true);
	calculator.LoadActor("C:\\VTK\\VTKData-master\\VTKData-master\\Data\\42400-IDGH.stl");
	float cameraLeftPos[] = {0,0,0};
	float cameraRightPos[] = {0.5,0,0};
	calculator.SetCameraPos(cameraLeftPos,cameraRightPos);
	float boardLeftSize[] = {10,10};
	float boardRightSize[] = {10,10};
	calculator.SetBoardSize(boardLeftSize,boardRightSize);
	float cameraLeftOri[] = {0,0,0};
	float cameraRightOri[] = {0,0,0};
	calculator.SetCameraOrientation(cameraLeftOri,cameraRightOri);
	calculator.SetFocalLength(20,20);
	unsigned int imageLeftSize[] = {512,512 };
	unsigned int imageRightSize[] = { 512,512 };
	calculator.SetImageSize(imageLeftSize,imageRightSize);
	float rangeLeft[] = {0.01,500};
	float rangeRight[] = {0.01,500};
	calculator.SetClippingRange(rangeLeft,rangeRight);
	calculator.Update();
	return 0;
}