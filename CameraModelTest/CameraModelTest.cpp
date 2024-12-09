//#include "../CameraModel/RenderingCamera.h"
#include "../CalculateImageDepth/CalculateImageDepth.h"

int main(int argc, char** argv)
{
	CalculateImageDepth calculator;
	calculator.IsDebug(true);
	calculator.LoadActor(argv[1]);
	float cameraLeftPos[] = {50,0,0};
	float cameraRightPos[] = {45,0,0};
	calculator.SetCameraPos(cameraLeftPos,cameraRightPos);
	float boardLeftSize[] = {10,10};
	float boardRightSize[] = {10,10};
	calculator.SetBoardSize(boardLeftSize,boardRightSize);
	float cameraLeftOri[] = { 0,0,PI / 180 * 90 };
	float cameraRightOri[] = {0,0,PI / 180 * 90 };
	calculator.SetCameraOrientation(cameraLeftOri,cameraRightOri);
	calculator.SetFocalLength(10,10);
	unsigned int imageLeftSize[] = {512,512 };
	unsigned int imageRightSize[] = { 512,512 };
	calculator.SetImageSize(imageLeftSize,imageRightSize);
	float rangeLeft[] = {0.01,200};
	float rangeRight[] = {0.01,200};
	calculator.SetClippingRange(rangeLeft,rangeRight);
	calculator.Update();
	return 0;
}