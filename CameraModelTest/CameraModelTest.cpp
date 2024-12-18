//#include "../CameraModel/RenderingCamera.h"
#include "../CalculateImageDepth/CalculateImageDepth.h"

int main(int argc, char** argv)
{
	CalculateImageDepth calculator;
	calculator.IsDebug(true);
	calculator.LoadActor(argv[1]);
	float cameraLeftPos[] = {std::stof(argv[2]),std::stof(argv[3]),std::stof(argv[4])};
	float cameraRightPos[] = { std::stof(argv[5]),std::stof(argv[6]),std::stof(argv[7]) };
	calculator.SetCameraPos(cameraLeftPos,cameraRightPos);
	float boardLeftSize[] = {std::stof(argv[8]),std::stof(argv[9])};
	float boardRightSize[] = { std::stof(argv[10]),std::stof(argv[11]) };
	calculator.SetBoardSize(boardLeftSize,boardRightSize);
	float cameraLeftOri[] = { 0,-0.1,PI / 180 * 90 };
	float cameraRightOri[] = {0,0.1,PI / 180 * 90 };
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