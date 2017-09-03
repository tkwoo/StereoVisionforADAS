/**
@file main.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com).
@brief main file
*/

#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <string>
 
#include "StereoVisionForADAS.h"
#include "SurfaceNormal.h"

// #define VIZ
#ifdef VIZ
#include <opencv2/viz.hpp>
#endif

using namespace std;
using namespace cv;

int main()
{
#ifdef VIZ
	viz::Viz3d plot3d("Coordinate Frame");
	plot3d.showWidget("Coordinate Widget", viz::WCoordinateSystem());

	viz::Viz3d vizPlotVec("vector direction");
	vizPlotVec.showWidget("Coordinate Widget", viz::WCoordinateSystem());
#endif

	StereoCamParam_t objParam = CStereoVisionForADAS::InitStereoParam(KITTI);
	CStereoVisionForADAS objStereoVision(objParam); //stixel constructor
	CSuNoVeMap objSuNoVeMap(objParam); //SNV

	int cntFrame = 0;
	char chLeftImageName[150] = {};
	char chRightImageName[150] = {};
	char chOxtsName[150] = {};
	int nWaitTime = 0;
	
	while (1){// (!plot3d.wasStopped()){
		cout << cntFrame << ", ";

		sprintf(chLeftImageName, "./data/left/%010d.png", cntFrame);
		sprintf(chRightImageName, "./data/right/%010d.png", cntFrame);

		cntFrame++;

		/*fstream fs;
		fs.open(chOxtsName);
		double dTemp;
		fs >> dTemp >> dTemp >> dTemp >> dTemp >> dTemp;
		printf("%.1lfdeg, ", -dTemp*180/PI);*/
		//cout << dTemp * 180 / PI << "deg, ";

		Mat imgLeft = imread(chLeftImageName, 1);
		Mat imgRight = imread(chRightImageName, 1);
		if (imgLeft.empty()) {waitKey();break;}
		
		//imshow("right", imgRight);
		
		int64 t = getTickCount();

		// procesing
		objStereoVision.Objectness(imgLeft, imgRight);

		Mat imgDisp8;
		objStereoVision.m_matDisp16.convertTo(imgDisp8, CV_8U, 255 / (objParam.m_nNumberOfDisp*16.));

		Mat imgG;
		bitwise_and(objStereoVision.m_imgGround, imgDisp8, imgG);
		// surface normal prcessing
		//objSuNoVeMap.Compute(objStereoVision.m_imgDisp8);
		//objSuNoVeMap.Compute(imgG);
		objSuNoVeMap.Compute(imgDisp8);
		printf("Time elapsed: %.3fms\n", (getTickCount() - t) * 1000 / getTickFrequency());
		
		Mat imgResult = imgLeft.clone();
		Mat imgStixel = imgLeft.clone();

		Mat imgDispColor;
		applyColorMap(imgDisp8, imgDispColor, COLORMAP_OCEAN);
		//cout << objSuNoVeMap.m_objValidSNVs.size() << endl;
		for (int i = 0; i < objSuNoVeMap.m_objValidSNVs.size(); i = i + 1){
			Point ptEnd;
			Point3d ptDir = 0.15*objSuNoVeMap.m_objValidSNVs[i].m_vec3dDirection / objSuNoVeMap.m_objValidSNVs[i].dScale;
			Point3d ptEnd3D = ptDir + objSuNoVeMap.m_objValidSNVs[i].m_ptPositionRealWorld;
			ptEnd.x = ptEnd3D.x * objParam.objCamParam.m_dFocalLength / ptEnd3D.z + (objParam.objCamParam.m_sizeSrc.width / 2);
			ptEnd.y = ptEnd3D.y * objParam.objCamParam.m_dFocalLength / ptEnd3D.z + (objParam.objCamParam.m_sizeSrc.height / 2);
			line(imgDispColor, objSuNoVeMap.m_objValidSNVs[i].m_ptPositionImage, ptEnd, Scalar(0, 255, 0));
		}
		for (int i = 0; i < objSuNoVeMap.m_objTrashSNVs.size(); i = i + 1){
			line(imgDispColor, objSuNoVeMap.m_objTrashSNVs[i].m_ptPositionImage, objSuNoVeMap.m_objTrashSNVs[i].m_ptPositionImage, Scalar(0, 0, 255));
		}

#ifdef VIZ
		vector<Point3f> depthPts;
		vector<Vec3b> colorPts;
		for (int i = 0; i < imgDisp8.rows; i++){
			for (int j = 0; j < imgDisp8.cols; j++){
				if (imgDisp8.at<uchar>(i, j) == 0) continue;
				//float z = objParam.objCamParam.m_dFocalLength * objParam.m_dBaseLine / ((float)imgDisp8.at<uchar>(i, j)*(float)objParam.m_nNumberOfDisp / 255);
				float z = objParam.objCamParam.m_dFocalLength * objParam.m_dBaseLine / ((float)objStereoVision.m_matDisp16.at<short>(i, j) / 16);
				if (z < 60 && z > 0){
					Point3f temp_point((j - 621)*z / objParam.objCamParam.m_dFocalLength, -(i - 188)*z / objParam.objCamParam.m_dFocalLength, -z);
					depthPts.push_back(temp_point);
					colorPts.push_back(imgLeft.at<Vec3b>(i, j));
				}
			} 
		}

		cv::viz::WCloud cloud_widget = cv::viz::WCloud(depthPts, colorPts);
		cloud_widget.setRenderingProperty(cv::viz::POINT_SIZE, 2);

		plot3d.showWidget("ref_cloud", cloud_widget);

		vector<Point3f> vecDir;
		vector<Vec3b> vecColor;
		for (int i = 0; i < objSuNoVeMap.m_objValidSNVs.size(); i++){
			if (objSuNoVeMap.m_objValidSNVs[i].dScale > 10) continue;
			Point3f ptTemp(objSuNoVeMap.m_objValidSNVs[i].m_vec3dDirection[0], -objSuNoVeMap.m_objValidSNVs[i].m_vec3dDirection[1], objSuNoVeMap.m_objValidSNVs[i].m_vec3dDirection[2]);
			ptTemp /= objSuNoVeMap.m_objValidSNVs[i].dScale;
			vecDir.push_back(ptTemp);
			vecColor.push_back(Vec3b(255, 255, 255));
		}

		cv::viz::WCloud cw = cv::viz::WCloud(vecDir, vecColor);
		cw.setRenderingProperty(cv::viz::POINT_SIZE, 2);
		vizPlotVec.showWidget("cloud", cw);
		
		vizPlotVec.spinOnce(1, true);
		plot3d.spinOnce(1, true);
#endif
		objStereoVision.Display(imgResult, imgStixel);
		
		resize(imgLeft, imgLeft, Size(imgLeft.cols / 2, imgLeft.rows / 2));
		resize(imgDisp8, imgDisp8, Size(imgDisp8.cols / 2, imgDisp8.rows / 2));
		resize(imgDispColor, imgDispColor, Size(imgDispColor.cols / 2, imgDispColor.rows / 2));
		resize(imgResult, imgResult, Size(imgLeft.cols, imgLeft.rows));
		resize(imgStixel, imgStixel, Size(imgLeft.cols, imgLeft.rows));
	
		//imshow("left", imgLeft);
		//imshow("disp8", imgDisp8);
		imshow("vec", imgDispColor);
		imshow("result", imgResult);
		imshow("stixel", imgStixel);

		char chKey = waitKey(nWaitTime);
		if (chKey == 27) return 1;
		if (chKey == ' ') nWaitTime = !nWaitTime;
		if (chKey == '[') cntFrame -= 2;
	}

	return 0;
}