/**
@file SurfaceNormal.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief create surface normal based on disparity map
*/
#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>
#include "./include/DefStruct.h"
#include <fstream>

using namespace std;
using namespace cv;

#define VECTORSTEP 4 /// Each vector is VECTORSTEP away from next vector because image has too many pixels
#define POINTSTEP 10 /// When vector is calculated, a point is POINTSTEP away from other point(pixel)

class CSurfaceNormal{
private:
	 
public:
	Point m_ptPositionImage;
	Point3d m_ptPositionRealWorld;
	Vec3d m_vec3dDirection;
	double dScale;

	CSurfaceNormal(){}

	CSurfaceNormal(Mat& imgDisp8, Point& ptImgLoc){}
	int SNV_point(Mat& imgDisp8, Mat& matDepth, Point ptImgLoc){
		//cout << ptImgLoc << endl;
		m_ptPositionImage = ptImgLoc;
		m_ptPositionRealWorld = matDepth.at<Vec3d>(ptImgLoc.y, ptImgLoc.x);

		if (imgDisp8.at<uchar>(ptImgLoc) == 0
			|| matDepth.at<Vec3d>(ptImgLoc.y + POINTSTEP, ptImgLoc.x) == Vec3d(0, 0, 0)
			|| matDepth.at<Vec3d>(ptImgLoc.y, ptImgLoc.x + POINTSTEP) == Vec3d(0, 0, 0)) {
			dScale = 0;
			m_vec3dDirection = Vec3d(0, 0, 0);
			return -1;
		}
		/*printf("%d, ", imgDisp8.at<uchar>(ptImgLoc.y, ptImgLoc.x));
		cout << matDepth.at<Vec3d>(ptImgLoc.y + 1, ptImgLoc.x) << ", ";
		cout << matDepth.at<Vec3d>(ptImgLoc.y, ptImgLoc.x + 1) << endl;*/

		Vec3d a = matDepth.at<Vec3d>(ptImgLoc.y + POINTSTEP, ptImgLoc.x) - matDepth.at<Vec3d>(ptImgLoc.y, ptImgLoc.x);
		Vec3d b = matDepth.at<Vec3d>(ptImgLoc.y, ptImgLoc.x + POINTSTEP) - matDepth.at<Vec3d>(ptImgLoc.y, ptImgLoc.x);
		m_vec3dDirection = a.cross(b);
		dScale = norm(m_vec3dDirection, NORM_L2);
		return 0;
	}

};
class CSuNoVeMap{
private:
	Mat m_imgDisp8;

	int m_nNumberOfDisparities;
	double m_dFocalLength;
	double m_dBaseLine;

public:
	Mat m_matDepth;
	//vector<CSurfaceNormal> m_vecNormals;
	vector<CSurfaceNormal> m_objValidSNVs;
	vector<CSurfaceNormal> m_objTrashSNVs;

	CSuNoVeMap(){}
	CSuNoVeMap(StereoCamParam_t objParam){
		m_matDepth.create(objParam.objCamParam.m_sizeSrc.height, objParam.objCamParam.m_sizeSrc.width, CV_64FC3);
		m_nNumberOfDisparities = objParam.m_nNumberOfDisp;
		m_dFocalLength = objParam.objCamParam.m_dFocalLength;
		m_dBaseLine = objParam.m_dBaseLine;
	}
	CSuNoVeMap(Mat& imgDisp8){
		m_imgDisp8 = imgDisp8;
		m_matDepth.create(m_imgDisp8.rows, m_imgDisp8.cols, CV_64FC3);
		m_nNumberOfDisparities = 48;
		m_dFocalLength = 1200;
		m_dBaseLine = 0.25;
		DisparityToDepth(imgDisp8);
		CalSNV(imgDisp8);
	}
	CSuNoVeMap(Mat& imgDisp8, int& nNumberOfDisparities, double dFocalLength, double dBaseLine){
		m_imgDisp8 = imgDisp8;
		m_nNumberOfDisparities = nNumberOfDisparities;
		m_dBaseLine = dBaseLine;
		m_dFocalLength = dFocalLength;
		m_matDepth.create(m_imgDisp8.rows, m_imgDisp8.cols, CV_64FC3);
	}
	int Compute(Mat& imgDisp8){
		m_imgDisp8 = imgDisp8;
		DisparityToDepth(imgDisp8);
		CalSNV(imgDisp8);
		SuperSNV(m_objValidSNVs);
		return 0;
	}
	int CalSNV(Mat& imgDisp8){
		//m_vecNormals.clear();
		m_objTrashSNVs.clear();
		m_objValidSNVs.clear();
		for (int i = 0; i < imgDisp8.rows - POINTSTEP; i += VECTORSTEP){
			for (int j = 0; j < imgDisp8.cols - POINTSTEP; j += VECTORSTEP){
				CSurfaceNormal objSN;
				objSN.SNV_point(imgDisp8, m_matDepth, Point(j, i));
				//m_vecNormals.push_back(objSN);
				(objSN.dScale > 0) ? m_objValidSNVs.push_back(objSN) : m_objTrashSNVs.push_back(objSN);
			}
		}
		return 0;
	}
	int DisparityToDepth(Mat& imgDisp){
		for (int i = 0; i < imgDisp.rows; i++){
			for (int j = 0; j < imgDisp.cols; j++){
				if (imgDisp.at<uchar>(i, j) == 0) {
					m_matDepth.at<Vec3d>(i, j) = Vec3d(0, 0, 0);
					continue;
				}
				double z = m_dFocalLength * m_dBaseLine / ((double)imgDisp.at<uchar>(i, j)*(double)m_nNumberOfDisparities / 255);
				double x = (j - imgDisp.cols/2)*z / m_dFocalLength;
				double y = (i - imgDisp.rows/2)*z / m_dFocalLength;
				m_matDepth.at<Vec3d>(i, j) = Vec3d(x, y, z);
			}
		}
		return 0;
	}
	int SuperSNV(vector<CSurfaceNormal>& objValid){
		/*fstream fs;
		fs.open("Pitch5pixel29frame.txt", ios::out);*/

		Mat matPitch(1, objValid.size(), CV_32FC1);
		for (int i = 0; i < objValid.size(); i++){
			/*if (objValid[i].m_ptPositionRealWorld.z < 4 || objValid[i].m_ptPositionRealWorld.z > 30 ||
				objValid[i].m_ptPositionRealWorld.x < -7 || objValid[i].m_ptPositionRealWorld.x < 7 ) continue;*/
			//cout << "hi";
			Point3d ptTemp = objValid[i].m_vec3dDirection;
			double dYZprojDir = (sqrt(ptTemp.y*ptTemp.y + ptTemp.z*ptTemp.z) < 0.01) ? 0 : sqrt(ptTemp.y*ptTemp.y + ptTemp.z*ptTemp.z);
			matPitch.at<float>(0, i) = 90. - (float)(acos(ptTemp.z / dYZprojDir) * 180 / PI);
			if (dYZprojDir == 0) matPitch.at<float>(0, i) = 100;
			//fs << matPitch.at<float>(0, i) << endl;

		}

		//fs.close();

		int nHistSize = 100;
		float fRange[] = { -5.0f, 5.0f };
		const float* ptrfHistRange = { fRange };

		Mat matHist;
		calcHist(&matPitch, 1, 0, Mat(), matHist, 1, &nHistSize, &ptrfHistRange);

		int nHistWidth = 500; int nHistHeight = 500;
		int nBinWidth = cvRound((double)nHistWidth / nHistSize);


		int fMax = 0;
		int nMaxBin = 0;
		for (int i = 0; i < nHistSize; i++){
			if (fMax < matHist.at<float>(i)){
				fMax = matHist.at<float>(i);
				nMaxBin = i;
			}
			//cout << i << ":" << matHist.at<float>(i) << endl;
		}


		Mat imgHist(nHistHeight, nHistWidth, CV_8UC3, Scalar::all(25));
		for (int i = 0; i <= imgHist.cols; i += imgHist.cols / 4){
			line(imgHist, Point(i, 0), Point(i, imgHist.rows), Scalar(0, 125, 125), 1);
		}

		normalize(matHist, matHist, 0, imgHist.rows, NORM_MINMAX, -1, Mat());

		//drawing
		//cout << -5.0 + (float)nMaxBin / 10 + 0.1 << "deg" << endl;
		//cout << "max value : " << fMax << endl;

		for (int i = 1; i < nHistSize; i++){
			line(imgHist, Point(nBinWidth*(i), nHistHeight - cvRound(matHist.at<float>(i))), Point(nBinWidth*(i), imgHist.rows), Scalar(200, 250, 250), 2, 8);
			//line(imgHist, Point(nBinWidth*(i - 1), nHistHeight - cvRound(matHist.at<float>(i - 1))),Point(nBinWidth*(i), nHistHeight - cvRound(matHist.at<float>(i))),Scalar::all(128), 1, 8);
		}
		namedWindow("hist", 0);
		imshow("hist", imgHist);


		return 0;
	}
};