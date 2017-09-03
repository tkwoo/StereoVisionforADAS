/**
@file DefStruct.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com) Inha Univ.
@brief Struct file
*/

#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>
#define PI 3.141592

#ifndef WIN32
#define sprintf_s snprintf
#endif

using namespace cv;

enum { Daimler, KITTI, HICAM, CityScape };
//enum { Ped, Car, TrafficSign, TrafficLight, Else};
enum { TP, FN, FP, TP_S, FN_S};
enum { STEREO_BM = 0 }; //, STEREO_SGBM = 1
enum { GRAY, COLOR };

struct stixel_t
{
	int nGround;		///< Ground bound point
	int nHeight;		///< Upper bound point
	uchar chDisparity;	///< Disparity(normalized 255)
	int nCol;			///< column
	double dZ;			///< distance(meter)
	double dY;			///< meter
	double dX;			///< meter
	stixel_t(){
		nGround = -1;
		nHeight = -1;
		chDisparity = 0;
		dZ = 0.;
		dY = 0.;
		dX = 0.;
	}
};

struct Object_t
{
	Rect rectBB;
	double dZ;
	std::vector<stixel_t> vecobjStixels;
	int nClass;
	double dClassScore;
	double dCollisionRisk;
	Object_t(){
		rectBB = Rect(0, 0, 0, 0);
		dZ = 0.;
		vecobjStixels.clear();
		nClass = TP;
		dClassScore = 0.;
		dCollisionRisk = 1.;
	}
	Object_t(Rect rect, double dz){
		nClass = TP;
		dClassScore = 0.;
		dCollisionRisk = 1.;
		rectBB = rect; dZ = dz;
		vecobjStixels.clear();
	}
	Object_t(Rect rect, stixel_t objStixel){
		nClass = TP;
		dClassScore = 0.;
		dCollisionRisk = 1.;
		rectBB = rect; dZ = objStixel.dZ;
		vecobjStixels.push_back(objStixel);
	}
};

struct CameraParam_t
{
	double m_dPitchDeg;	///< unit : degree
	double m_dYawDeg; ///< unit : degree
	double m_dFocalLength; ///< unit : pixels
	double m_dOx; ///< unit : pixels
	double m_dOy; ///< unit : pixels
	double m_dCameraHeight; ///< cam height from ground
	double m_dFOVvDeg; ///< vertical FOV
	double m_dFOVhDeg; ///< horizontal FOV
	int m_nVanishingY; ///< vanishing line location
	cv::Size m_sizeSrc;
	CameraParam_t(){
		m_dPitchDeg = 0.;
		m_dYawDeg = 0.;
		m_dFocalLength = 0.;
		m_dOx = 0.;
		m_dOy = 0.;
		m_dCameraHeight = 0.;
		m_dFOVvDeg = 0.;
		m_dFOVhDeg = 0.;
		m_sizeSrc = cv::Size(0, 0);
	}
};
struct StereoCamParam_t
{
	int m_nNumberOfDisp; ///< number of disparity.
	int m_nWindowSize; ///< window size. It must be odd number.
	double m_dBaseLine; ///< baseline, unit : meters1
	double m_dMaxDist; ///< Maximum distance value, unit : meters
	CameraParam_t objCamParam;
	StereoCamParam_t(){
		m_nNumberOfDisp = 80;
		m_nWindowSize = 9;
		m_dBaseLine = 0.;
		m_dMaxDist = 50.0;
	}
};

