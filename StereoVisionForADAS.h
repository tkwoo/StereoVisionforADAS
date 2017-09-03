/**
@file StereoVisionForADAS.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief matching, stixel creation, objectness wrapper
*/
#pragma once 

#include "opencv2/opencv.hpp"
#include <iostream>

#include "StereoMatching.h"
#include "StixelEstimation.h"
#include "StixelSegmenation.h"
#include "./include/DefStruct.h"

using namespace std;
using namespace cv;

/**
	@class CStereoVisionForADAS
	@brief stixel wrapping class
*/
class CStereoVisionForADAS{
private:
	//----------------input--------------------
	Mat m_imgLeftInput;		///< rectified image
	Mat m_imgRightInput;	///< rectified image

	//----------------param--------------------
	StereoCamParam_t m_objStereoParam;
	CStereoMatching m_objStereoMatching;
	CStixelEstimation m_objStixelEstimation;
	CStixelSegmentation m_objStixelSegmentation;

	MATCHING_ERROR match_err;
	STIXEL_ERROR stixel_err;
	SEG_ERROR seg_err;

	//LUT
	unsigned char m_pseudoColorLUT[256][3]; ///< RGB pseudo color

	//-------------member image----------------
	Mat m_imgColorDisp; ///< 8bit 3ch disparity image
	Mat m_imgStixelGray;
	Mat m_imgTopView;

	//---------------function------------------
	//Pseudo color Disparity
	void MakePseudoColorLUT(); ///< pseudo color LUT
	void cvtPseudoColorImage(Mat& srcGray, Mat& dstColor); ///< input : gray image, output : color image
	
	void TopViewStixel(vector<stixel_t>& objStixelInROI);
	void TopViewLane(Mat& imgTopView, float fLaneInterval, int nCenterPointX);

public:
	//----------------output-------------------
	/// Matching output
	Mat m_matDisp16;
	Mat m_imgDisp8;

	/// Stixel output
	vector<stixel_t> m_vecobjStixels; ///< stixel output
	vector<stixel_t> m_vecobjStixelInROI; ///< stixel output in 3D ROI
	Mat m_imgGround;

	/// segmentation output
	vector<Object_t> m_vecobjBB;

	//---------------function------------------
	CStereoVisionForADAS(StereoCamParam_t& objStereoParam);
	int Objectness(Mat& imgLeft, Mat& imgRight);
	int Objectness(Mat& imgLeft, Mat& imgRight, Mat& imgDisp8);

	// depth
	int RectToDisp(Rect& rectBox, Mat& matRect);
	int Disp16ToDepth(const uchar fDisparity, float& fDistanceMeter);

	/// display
	void Display();
	void Display(Mat& imgDisplay);
	void Display(Mat& imgDisplay, Mat& imgStixelResult);

	// display, draw
	void DrawLane(Mat& imgResult, StereoCamParam_t& objStereoParam);
	void DrawStixel(Mat& imgResult, vector<stixel_t>& vecobjStixels);
	void DrawGround(Mat& imgResult, Mat& imgGround);

	static StereoCamParam_t InitStereoParam(int nDatasetName);
	static int PitchDegToVanishingLine(StereoCamParam_t& objStereoParam);
	
};
