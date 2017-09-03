/**
@file StixelEstimation.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief stixel creation
*/
#pragma once 

#include <iostream>

#include "./include/DefStruct.h"

using namespace std;
using namespace cv;

typedef enum stixel_error { OK, SIZE_ERR, GND_ERR, VDISP_ERR, GNDRANSAC_ERR } STIXEL_ERROR;

/**
	@class CStixelEstimation
	@brief stixel ������ ���� class
*/
class CStixelEstimation{
private:
	//----------------input-------------------
	Mat m_matDisp16;
	Mat m_imgDisp8;

	//----------------param-------------------
	StereoCamParam_t m_objStereoParam;
	int m_nStixelWidth;

	double m_dGroundVdispSlope; ///< The ground line slope in v-disparity
	double m_dGroundVdispOrig; ///< The ground line origin in v-disparity
	int m_nVdispGroundThreshold; ///< Ground plane threshold value in Vdisparity map
	int m_nStixelGroundMargin;	///< Marginal compensation value for ground plane

	vector<Point2f> m_vecLinePoint; ///< v-disparity point : ground point
	Vec4f m_vec4fLine; ///< ground line

	//-------------member image---------------
	Mat m_imgVdisp;
	Mat m_imgTopView;

	//---------------function------------------

public:
	//----------------output-------------------
	vector<stixel_t> m_vecobjStixels; ///< stixel output
	vector<stixel_t> m_vecobjStixelInROI; ///< stixel output in 3D ROI
	Mat m_imgGround; ///< ground

	//---------------function------------------
	CStixelEstimation(StereoCamParam_t& objStereoParam);
	STIXEL_ERROR SetDispImage(Mat& matDisp16);
	STIXEL_ERROR SetDispImage(Mat& matDisp16, Mat& imgDisp8);
	STIXEL_ERROR SetDisp8Image(Mat& imgDisp8);

	//============processing part==============
	//wrapping function
	STIXEL_ERROR EstimateStixels(Mat& matDisp16);
	STIXEL_ERROR EstimateStixels(Mat& matDisp16, Mat& imgDisp8, bool flgUseMultiLayer=true);
	STIXEL_ERROR EstimateStixels_only8bitDisp(Mat& imgDisp8, bool flgUseMultiLayer=true);

	//Ground Estimation
	STIXEL_ERROR GroundEstimation(Mat& imgDisp8);

	STIXEL_ERROR ComputeVDisparity(Mat& imgDisp8);
	STIXEL_ERROR RmVDisparityNoise(Mat& imgVdisp);
	STIXEL_ERROR ExtractGroundPoint(Mat& imgVdisp, vector<Point2f>& vecLinePoint);
	STIXEL_ERROR FitLineRansac(vector<Point2f>& vecLinePoint, Vec4f& vec4fLine);
	STIXEL_ERROR RmGround(Vec4f vec4fLine, Mat& imgDisp8);

	//Sky Estimation
	STIXEL_ERROR RmSky(Mat& imgDisp8);

	//Stixel distance estimation
	STIXEL_ERROR StixelDistanceEstimation(Mat& imgDisp8, vector<stixel_t>& vecStixels, bool flgUseMultiLayer=true);
	STIXEL_ERROR StixelDisparityEstimation_col_SL(Mat& imgDisp8, int col, stixel_t& objStixel); ///< column single layer
	STIXEL_ERROR StixelDisparityEstimation_col_ML(Mat& imgDisp8, int col, vector<stixel_t>& vecStixels); ///< column multi layer

	STIXEL_ERROR StixelDisparityToDistance(vector<stixel_t>& vecStixels);

	//Stixel Optimization
	//STIXEL_ERROR StixelROIConstraint(vector<stixel_t>& vecStixelsInput, vector<stixel_t>& vecStixelsOutput);
	STIXEL_ERROR StixelROIConstraint_Lane(vector<stixel_t>& vecStixelsInput, vector<stixel_t>& vecStixelsOutput, float fLaneInterval, int nCenterPointX);

};