/**
@file StixelSegmentation.h
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief stixel segmentation(objectness)
*/
#pragma once

// #include <opencv2\opencv.hpp>
#include <iostream>
#include <map> 

#include "./include/DefStruct.h"
#include "StixelEstimation.h"

using namespace std;
using namespace cv;

typedef enum Seg_error { GOOD, SIZ_ERR } SEG_ERROR;

/**
	@class CStixelSegmentation
	@brief stixel Segmentation�� ���� class
*/
class CStixelSegmentation{
private:
	//----------------input--------------------
	vector<stixel_t> m_vecobjStixels;

	//----------------param--------------------
	StereoCamParam_t m_objStereoParam;

	//-------------member image----------------
	Mat m_imgDebug;
	
	//---------------function------------------
	//============processing part==============

	//old version
	//SEG_ERROR StixelZClustering(vector<stixel_t>& objStixels, vector<boundingbox_t>& objBBcandidate); ///< Z ���� Distance ��� �з�, Stixel -> BB candidate
	//SEG_ERROR StixelXClustering(vector<boundingbox_t>& objBBinput, vector<boundingbox_t>& objBBOutput); ///< X ���� Distance ��� �з�, BB candidate -> BB result
	//SEG_ERROR StixelBBboxOptimization(vector<boundingbox_t>& objBBinput, vector<boundingbox_t>& objBBOutput); ///< bounding box ����ȭ

	SEG_ERROR StixelZClustering(vector<stixel_t>& objStixels, vector<Object_t>& objBBcandidate); ///< Z ���� Distance ��� �з�, Stixel -> BB candidate
	SEG_ERROR StixelXClustering(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput); ///< X ���� Distance ��� �з�, BB candidate -> BB result
	SEG_ERROR StixelBBboxOptimization(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput); ///< bounding box ����ȭ

public:
	//----------------output-------------------
	vector<Object_t> m_vecobjBB;

	//---------------function------------------
	CStixelSegmentation(StereoCamParam_t objStereoParam);
	void SetDebugImg(Mat imgTemp);

	//wrapping function
	SEG_ERROR SegmentStixel(vector<stixel_t>& objStixels); ///< Stixel �з��۾�
	
	
};
