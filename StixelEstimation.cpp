/**
@file StixelEstimation.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief stixel creation
*/
#include "StixelEstimation.h"
 

CStixelEstimation::CStixelEstimation(StereoCamParam_t& objStereroParam)
{
	m_objStereoParam = objStereroParam;
	
	m_nStixelWidth = 1;

	m_dGroundVdispSlope = 0;
	m_dGroundVdispOrig = 0;

	m_nVdispGroundThreshold = 50;
	m_nStixelGroundMargin = 15;//10;

	m_imgGround = Mat(m_objStereoParam.objCamParam.m_sizeSrc, CV_8U);
	m_imgGround = Scalar(0);
}
STIXEL_ERROR CStixelEstimation::SetDispImage(Mat& matDisp16)
{
	m_matDisp16 = matDisp16;
	matDisp16.convertTo(m_imgDisp8, CV_8U, 255 / (m_objStereoParam.m_nNumberOfDisp*16.));
	if (m_imgDisp8.size() != m_objStereoParam.objCamParam.m_sizeSrc) return SIZE_ERR;
	return OK;
}
STIXEL_ERROR CStixelEstimation::SetDispImage(Mat& matDisp16, Mat& imgDisp8)
{
	m_matDisp16 = matDisp16;
	m_imgDisp8 = imgDisp8;
	if (m_imgDisp8.size() != m_objStereoParam.objCamParam.m_sizeSrc) return SIZE_ERR;
	return OK;
}
STIXEL_ERROR CStixelEstimation::SetDisp8Image(Mat& imgDisp8)
{
	m_imgDisp8 = imgDisp8;
	if (m_imgDisp8.size() != m_objStereoParam.objCamParam.m_sizeSrc) return SIZE_ERR;
	return OK;
}
STIXEL_ERROR CStixelEstimation::EstimateStixels(Mat& matDisp16)
{
	Size size(5, 5);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);

	SetDispImage(matDisp16);

	GroundEstimation(m_imgDisp8);
	RmSky(m_imgDisp8);
	morphologyEx(m_imgDisp8, m_imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 1);

	//imshow("rmgndtemp", m_imgDisp8);

	StixelDistanceEstimation(m_imgDisp8, m_vecobjStixels);

	StixelROIConstraint_Lane(m_vecobjStixels, m_vecobjStixelInROI, 3., m_imgDisp8.cols / 2);	

	return OK;
}
STIXEL_ERROR CStixelEstimation::EstimateStixels(Mat& matDisp16, Mat& imgDisp8, bool flgUseMultiLayer)
{
	Size size(5, 5);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);

	SetDispImage(matDisp16, imgDisp8);

	GroundEstimation(m_imgDisp8);
	RmSky(m_imgDisp8);
	morphologyEx(m_imgDisp8, m_imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 1);

	//imshow("rmgndtemp", m_imgDisp8);

	StixelDistanceEstimation(m_imgDisp8, m_vecobjStixels, flgUseMultiLayer);

	StixelROIConstraint_Lane(m_vecobjStixels, m_vecobjStixelInROI, 3., m_imgDisp8.cols / 2);

	return OK;
}
STIXEL_ERROR CStixelEstimation::EstimateStixels_only8bitDisp(Mat& imgDisp8, bool flgUseMultiLayer)
{
	Size size(5, 5);
	Mat matKernel = getStructuringElement(MORPH_RECT, size);

	SetDisp8Image(imgDisp8);

	GroundEstimation(m_imgDisp8);
	RmSky(m_imgDisp8);
	morphologyEx(m_imgDisp8, m_imgDisp8, MORPH_OPEN, matKernel, Point(-1, -1), 1);

	//imshow("rmgndtemp", m_imgDisp8);

	StixelDistanceEstimation(m_imgDisp8, m_vecobjStixels, flgUseMultiLayer);

	StixelROIConstraint_Lane(m_vecobjStixels, m_vecobjStixelInROI, 3., m_imgDisp8.cols / 2);

	return OK;
}

STIXEL_ERROR CStixelEstimation::GroundEstimation(Mat& imgDisp8)
{
	ComputeVDisparity(imgDisp8);

	//imshow("imgDisp8", imgDisp8);
	//imshow("imgVDISP", m_imgVdisp);
	//waitKey();
	RmVDisparityNoise(m_imgVdisp);
	ExtractGroundPoint(m_imgVdisp, m_vecLinePoint);
	FitLineRansac(m_vecLinePoint, m_vec4fLine);
	RmGround(m_vec4fLine, imgDisp8);
	//line(m_imgVdisp, Point(0, m_dGroundVdispOrig), Point(255, 255 * m_dGroundVdispSlope + m_dGroundVdispOrig), Scalar(255), 4);
	//imshow("vdisp", m_imgVdisp);
	//waitKey(1);
	if (m_dGroundVdispOrig <= 0 || m_dGroundVdispSlope > 2 || m_dGroundVdispSlope < 0.5) {
#ifdef _DEBUG
		printf("RANSAC : cannot find ground line. out of ranges\n"); 
#endif
		return GND_ERR;
	}
	return OK;
}
STIXEL_ERROR CStixelEstimation::ComputeVDisparity(Mat& imgDisp8)
{
	int maxDisp = 255;
	m_imgVdisp = Mat(imgDisp8.rows, 255, CV_8U, Scalar(0));
	for (int u = 0; u<imgDisp8.rows; u++){
		if (u < m_objStereoParam.objCamParam.m_nVanishingY) continue; // we are finding ground. therefore we check pixels below vanishing point 
		for (int v = 0; v<imgDisp8.cols; v++){
			int disp = (imgDisp8.at<uchar>(u, v));// / 8;
			//if(disp>0 && disp < maxDisp){
			if (disp>6 && disp < maxDisp - 2){ //We remove pixels of sky and car to compute the roadline
				m_imgVdisp.at<uchar>(u, disp) += 1;
			}
		}
	}
	return OK;
}
STIXEL_ERROR CStixelEstimation::RmVDisparityNoise(Mat& imgVdisp)
{
	int nThresh = m_nVdispGroundThreshold;	 // temp �̹��� ������ �� �ڵ����� ��������
	threshold(imgVdisp, imgVdisp, nThresh, 255, 3);
	return OK;
}
STIXEL_ERROR CStixelEstimation::ExtractGroundPoint(Mat& imgVdisp, vector<Point2f>& vecLinePoint)
{
	vecLinePoint.clear();
	for (int u = m_objStereoParam.objCamParam.m_nVanishingY; u<imgVdisp.rows; u++){//200 is the vanishing row in image : It will be fixed 150901
		for (int v = 0; v < imgVdisp.cols; v++){
			int value = imgVdisp.at<uchar>(u, v);
			if (value > 0){
				vecLinePoint.push_back(Point2f((float)u, (float)v));
			}
		}
	}
	return OK;
}
STIXEL_ERROR CStixelEstimation::FitLineRansac(vector<Point2f>& vecLinePoint, Vec4f& vec4fLine)
{
	int iterations = 100;
	double sigma = 1.;
	double a_max = 7.;

	int n = vecLinePoint.size();
	//cout <<"point size : "<< n << endl;
	if (n<2)
	{
		printf("Points must be more than 2 EA\n");
		return GNDRANSAC_ERR;
	}

	RNG rng;
	double bestScore = -1.;
	for (int k = 0; k<iterations; k++)
	{
		int i1 = 0, i2 = 0;
		double dx = 0;
		while (i1 == i2)
		{
			i1 = rng(n);
			i2 = rng(n);
		}
		Point2f p1 = vecLinePoint[i1];
		Point2f p2 = vecLinePoint[i2];

		Point2f dp = p2 - p1;
		dp *= 1. / norm(dp);
		double score = 0;

		if (fabs(dp.x / 1.e-5f) && fabs(dp.y / dp.x) <= a_max)
		{
			for (int i = 0; i<n; i++)
			{
				Point2f v = vecLinePoint[i] - p1;
				double d = v.y*dp.x - v.x*dp.y;
				score += exp(-0.5*d*d / (sigma*sigma));
			}
		}
		if (score > bestScore)
		{
			vec4fLine = Vec4f(dp.x, dp.y, p1.x, p1.y);
			bestScore = score;
		}
	}
	return OK;
}
STIXEL_ERROR CStixelEstimation::RmGround(Vec4f vec4fLine, Mat& imgDisp8)
{
	double slope = vec4fLine[0] / vec4fLine[1];
	double orig = vec4fLine[2] - slope*vec4fLine[3];
	if (orig < 0 || slope > 2 || slope < 0.5) {
#ifdef _DEBUG
		printf("ground line error\n");
#endif
	}
	else{
		m_dGroundVdispOrig = orig;
		m_dGroundVdispSlope = slope;
	}
	
	m_dGroundVdispOrig += 5;

	//printf("v=%lf * d + %lf\n", m_dGroundVdispSlope, m_dGroundVdispOrig); // print line eq.
	double dPit = atan((m_dGroundVdispOrig - imgDisp8.rows / 2) / m_objStereoParam.objCamParam.m_dFocalLength) * 180 / PI;
	//printf("%.1lfdeg, ", dPit);
	//cout << atan((m_dGroundVdispOrig - imgDisp8.rows / 2) / m_objStereoParam.objCamParam.m_dFocalLength) * 180 / PI << "deg, ";
	//slope = -0.7531;
	//orig = 200.;
	for (int u = (int)m_dGroundVdispOrig; u<imgDisp8.rows; u++){//200 is the vanishing row in image : It will be fixed 150901
		for (int v = 0; v<imgDisp8.cols; v++){
			int value = imgDisp8.at<uchar>(u, v);
			double test = m_dGroundVdispOrig + m_dGroundVdispSlope*value - u;
			if (test > m_nStixelGroundMargin){
				imgDisp8.at<uchar>(u, v) = value;
				//res.at<unsigned char>(u, v) = value;
			}
			else{
				imgDisp8.at<uchar>(u, v) = 0;
				m_imgGround.at<uchar>(u, v) = value;
				//res.at<unsigned char>(u, v) = 0;
			}
		}
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::RmSky(Mat& imgDisp8)
{
	/*double slope = -0.5016;
	double orig = 191.696210;*/
	double orig = m_dGroundVdispOrig - 3;
	double slope = -0.7; // const - 0.5

	for (int u = 0; u<imgDisp8.rows; u++){
		for (int v = 0; v<imgDisp8.cols; v++){
			int value = imgDisp8.at<uchar>(u, v);
			//double test = orig + slope*value - u;
			if (u < (orig + slope*value)){
				//img.at<unsigned char>(u, v) = value;
				imgDisp8.at<uchar>(u, v) = 0;
				//res.at<unsigned char>(u, v) = value;
			}
			//else{
			//	img.at<unsigned char>(u, v) = 0;
			//	//res.at<unsigned char>(u, v) = 0;
			//}
		}
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::StixelDistanceEstimation(Mat& imgDisp8, vector<stixel_t>& vecStixels, bool flgUseMultiLayer)
{
	STIXEL_ERROR Err;
	vecStixels.clear();
	for (int u = 0; u < imgDisp8.cols; u++){
		stixel_t objStixelTemp;
		if (u < 30) { //Removal manually left 30 cols because of paralax
			//m_ptobjStixels[u].chDisparity = 0;
			//m_ptobjStixels[u].nGround = 0;
			//m_ptobjStixels[u].nHeight = 0;

			objStixelTemp.chDisparity = 0;
			objStixelTemp.nGround = 0;
			objStixelTemp.nHeight = 0;
			objStixelTemp.nCol = u;

		}
		//else StixelDistanceEstimation_col(u, m_ptobjStixels[u]);
		else if (flgUseMultiLayer == true)
			Err = StixelDisparityEstimation_col_ML(imgDisp8, u, vecStixels);
		else {
			Err = StixelDisparityEstimation_col_SL(imgDisp8, u, objStixelTemp);
			if (Err == OK) m_vecobjStixels.push_back(objStixelTemp);
		}
	}	
	StixelDisparityToDistance(vecStixels);
	return OK;
}
STIXEL_ERROR CStixelEstimation::StixelDisparityEstimation_col_SL(Mat& imgDisp8, int col, stixel_t& objStixel)
{
	int nIter = imgDisp8.rows / 2;
	uchar chDisp;

	for (int v = 1; v < nIter; v++){
		chDisp = imgDisp8.at<uchar>(imgDisp8.rows - v, col);

		if (imgDisp8.at<uchar>(v, col)>0 && objStixel.nHeight == -1){
			objStixel.nHeight = v; nIter = imgDisp8.rows - v;
		}
		if (chDisp > 0 && objStixel.nGround == -1){
			objStixel.nGround = imgDisp8.rows - v + m_nStixelGroundMargin; //number is manually
			objStixel.chDisparity = chDisp; // 
			objStixel.nCol = col;
			nIter = imgDisp8.rows - v;
		}
	}
	//cout << col << " : " << objStixel.nGround << ", " << objStixel.nHeight << endl;
	return OK;
}
STIXEL_ERROR CStixelEstimation::StixelDisparityEstimation_col_ML(Mat& imgDisp8, int col, vector<stixel_t>& vecStixels)
{
	int nSkybound = 0;
	int nMargin = 10;
	uchar chDisp;
	stixel_t objStixelTemp;
	//cout << "col : "<<col <<" ";
	//bool flgObject = false;

	for (int v = 0; v < imgDisp8.rows / 2; v++){
		if (imgDisp8.at<uchar>(v, col)>1){ nSkybound = v; break; }
	}
	//cout << "skybound:" << nSkybound << endl;
	for (int v = imgDisp8.rows - 1; v >= nSkybound; v--){
		//chDisp = m_imgGrayDisp8.at<uchar>(m_imgGrayDisp8.rows - v, col);	//from down
		chDisp = imgDisp8.at<uchar>(v, col);
		if (chDisp > 0 && objStixelTemp.nGround == -1 && v > m_objStereoParam.objCamParam.m_nVanishingY - 10){
			objStixelTemp.nGround = v + 10; //10 is manually selected
			objStixelTemp.chDisparity = imgDisp8.at<uchar>(v, col); //chDisp; // manual temp number 160303
			objStixelTemp.nCol = col;
			//cout << "ground:"<< v << " ";
		}
		else if (chDisp <= 1 && objStixelTemp.nGround != -1){
			objStixelTemp.nHeight = v;
			vecStixels.push_back(objStixelTemp);
			//cout << "push" << endl;
			objStixelTemp.chDisparity = 0; objStixelTemp.nGround = -1; objStixelTemp.nHeight = -1;
			//cout << "Height:" << v << endl;
		}
		else if (v == nSkybound + 1 && objStixelTemp.nGround != -1)
		{
			objStixelTemp.nHeight = v;
			vecStixels.push_back(objStixelTemp);
			objStixelTemp.chDisparity = 0; objStixelTemp.nGround = -1; objStixelTemp.nHeight = -1;
			//cout << "Height:" << v << endl;
		}
	}
	return OK;
}
STIXEL_ERROR CStixelEstimation::StixelDisparityToDistance(vector<stixel_t>& vecStixels)
{
	for (unsigned int u = 0; u < vecStixels.size(); u++){
		if (vecStixels[u].chDisparity == 0) vecStixels[u].dZ = 0;
		vecStixels[u].dZ = (m_objStereoParam.objCamParam.m_dFocalLength*m_objStereoParam.m_dBaseLine / ((double)vecStixels[u].chDisparity*(double)m_objStereoParam.m_nNumberOfDisp / 255));
		vecStixels[u].dX = vecStixels[u].dZ*(double)(vecStixels[u].nCol - m_objStereoParam.objCamParam.m_sizeSrc.width / 2) / m_objStereoParam.objCamParam.m_dFocalLength;
	}
	return OK;
}

STIXEL_ERROR CStixelEstimation::StixelROIConstraint_Lane(vector<stixel_t>& vecStixelsInput, 
	vector<stixel_t>& vecStixelsOutput, float fLaneInterval, int nCenterPointX)
{
	vecStixelsOutput.clear();
	//double dCenterPointXmeter;
	//float fLaneIntervalPixel = 0;
	/*Mat imgTemp = m_imgDisp8.clone();
	threshold(imgTemp, imgTemp, 255, 255, CV_THRESH_BINARY);*/

	for (unsigned int u = 0; u < vecStixelsInput.size(); u++){
		if (vecStixelsInput[u].chDisparity == 0) continue;
		//fLaneIntervalPixel = (float)m_nFocalLength*(float)fLaneInterval / ((float)(500 - i) / 10);
		if (vecStixelsInput[u].dX < (double)fLaneInterval / 2 - vecStixelsInput[u].dZ / m_objStereoParam.objCamParam.m_dFocalLength*(double)(m_imgDisp8.cols / 2 - nCenterPointX))
		{
			if (vecStixelsInput[u].dX > -(double)fLaneInterval / 2 + vecStixelsInput[u].dZ / m_objStereoParam.objCamParam.m_dFocalLength*(double)(m_imgDisp8.cols / 2 - nCenterPointX))
			{
				
				vecStixelsOutput.push_back(vecStixelsInput[u]);
				/*line(imgTemp,
					Point(vecStixelsInput[u].nCol, vecStixelsInput[u].nGround),
					Point(vecStixelsInput[u].nCol, vecStixelsInput[u].nHeight),
					Scalar(vecStixelsInput[u].chDisparity));
				imshow("stixel", imgTemp);
				waitKey(1);*/
			}
		}
	}
	return OK;
}