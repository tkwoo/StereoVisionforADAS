/**
@file StixelSegmentation.cpp
@date 2017/09/03
@author tkwoo(wtk1101@gmail.com)
@brief stixel segmentation(objectness)
*/

#include "StixelSegmenation.h"

CStixelSegmentation::CStixelSegmentation(StereoCamParam_t objStereoParam)
{
	m_objStereoParam = objStereoParam;
}
 
SEG_ERROR CStixelSegmentation::SegmentStixel(vector<stixel_t>& objStixels)
{
	if(objStixels.size() < 1) return SIZ_ERR;
	m_vecobjBB.clear();

	vector<Object_t> objBBcandidateZ, objBBcandidateX;
	objBBcandidateZ.clear();
	objBBcandidateX.clear();

	StixelZClustering(objStixels, objBBcandidateZ);
	//cout << objBBcandidateZ.size() << endl;
	/*for (int n = 0; n < objBBcandidateZ.size(); n++)
	{
		cout << n << " ��° ==============================" << endl;
		for (int i = 0; i < objBBcandidateZ[n].objStixels.size(); i++){
			cout << objBBcandidateZ[n].objStixels[i].nCol << ":"
				<< objBBcandidateZ[n].objStixels[i].nGround << ":"
				<< objBBcandidateZ[n].objStixels[i].nHeight << ":"
				<< objBBcandidateZ[n].objStixels[i].dZ << endl;		
		}
	}*/

	StixelXClustering(objBBcandidateZ, objBBcandidateX);
	//cout << objBBcandidateX.size() << endl;

	StixelBBboxOptimization(objBBcandidateX, m_vecobjBB);
	//cout << m_vecobjBB.size() << endl;

	
	return GOOD;
}
SEG_ERROR CStixelSegmentation::StixelZClustering(vector<stixel_t>& objStixels, vector<Object_t>& objBBcandidate)
{
	multimap<double, stixel_t> mapSortedStixel;		// distance

	for (unsigned int i = 0; i < objStixels.size(); i++)
	{
		mapSortedStixel.insert(pair<double, stixel_t>(objStixels[i].dZ, objStixels[i]));
	}

	multimap<double, stixel_t>::iterator start;
	multimap<double, stixel_t>::iterator end;
	start = mapSortedStixel.begin();
	end = mapSortedStixel.end();

	objBBcandidate.clear();
	Rect rectTemp(start->second.nCol, start->second.nHeight, 0, abs(start->second.nHeight - start->second.nGround));
	Object_t bbTemp(rectTemp, start->second);
	objBBcandidate.push_back(bbTemp);
	*start++;

	while (start != end)
	{
		bool flgDetected = false;
		unsigned int i = 0;
		for (i = 0; i < objBBcandidate.size(); i++)
		{
			/// ��ü�� ��踸�� �ν� (�Ǽ���, �̹��� ��ǥ)
			if (abs(objBBcandidate[i].dZ - start->first) < 2.)// && ((objBBcandidate[i].bb.x - start->second.nCol < 3) || (start->second.nCol - (objBBcandidate[i].bb.x+objBBcandidate[i].bb.width) < 3)))
			{
				//objBBcandidate[i].bb.width = objBBcandidate[i].bb.width > abs(objBBcandidate[i].bb.x - start->second.nCol) ? objBBcandidate[i].bb.width : abs(objBBcandidate[i].bb.x - start->second.nCol);
				if (objBBcandidate[i].rectBB.width < start->second.nCol - objBBcandidate[i].rectBB.x)
				{
					objBBcandidate[i].rectBB.width = abs(objBBcandidate[i].rectBB.x - start->second.nCol);
				}
				else if (objBBcandidate[i].rectBB.width < (objBBcandidate[i].rectBB.x + objBBcandidate[i].rectBB.width) - start->second.nCol)
				{
					objBBcandidate[i].rectBB.width = (objBBcandidate[i].rectBB.x + objBBcandidate[i].rectBB.width) - start->second.nCol;
				}
				objBBcandidate[i].rectBB.x = objBBcandidate[i].rectBB.x < start->second.nCol ? objBBcandidate[i].rectBB.x : start->second.nCol;
				objBBcandidate[i].rectBB.y = objBBcandidate[i].rectBB.y < start->second.nHeight ? objBBcandidate[i].rectBB.y : start->second.nHeight;
				objBBcandidate[i].rectBB.height = objBBcandidate[i].rectBB.height >(start->second.nGround - start->second.nHeight) ? objBBcandidate[i].rectBB.height : (start->second.nGround - start->second.nHeight);
				objBBcandidate[i].vecobjStixels.push_back(start->second);

				flgDetected = true;
				break;
			}
		}
		if (flgDetected == false)//i == objBBcandidate.size())
		{
			//objBBcandidate.clear();
			Rect rectTemp(start->second.nCol, start->second.nHeight, 0, abs(start->second.nHeight - start->second.nGround));
			Object_t bbTemp(rectTemp, start->second);
			//bbTemp.objStixels.push_back(start->second);
			objBBcandidate.push_back(bbTemp);

			/*cout << bbTemp.objStixels[0].nCol << ":"
				<< bbTemp.objStixels[0].nGround << ":"
				<< bbTemp.objStixels[0].nHeight << ":"
				<< bbTemp.objStixels[0].dZ << endl;*/
		}

		//cout << start->first << ":" << start->second.nCol << endl;
		/*for (int i = 0; i < objBBcandidate.size(); i++){
		cout << objBBcandidate[i].dZ << ":" << objBBcandidate[i].bb.x << ":" << objBBcandidate[i].bb.width << ":" << objBBcandidate[i].bb.height << endl;
		}*/
		*start++;
	}
	return GOOD;
}
void CStixelSegmentation::SetDebugImg(Mat imgTemp)
{
	if(imgTemp.channels() == 1) cvtColor(imgTemp, m_imgDebug, CV_GRAY2BGR);
	else m_imgDebug = imgTemp;
}

SEG_ERROR CStixelSegmentation::StixelXClustering(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput)
{
	objBBOutput.clear();

	for (unsigned int i = 0; i < objBBinput.size(); i++)
	{
		//cout << i << " ��° ==============================" << endl;
		//debug
		/*rectangle(m_imgDebug, objBBinput[i].bb, Scalar(255,255,255), 2);
		imshow("debug", m_imgDebug);
		waitKey(1);
		*/
		multimap<int, stixel_t> mapSortedStixel;		// X-direction distance
		for (unsigned int j = 0; j < objBBinput[i].vecobjStixels.size(); j++)
		{
			/*cout << objBBinput[i].objStixels[j].nCol << ":"
				<< objBBinput[i].objStixels[j].nGround << ":"
				<< objBBinput[i].objStixels[j].nHeight << ":"
				<< objBBinput[i].objStixels[j].dZ << endl;*/
			mapSortedStixel.insert(pair<int, stixel_t>(objBBinput[i].vecobjStixels[j].nCol, objBBinput[i].vecobjStixels[j]));
		}

		multimap<int, stixel_t>::iterator start;
		multimap<int, stixel_t>::iterator end;
		multimap<int, stixel_t>::iterator iter;
		start = mapSortedStixel.begin();
		iter = mapSortedStixel.begin();
		end = mapSortedStixel.end();

		*start++;

		bool flgSeparator = false;

		multimap<int, stixel_t>::iterator iterLB = mapSortedStixel.begin(); // Left bound
		multimap<int, stixel_t>::iterator iterRB = mapSortedStixel.end(); // Right bound
		//cout << endl;
		while (start != end)
		{
			//cout << iter->first << " : " << iter->second.dX << endl;
			// X�� �������� ������ �Ÿ��� 0.3m �̻��̸� �� �� �з��� bb��� �Ǵ�
			if (start->second.dX - iter->second.dX > 0.3){
				flgSeparator = true;

				iterRB = iter;
				//cout << mapSortedStixel.begin()->second.nCol << endl;
				//cout << iter->second.nCol << endl;
				Rect rectTemp(iterLB->second.nCol, iterLB->second.nHeight, iterRB->second.nCol - iterLB->second.nCol, 0);
				Object_t bbTemp(rectTemp, objBBinput[i].vecobjStixels[0]);

				int nBBminRow = m_objStereoParam.objCamParam.m_sizeSrc.height;
				int nBBmaxRow = 0;
				while (iterLB != iterRB)
				{
					bbTemp.vecobjStixels.push_back(iterLB->second);
					if (iterLB->second.nGround > nBBmaxRow)
						nBBmaxRow = iterLB->second.nGround;
					if (iterLB->second.nHeight < nBBminRow)
						nBBminRow = iterLB->second.nHeight;
					*iterLB++;
				}
				rectTemp.height = nBBmaxRow - nBBminRow;
				rectTemp.y = nBBminRow;
				
				bbTemp.rectBB = rectTemp;
				
				objBBOutput.push_back(bbTemp);
				
				//debug
				/*rectangle(m_imgDebug, rectTemp, Scalar(0,0,255), 3);
				imshow("debug", m_imgDebug);
				waitKey(1);*/

				iterLB = start;
			}
			*start++; *iter++;
		}
		if (flgSeparator == false) objBBOutput.push_back(objBBinput[i]);
		if (flgSeparator == true){

			/*cout << iterLB->second.nCol << endl;
			cout << objBBinput[i].bb.x + objBBinput[i].bb.width << endl;*/
			Rect rectTemp(iterLB->second.nCol, iterLB->second.nHeight, objBBinput[i].rectBB.x + objBBinput[i].rectBB.width - iterLB->second.nCol, 0);
			Object_t bbTemp(rectTemp, objBBinput[i].vecobjStixels[0]);
			int nBBminRow = m_objStereoParam.objCamParam.m_sizeSrc.height;
			int nBBmaxRow = 0;
			while (iterLB != end)
			{
				bbTemp.vecobjStixels.push_back(iterLB->second);
				if (iterLB->second.nGround > nBBmaxRow)
					nBBmaxRow = iterLB->second.nGround;
				if (iterLB->second.nHeight < nBBminRow)
					nBBminRow = iterLB->second.nHeight;
				*iterLB++;
			}
			rectTemp.height = nBBmaxRow - nBBminRow;
			rectTemp.y = nBBminRow;

			/*cout << rectTemp << endl;
			cout << "last!" << endl;*/
			bbTemp.rectBB = rectTemp;
			
			objBBOutput.push_back(bbTemp);
			
			/*rectangle(m_imgDebug, rectTemp, Scalar(0, 0, 255), 3);
			imshow("debug", m_imgDebug);
			waitKey(1);*/
			//rectangle(m_imgColorDisp8, rectTemp, Scalar(255, 0, 0), 5);
		}
	}
	return GOOD;
}
SEG_ERROR CStixelSegmentation::StixelBBboxOptimization(vector<Object_t>& objBBinput, vector<Object_t>& objBBOutput)
{
	objBBOutput.clear();

	for (unsigned int i = 0; i < objBBinput.size(); i++){
		if (objBBinput[i].rectBB.width*objBBinput[i].dZ > m_objStereoParam.objCamParam.m_dFocalLength*0.25		// �ʺ� 0.25m �̻�
			&& objBBinput[i].rectBB.height*objBBinput[i].dZ > m_objStereoParam.objCamParam.m_dFocalLength*0.4	// ���� 0.4m �̻�
			&& objBBinput[i].dZ < m_objStereoParam.m_dMaxDist
			&& objBBinput[i].rectBB.width > 10
			&& objBBinput[i].rectBB.height > 10
			&& (float)objBBinput[i].rectBB.width / objBBinput[i].rectBB.height > 0.15
			){
			objBBOutput.push_back(objBBinput[i]);
		}
	}

	for (unsigned int i = 0; i < objBBOutput.size(); i++)
	{
		int nMaxRow = 0;
		double dMinDist = m_objStereoParam.m_dMaxDist;
		for (unsigned int j = 0; j < objBBOutput[i].vecobjStixels.size(); j++)
		{
			nMaxRow = (nMaxRow < objBBOutput[i].vecobjStixels[j].nGround) ? objBBOutput[i].vecobjStixels[j].nGround : nMaxRow;
			dMinDist = dMinDist < objBBOutput[i].vecobjStixels[j].dZ ? dMinDist : objBBOutput[i].vecobjStixels[j].dZ;
		}
		objBBOutput[i].rectBB.height = nMaxRow - objBBOutput[i].rectBB.y;
		objBBOutput[i].dZ = dMinDist;
	}

	return GOOD;
}