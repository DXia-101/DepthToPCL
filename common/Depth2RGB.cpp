#include <stdio.h>
#include "Depth2RGB.h"
#include "pcl_function.h"
#include "teDataStorage.h"
#include "te2DCanvasController.h"

TeJetColorCode::TeJetColorCode()
{
	teBuildJetTab();

	memset(m_pJetTab1024, 0, sizeof(m_pJetTab1024));

	memcpy(m_pJetTab1024, m_pJetTab1280 + 128, 1024 * sizeof(TeBGR));

}

bool TeJetColorCode::cvt16Bit2BGR(cv::Mat& obj16Bit, cv::Mat& objBGR)
{
	if (obj16Bit.type() != CV_16UC1 || objBGR.type() != CV_8UC3)
	{
		return false;
	}
	int iWidth = obj16Bit.cols;
	int iHeight = obj16Bit.rows;

	for (size_t h = 0; h < iHeight; h++)
	{
		unsigned short* pDepth = obj16Bit.ptr<unsigned short>(h);
		TeBGR* pBGR = objBGR.ptr<TeBGR>(h);

		for (size_t w = 0; w < iWidth; w++)
		{
			int iIndex = 1023 * pDepth[w] / 65535.0;//将[0-65535]之间的数据映射到[0-1024)之间

			pBGR[w] = m_pJetTab1024[iIndex];
		}
	}
	return true;
}

bool TeJetColorCode::cvt16Bit2BGR(float minHeight, float maxHeight, cv::Mat& obj16Bit, cv::Mat& objBGR)
{

	if (obj16Bit.type() != CV_16UC1 || objBGR.type() != CV_8UC3)
	{
		return false;
	}
	int iWidth = obj16Bit.cols;
	int iHeight = obj16Bit.rows;

	for (size_t h = 0; h < iHeight; h++)
	{
		unsigned short* pDepth = obj16Bit.ptr<unsigned short>(h);
		TeBGR* pBGR = objBGR.ptr<TeBGR>(h);

		for (size_t w = 0; w < iWidth; w++)
		{
			float absDepth = pDepth[w] > minHeight ? pDepth[w] : minHeight;
			absDepth = absDepth < maxHeight ? absDepth : maxHeight;
			float realDepth = maxHeight == minHeight ? 0 : ((absDepth - minHeight) / (maxHeight - minHeight));
			int iIndex = 1023 * realDepth;//将[0-1.0]之间的数据映射到[0-1024)之间
			pBGR[w] = m_pJetTab1024[iIndex];
		}
	}
	return true;
}

bool TeJetColorCode::cvt32F2BGR(float minHeight, float maxHeight,cv::Mat& obj32FC1, cv::Mat& objBGR)
{
	if (obj32FC1.type() != CV_32FC1 || objBGR.type() != CV_8UC3)
	{
		return false;
	}
	int iWidth = obj32FC1.cols;
	int iHeight = obj32FC1.rows;

	for (size_t h = 0; h < iHeight; h++)
	{
		float* pDepth = obj32FC1.ptr<float>(h);
		TeBGR* pBGR = objBGR.ptr<TeBGR>(h);

		for (size_t w = 0; w < iWidth; w++)
		{
			float absDepth = pDepth[w] > minHeight ? pDepth[w] : minHeight;
			absDepth = absDepth < maxHeight ? absDepth : maxHeight;
			float realDepth = maxHeight == minHeight ? 0 : ((absDepth - minHeight) / (maxHeight - minHeight));
			int iIndex = 1023 * realDepth;//将[0-1.0]之间的数据映射到[0-1024)之间
			pBGR[w] = m_pJetTab1024[iIndex];
		}
	}
	return true;
}

bool TeJetColorCode::cvt32F2BGR(cv::Mat& obj32FC1, cv::Mat& objBGR)
{
	if (obj32FC1.type() != CV_32FC1 || objBGR.type() != CV_8UC3)
	{
		return false;
	}
	int iWidth = obj32FC1.cols;
	int iHeight = obj32FC1.rows;

	for (size_t h = 0; h < iHeight; h++)
	{
		float* pDepth = obj32FC1.ptr<float>(h);
		TeBGR* pBGR = objBGR.ptr<TeBGR>(h);

		for (size_t w = 0; w < iWidth; w++)
		{
			float absDepth = pDepth[w] > 0 ? pDepth[w] : 0.0;
			int iIndex = 1023 * absDepth;//将[0-1.0]之间的数据映射到[0-1024)之间
			if (iIndex > 1023)
			{
				iIndex = 1023;
			}
			pBGR[w] = m_pJetTab1024[iIndex];
		}
	}
	return true;
}

void TeJetColorCode::dealWithCvt(cv::Mat& image,const int& index)
{
	cv::Mat median;
	median.create(image.size(), CV_8UC3);
	if (CV_MAT_DEPTH(image.type()) == CV_16U) {
		if (index >= 0) {
			if (!cvt16Bit2BGR(teDataStorage::getInstance()->getSelectInvalidPointThreshold(index), teDataStorage::getInstance()->getSelectValidPointThreshold(index), image, median)) {
				return;
			}
		}
		else {
			if (!cvt16Bit2BGR(teDataStorage::getInstance()->getCurrentInvalidPointThreshold(), teDataStorage::getInstance()->getCurrentValidPointThreshold(), image, median)) {
				return;
			}
		}

	}
	else if (CV_MAT_DEPTH(image.type() == CV_32F)) {
		if (index >= 0) {
			if (!cvt32F2BGR(teDataStorage::getInstance()->getSelectInvalidPointThreshold(index), teDataStorage::getInstance()->getSelectValidPointThreshold(index), image, median)) {
				return;
			}
		}
		else {
			if (!cvt32F2BGR(teDataStorage::getInstance()->getCurrentInvalidPointThreshold(), teDataStorage::getInstance()->getCurrentValidPointThreshold(), image, median)) {
				return;
			}
		}
	}
	//cv::cvtColor(median, median, cv::COLOR_BGR2RGB);
	te2DCanvasController::getInstance()->setImage(te::Image(median).clone());
	cv::waitKey(0);
}

void TeJetColorCode::teBuildJetTab()
{
	memset(m_pJetTab1280, 0, sizeof(m_pJetTab1280));

	for (int i = 0; i < 256; i++)
	{
		m_pJetTab1280[i].b = i;
	}

	for (int i = 256; i < 512; i++)
	{
		m_pJetTab1280[i].b = 255;
		m_pJetTab1280[i].g = i - 256;
	}

	for (int i = 512; i < 768; i++)
	{
		m_pJetTab1280[i].b = 255 - (i - 512);
		m_pJetTab1280[i].g = 255;
		m_pJetTab1280[i].r = (i - 512);
	}

	for (int i = 768; i < 1024; i++)
	{
		m_pJetTab1280[i].g = 255 - (i - 768);
		m_pJetTab1280[i].r = 255;
	}

	for (int i = 1024; i < 1280; i++)
	{
		m_pJetTab1280[i].r = 255 - (i - 1024);
	}
}
