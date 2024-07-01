#ifndef _DEPTH2RGB_H_
#define _DEPTH2RGB_H_

#include <opencv2/opencv.hpp>
#include "teImage.h"

struct TeBGR
{
	uchar b;
	uchar g;
	uchar r;
};

class TeJetColorCode
{
public:

	TeBGR  m_pJetTab1024[1024];

	TeJetColorCode();

	bool cvt16Bit2BGR(cv::Mat& obj16Bit, cv::Mat& objBGR);
	bool cvt16Bit2BGR(float minHeight, float maxHeight,cv::Mat& obj16Bit, cv::Mat& objBGR);

	bool cvt32F2BGR(float minHeight, float maxHeight, cv::Mat& obj32FC1, cv::Mat& objBGR);
	bool cvt32F2BGR(cv::Mat& obj32FC1, cv::Mat& objBGR);

	te::Image dealWithCvt(cv::Mat& image, const double& InvalThreshold, const double& ValThreshold);
private:
	void teBuildJetTab();

private:
	TeBGR  m_pJetTab1280[1280];

};

#endif