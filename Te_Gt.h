#pragma once

#include <vector>
#include "Configure.h"
#include "pcl_function.h"

typedef std::vector<cv::Point > TePolygon; //轮廓
typedef std::vector<TePolygon> TeContour; //内外轮廓集
typedef std::vector<cv::Vec4i> TeContourContact; //轮廓关系集

class Te_Gt //一张图片的所有内外轮廓
{
public:
	explicit Te_Gt(QString tag);

	TeContour& GetContourVec();
	TeContourContact& GetHierarchy();
	QString GetLabel();
private:
	QString label;
	TeContour contour_vec;
	TeContourContact hierarchy;

};

typedef std::vector<Te_Gt*> SampleLabel; //多张图片的内外轮廓集