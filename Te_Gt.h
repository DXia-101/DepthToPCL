#pragma once

#include <vector>
#include "Configure.h"
#include "pcl_function.h"

typedef std::vector<cv::Point > TePolygon; //����
typedef std::vector<TePolygon> TeContour; //����������
typedef std::vector<cv::Vec4i> TeContourContact; //������ϵ��

class Te_Gt //һ��ͼƬ��������������
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

typedef std::vector<Te_Gt*> SampleLabel; //����ͼƬ������������