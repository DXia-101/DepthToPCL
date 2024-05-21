#pragma once
#include <QObject>

struct AxisSet {
    int curwidth;
    int curheight;
    float OriginX;
    float OriginY;
    pcl::PointXYZ minPt;
    pcl::PointXYZ maxPt;
};

struct te3DCanvasMember {
    bool PositiveAndNegative_X_axis = true;
    bool PositiveAndNegative_Y_axis = true;
    bool PositiveAndNegative_Z_axis = true;

    bool isPickingMode = false; //�Ƿ������ģʽ
    bool flag = false;//�ж��ǲ��ǵ�һ�ε��
    unsigned int line_id = 0;

    bool isShowDimension = true; //�Ƿ���ʾ���
    bool isShowResult = true;
    bool isShowLocalMask = true;
    bool isShowGlobalMask = true;

    int cutParameter = 70;
};

enum ReRenderMode {
    ReSetCamera,
    NoSetCamera,
};