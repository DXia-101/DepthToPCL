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

    bool isPickingMode = false; //是否进入标记模式
    bool flag = false;//判断是不是第一次点击
    unsigned int line_id = 0;

    bool isShowDimension = true; //是否显示标记
    bool isShowResult = true;
    bool isShowLocalMask = true;
    bool isShowGlobalMask = true;

    int cutParameter = 70;
};

enum ReRenderMode {
    ReSetCamera,
    NoSetCamera,
};