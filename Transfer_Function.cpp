#include "Transfer_Function.h"

void Transfer_Function::Cloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr datumPointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout)
{
    pcl::PointXYZ min;//用于存放三个轴的最小值
    pcl::PointXYZ max;//用于存放三个轴的最大值
    pcl::getMinMax3D(*datumPointCloud, min, max);
    int currentDisplayImageLength = max.x - min.x;
    int currentDisplayImageHeight = max.y - min.y;

    if (!imageout.empty())
        imageout.release();
    imageout.create(currentDisplayImageHeight, currentDisplayImageLength, CV_8UC3);

    for (int i = 0; i < cloudin->size(); i++)
    {
        //计算点对应的像素坐标
        int x = (cloudin->points[i].x - min.x);
        int y = (cloudin->points[i].y - min.y);

        //将颜色信息赋予像素
        if (x > 0 && x < currentDisplayImageLength && y>0 && y < currentDisplayImageHeight)
        {
            imageout.at<cv::Vec3b>(y, x)[0] = cloudin->points[i].z;
            imageout.at<cv::Vec3b>(y, x)[1] = cloudin->points[i].z;
            imageout.at<cv::Vec3b>(y, x)[2] = cloudin->points[i].z;
        }
    }
}

void Transfer_Function::cvMat2Cloud(cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    cloudOut->width = imageIn.cols;
    cloudOut->height = imageIn.rows;
    cloudOut->points.resize(cloudOut->width * cloudOut->height);

    cv::MatIterator_<float> pixel_it, pixel_end;
    pixel_it = imageIn.begin<float>();
    pixel_end = imageIn.end<float>();

    for (int i = 0; pixel_it != pixel_end; ++pixel_it, ++i) {
        int y = i / imageIn.cols;
        int x = i % imageIn.cols;
        float depth = *pixel_it;

        cloudOut->at(x, y).x = static_cast<float>(x);
        cloudOut->at(x, y).y = static_cast<float>(y);
        cloudOut->at(x, y).z = depth;
    }
}

void Transfer_Function::cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours)
{
    cv::Mat grayImg, binImg;
    cv::cvtColor(Matin, grayImg, cv::COLOR_BGR2GRAY);
    threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);
    std::vector<cv::Vec4i> hierarchy;
    findContours(binImg, *contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);//会将整个图像的最外层轮廓算进去
}

void Transfer_Function::ExtractImage(cv::Mat& Matin, std::vector<cv::Point>* contour, cv::Mat* extractedImages)
{
    cv::Mat mask = cv::Mat::zeros(Matin.size(), CV_8U);

    // 将多边形的点集转换为OpenCV所需的格式
    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(*contour);

    // 在掩膜图像上绘制多边形
    cv::fillPoly(mask, contours, cv::Scalar(255));

    // 将掩膜应用于输入图像，提取感兴趣的区域
    Matin.copyTo(*extractedImages, mask);
}

bool Transfer_Function::isPointInsideContour(int x, int y, std::vector<cv::Point>* contour)
{
    cv::Point point(x, y);

    // 判断点是否在多边形内部
    double distance = cv::pointPolygonTest(*contour, point, false);

    // 如果距离为正值，点在多边形内部
    if (distance >= 0)
        return true;
    else
        return false;
}

void Transfer_Function::ExtractImage2Cloud(cv::Mat& imageIn, std::vector<cv::Point>* contour, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    int width = imageIn.cols;
    int height = imageIn.rows;

    for (int y = 0; y < height; y++)
    {
        for (int x = 0; x < width; x++)
        {
            int intensity = static_cast<int>(imageIn.at<uchar>(y, x));
            if (isPointInsideContour(x, y, contour))
            {
                pcl::PointXYZ point;
                point.x = static_cast<float>(x);
                point.y = static_cast<float>(y);
                point.z = static_cast<float>(intensity);

                cloudOut->push_back(point);
            }
        }
    }
}
