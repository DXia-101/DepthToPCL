#include "Transfer_Function.h"

void Transfer_Function::Cloud2cvMat(int width,int height,float originX,float originY, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout)
{
    if (!imageout.empty())
        imageout.release();
    imageout.create(height, width, CV_32FC1);

    for (int i = 0; i < cloudin->size(); ++i) {
        int x = static_cast<int>(cloudin->points[i].x-originX);
        int y = static_cast<int>(cloudin->points[i].y-originY);
        float z = cloudin->points[i].z;

        if (x >= 0 && x < width && y >= 0 && y < height) {
            imageout.at<float>(y, x) = z;
        }
    }
}

void Transfer_Function::cvMat2Cloud(cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    //cloudOut->width = imageIn.cols;
    //cloudOut->height = imageIn.rows;
    //cloudOut->points.resize(cloudOut->width * cloudOut->height);

    for (int x = 0; x < imageIn.cols; ++x) {
        for (int y = 0; y < imageIn.rows; ++y) {
            pcl::PointXYZ point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = imageIn.at<float>(y,x);

            cloudOut->push_back(point);
        }
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
    cv::Mat mask = cv::Mat::zeros(Matin.size(), CV_32F);

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

void Transfer_Function::ExtractImage2Cloud(cv::Mat& imageIn, float originX,float originY , std::vector<cv::Point>* contour, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    cv::Rect boundingRect = cv::boundingRect(*contour);

    for (int y = boundingRect.y; y < boundingRect.y + boundingRect.height; y++)
    {
        for (int x = boundingRect.x; x < boundingRect.x + boundingRect.width; x++)
        {
            cv::Point point(x, y);

            //检查点是否在轮廓内部
            double distance = cv::pointPolygonTest(*contour, point, false);

            if (distance >= 0)
            {
                pcl::PointXYZ point;
                point.x = static_cast<float>(x) + originX;
                point.y = static_cast<float>(y) + originY;
                point.z = imageIn.at<float>(y, x);

                cloudOut->push_back(point);
            }
        }
    }
}

void Transfer_Function::AddPointsInsideContour(cv::Mat& Matin, std::vector<cv::Point>* contour)
{
    cv::Rect boundingRect = cv::boundingRect(*contour);

    for (int y = boundingRect.y; y < boundingRect.y + boundingRect.height; y++)
    {
        for (int x = boundingRect.x; x < boundingRect.x + boundingRect.width; x++)
        {
            cv::Point point(x, y);

            //检查点是否在轮廓内部
            double distance = cv::pointPolygonTest(*contour, point, false);

            if (distance >= 0)
            {
                contour->push_back(point);
            }
        }
    }
}

QPixmap Transfer_Function::loadPixmap(const QString& filePath)
{
    QPixmap pixmap;

    if (pixmap.load(filePath))
    {
        return pixmap;
    }
    else
    {
        return QPixmap();
    }
}
