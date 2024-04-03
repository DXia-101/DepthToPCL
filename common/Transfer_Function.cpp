#include "Transfer_Function.h"
#include "Depth2RGB.h"

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

void Transfer_Function::cvMat2Cloud(int minHeight, int maxHeight, cv::Mat& imageIn, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    cloudOut->points.clear();

    for (int x = 0; x < imageIn.cols; ++x) {
        for (int y = 0; y < imageIn.rows; ++y) {
            pcl::PointXYZ point;
            point.x = static_cast<float>(x);
            point.y = static_cast<float>(y);
            point.z = imageIn.at<float>(y,x);
            if (point.z > minHeight && point.z < maxHeight) {
                cloudOut->push_back(point);
            }
        }
    }
}

void Transfer_Function::cvMat2Contour(cv::Mat& Matin, std::vector<std::vector<cv::Point>>* contours)
{
    cv::Mat grayImg, binImg;
    TeJetColorCode trans;
    cv::Mat median;
    median.create(Matin.size(), CV_8UC3);
    if (trans.cvt32F2BGR(Matin, median)) {
        cv::cvtColor(median, grayImg, cv::COLOR_BGR2GRAY);
        threshold(grayImg, binImg, 0, 255, cv::ThresholdTypes::THRESH_OTSU);
        std::vector<cv::Vec4i> hierarchy;
        findContours(binImg, *contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);//会将整个图像的最外层轮廓算进去
    }
}

void Transfer_Function::ExtractImage(cv::Mat& Matin, std::vector<cv::Point>* contour, cv::Mat* extractedImages)
{
    cv::Mat mask = cv::Mat::zeros(Matin.size(), CV_32F);

    std::vector<std::vector<cv::Point>> contours;
    contours.push_back(*contour);

    cv::fillPoly(mask, contours, cv::Scalar(255));

    Matin.copyTo(*extractedImages, mask);
}

bool Transfer_Function::isPointInsideContour(int x, int y, std::vector<cv::Point>* contour)
{
    cv::Point point(x, y);

    double distance = cv::pointPolygonTest(*contour, point, false);

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

void Transfer_Function::ExtractCloud2Cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, std::vector<cv::Point>* contour, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut)
{
    cloudOut->clear();

    int minX = std::numeric_limits<int>::max();
    int minY = std::numeric_limits<int>::max();
    int maxX = std::numeric_limits<int>::min();
    int maxY = std::numeric_limits<int>::min();
    for (const auto& point : *contour)
    {
        minX = std::min(minX, point.x);
        minY = std::min(minY, point.y);
        maxX = std::max(maxX, point.x);
        maxY = std::max(maxY, point.y);
    }

    for (const auto& point : cloudIn->points)
    {
        Eigen::Vector3f pointEigen(point.x, point.y, point.z);

        //Eigen::Vector3f pointProjected = pointEigen;
        pointEigen.z() = 0.0;

        if (pointEigen.x() >= minX && pointEigen.x() <= maxX && pointEigen.y() >= minY && pointEigen.y() <= maxY)
        {
            cloudOut->points.push_back(point);
        }
    }

    cloudOut->width = cloudOut->points.size();
    cloudOut->height = 1;
    cloudOut->is_dense = true;
}

void Transfer_Function::AddPointsInsideContour(cv::Mat& Matin, std::vector<cv::Point>* contour)
{
    cv::Rect boundingRect = cv::boundingRect(*contour);

    for (int y = boundingRect.y; y < boundingRect.y + boundingRect.height; y++)
    {
        for (int x = boundingRect.x; x < boundingRect.x + boundingRect.width; x++)
        {
            cv::Point point(x, y);

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

te::AiInstance Transfer_Function::VectorToAiInstance(std::vector<std::vector<cv::Point>>* Contours)
{
    te::AiInstance instance;

    for (auto polygon : *Contours) {
        te::PolygonF polygonF;
        te::Point2f tePoint;
        for (auto point : polygon) {
            tePoint.x = static_cast<float>(point.x);
            tePoint.y = static_cast<float>(point.y);
            polygonF.push_back(tePoint);
        }
        instance.contour.polygons.push_back(polygonF);
    }
    
    return instance;
}