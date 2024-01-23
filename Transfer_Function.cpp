#include "Transfer_Function.h"

void Transfer_Function::Cloud2cvMat(pcl::PointCloud<pcl::PointXYZ>::Ptr datumPointCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin, cv::Mat& imageout)
{
    pcl::PointXYZ min;//���ڴ�����������Сֵ
    pcl::PointXYZ max;//���ڴ������������ֵ
    pcl::getMinMax3D(*datumPointCloud, min, max);
    int currentDisplayImageLength = max.x - min.x;
    int currentDisplayImageHeight = max.y - min.y;

    if (!imageout.empty())
        imageout.release();
    imageout.create(currentDisplayImageHeight, currentDisplayImageLength, CV_8UC3);

    for (int i = 0; i < cloudin->size(); i++)
    {
        //������Ӧ����������
        int x = (cloudin->points[i].x - min.x);
        int y = (cloudin->points[i].y - min.y);

        //����ɫ��Ϣ��������
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
    findContours(binImg, *contours, hierarchy, cv::RetrievalModes::RETR_TREE, cv::CHAIN_APPROX_NONE);//�Ὣ����ͼ���������������ȥ
}
