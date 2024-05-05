/**
 * @file camera.cpp
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <camera.h>
using namespace cv;
using namespace std;
using namespace Eigen;

void Camera::setImage(cv::Mat image)
{
    if (images_.size() > 1000)
        images_.pop_back();
    images_.push_back(image);
}

vector<Mat> Camera::getImage()
{
    return images_;
}

void Camera::setCameraInPara(Matrix3f camera_in_para)
{
    camera_in_para_ = camera_in_para;
}

Matrix3f Camera::getCameraInPara()
{
    return camera_in_para_;
}

void Camera::setCameraOutPara(Matrix4f camera_out_para)
{
    if (camera_out_para_.size() > 1000)
        camera_out_para_.pop_back();
    camera_out_para_.push_back(camera_out_para);
}

vector<Matrix4f> Camera::getCameraOutPara()
{
    return camera_out_para_;
}