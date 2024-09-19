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
#include "ceres/rotation.h"
using namespace cv;
using namespace std;
using namespace Eigen;

void Camera::spaceToPlane(const Quaternionf &q, const Matrix<double, 3, 1> &t,
                          const Matrix<double, 3, 1> &P_w, Matrix<double, 2, 1> &p_uv)
{
    // 从世界坐标系恢复到相机坐标系
    Matrix3d rotate_matrix_cw = q.matrix();
    Matrix<double, 3, 1> P_c = rotate_matrix_cw * P_w;

    // 相机坐标系转换到归一化坐标系
    double P_nx = P_c(0) / P_c(2);
    double P_ny = P_c(1) / P_c(2);

    // 相机坐标系到图像坐标系
    double x = fx_ * P_nx;
    double y = fy_ * P_ny;

    // 图像坐标系到像素坐标系
    double u = x / cx_ + image_width_ / 2;
    double v = y / cy_ + image_height_ / 2;

    p_uv(0) = u;
    p_uv(1) = v;
}

void Camera::liftProjective(const Eigen::Matrix<double, 2, 1> &p_uv, Eigen::Matrix<double, 2, 1> &p_n)
{
    // 相机内参矩阵
    Matrix<double, 3, 3> T_in;

    T_in << fx_ / cx_, 0, image_width_ / 2,
        0, fy_ / cy_, image_height_ / 2,
        0, 0, 1;

    // 像素坐标齐次化
    Matrix<double, 3, 1> p_uv_temp;
    p_uv_temp(0) = p_uv(0);
    p_uv_temp(1) = p_uv(1);
    p_uv_temp(2) = 1;

    // 计算得到相机归一化坐标
    p_n = T_in.inverse() * p_uv_temp;
}

void Camera::estimateExtrinsics(const vector<Point3f> &objectpoints, const vector<Point2f> &imagepoints,
                                Mat tvec, Mat rvec)
{
    vector<Point2f> normpoints;
    for (auto imagepoint : imagepoints)
    {
        Vector2d p_n;
        liftProjective(Vector2d(imagepoint.x, imagepoint.y), p_n);
        Point2f normpoint = Point2f(p_n(0), p_n(1));
        normpoints.push_back(normpoint);
    }
    // 注意这里的外参矩阵和畸变矩阵都为单位矩阵，因为世界坐标系到归一化坐标系之间并没有经过相机
    solvePnP(objectpoints, imagepoints, cv::Mat::eye(3, 3, CV_64F), cv::noArray(), rvec, tvec);
}