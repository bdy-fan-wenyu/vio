/**
 * @file feature_tracker.h
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include "imu.h"
#include <vector>
#include "camera.h"
#include <Eigen/Dense>
#include <unordered_map>
#include <opencv2/opencv.hpp>
class FeatureTracker
{
public:
    FeatureTracker(const camera_ptr &cp, const imu_ptr &ip) : p_camera(cp), p_imu(ip)
   {}

   void update();

private:
    // 一帧图像里所有的像素特征点，key:特征点ID value:特征点
    std::unordered_map<int, cv::Point2f> pt_uv_in_image_;

    // 所有图像，key:图像ID value:图像的特征点信息
    std::unordered_map<int, std::unordered_map<int, cv::Point2f>> fearures_image_;

    camera_ptr p_camera;

    std::vector<cv::Point2f> last_pts;
    std::vector<cv::Point2f> cur_pts_;
    std::vector<uchar> states;
    std::vector<float> err;
    
    imu_ptr p_imu;
};
