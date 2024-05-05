/**
 * @file camera.h
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-04
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
class Camera
{

public:
    Camera()
    {
        images_.reserve(1000);
        camera_out_para_.reserve(1000);
        camera_rt_.reserve(1000);
    };
    ~Camera() = default;
    // 设置相机图像
    void setImage(cv::Mat);
    // 获取相机图像
    std::vector<cv::Mat> getImage();

    // 设置相机内参
    void setCameraInPara(Eigen::Matrix3f);
    // 获取相机内参
    Eigen::Matrix3f getCameraInPara();

    // 设置相机和特征点(世界)的外参
    void setCameraOutPara(Eigen::Matrix4f);
    // 获取相机和特征点(世界)的外参
    std::vector<Eigen::Matrix4f> getCameraOutPara();

private:
    // 图像
    std::vector<cv::Mat> images_;
    // 相机内参
    Eigen::Matrix3f camera_in_para_;
    // 相机外参
    std::vector<Eigen::Matrix4f> camera_out_para_;
    // 相机自身位姿
    std::vector<Eigen::Matrix4f> camera_rt_;
};
inline std::shared_ptr<Camera> Camera_ptr;