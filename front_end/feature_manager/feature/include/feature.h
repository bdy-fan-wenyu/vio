/**
 * @file feature.h
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
/**
 * @brief 一个特征点，特征点的最小单位
 *
 */
class FeaturePerFrame
{
public:
    FeaturePerFrame(const Eigen::Matrix<double, 7, 1> &_point, double td)
    {
        cur_td = td;
        point.x() = _point(0);
        point.y() = _point(1);
        point.z() = _point(2);
        uv.x() = _point(3);
        uv.y() = _point(4);
        velocity.y() = _point(5);
        velocity.y() = _point(6);
    }


    double cur_td;
    Eigen::Vector3d point;    // 特征点的像素坐标（去相机畸变后的），第三维实际无效
    Eigen::Vector2d uv;       // 特征点的原始像素坐标（未去畸变）
    Eigen::Vector2d velocity; // 像素速度
};

/**
 * @brief 为一个特征点可能在多个帧（frame）中被追踪到，并且在不同帧中的像素坐标都不一样，
 *        这个结构体就是锚定FeatureID，保存特征点在所有帧中的观测信息。
 *
 */
class FeaturePerId
{
public:
    FeaturePerId(int _featureId, int _startFrame)
        : feature_id(_featureId), start_frame(_startFrame), used_num(0), estimated_depth(-1.0), solve_flag(0) {}

    // 当前，观测到这个特征点的最后一帧的id
    int endFrame() { return start_frame + feature_per_frame.size() - 1; }

public:
    std::vector<FeaturePerFrame> feature_per_frame; /*特征点在不同帧中的观测描述*/
    const int feature_id;
    int start_frame; /*注意这个frame索引是动态调整的，总是与滑窗帧的序号保持一致*/
    int used_num;
    double estimated_depth;
    int solve_flag; /* 0 haven't solve yet; 1 solve succ; 2 solve fail */
};

class FeatureManager
{
public:
    FeatureManager();
    void clearState();
    /**
     * @brief 三角化，求解特征点的深度以及3D坐标
     * 
     * @param frameCnt 当前特征点属于的帧数
     * @param Pwb imu相对于世界坐标的位置
     * @param Rwb imu相对于世界坐标的姿态
     * @param tbc 相机相对于imu的平移
     * @param rbc 相机相对于imu的旋转
     */
    void triangulate(int frameCnt, Eigen::Vector3d Pwb[], Eigen::Matrix3d Rwb[],
                     Eigen::Vector3d tbc, Eigen::Matrix3d rbc, Eigen::Vector3d &point_3d);

private:
    std::list<FeaturePerId> features_;
};