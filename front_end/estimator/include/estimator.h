/**
 * @file estimator.h
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-09-20
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <deque>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include "imu.h"
#include "feature.h"
#include "feature_tracker.h"

const int WINDOW_SIZE = 100;

class Estimator
{
public:
    Estimator();
    ~Estimator();
    /**
     * @brief 存储imu直出数据
     *
     * @param dt 时间间隔
     * @param linearAcc 线速度
     * @param angularVel 角速度
     */
    void inputImu(double t, double dt, Eigen::Vector3d linearAcc, Eigen::Vector3d angularVel);

    /**
     * @brief 对imu数据中值积分获取Pwb，Rwb
     *
     * @param dt 时间间隔
     * @param linearAcc 线速度（imu直出）
     * @param angularVelo 角速度（imu直出）
     */
    void processImu(double t, double dt, Eigen::Vector3d linearAcc, Eigen::Vector3d angularVel);

    /**
     * @brief 传入图像，运行processmeanurement
     *
     * @param t 当前时间
     * @param dt 时间间隔
     * @param img 传入图像
     */
    void inputImage(double t, double dt, cv::Mat &img);
    /**
     * @brief 图像处理，递推一次VIO系统，【算法pipeline】在这里（包括：feature初始化深度、BA优化、移除外点、滑动一次窗口、更新latest结果）
     *
     * @param t
     * @param img_feats
     */
    void processImage(double t, const std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> &img_feats);

    /**
     * @brief 第一帧初始化
     *
     */
    void initStructure();
    
    /**
     * @brief 滑动窗口
     * 
     */
    void slideWindow();

private:
    Eigen::Vector3d linearAcc_;
    Eigen::Vector3d angularVel_;
    Imu imu_;
    std::vector < Eigen::Vector3d> acc_buf_;                                                                                          // 原始imu观测队列
    std::vector < Eigen::Vector3d> gyr_buf_;                                                                                           // 原始imu观测队列
    std::queue<std::pair<double, std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>>> feature_buf_; // 原始imu观测队列
    int frame_count_;                                                                                                  // 当前滑窗帧数
    Eigen::Vector3d Pwb[(WINDOW_SIZE + 1)];                                                                            // 滑窗状态量：位置
    Eigen::Vector3d Vwb[(WINDOW_SIZE + 1)];                                                                            // 滑窗状态量：速度
    Eigen::Matrix3d Rwb[(WINDOW_SIZE + 1)];                                                                            // 滑窗状态量：姿态
    Eigen::Vector3d Bas[(WINDOW_SIZE + 1)];                                                                            // 滑窗状态量：加计零偏
    Eigen::Vector3d Bgs[(WINDOW_SIZE + 1)];                                                                            // 滑窗状态量：陀螺仪零偏
    Eigen::Vector3d Rbc;                                                                                               // imu到camera的旋转矩阵
    Eigen::Vector3d Tbc;                                                                                               // imu到camera的平移矩阵
    FeatureTracker feature_tracker_;                                                                                   // 特征点跟踪器
    FeatureManager feature_manager_;                                                                                   // 特征点管理器
    int flag_solver_type_;                                                                                             // 初始化标志位
};