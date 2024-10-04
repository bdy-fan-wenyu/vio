/**
 * @file feature_tracker.h
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-09-21
 *
 * @copyright Copyright (c) 2024
 *
 */
#include <Eigen/Dense>
#include <unordered_map>
#include <opencv2/opencv.hpp>
/**
 * @brief 特征点跟踪模块，主要实现一下功能
 *        ① 光流法跟踪帧间特征点
 *        ② 计算特征点的速度，用于做预测
 *
 */
class FeatureTracker
{
public:
    FeatureTracker() = default;
    /**
     * @brief 利用光流法计算得到前后两帧之间的相同特征点
     *
     * @param _cur_time
     * @param _img
     * @param _img1
     * @return std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>>
     */
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(
        const cv::Mat &_img);
    /**
     * @brief 计算特征点的移动速度
     *
     * @param IDs 特征点的id
     * @param PTs 特征点
     * @param curIdPtMap （输出）使用map来记录当前特征点
     * @param preIdPtMap  （输入）上一帧特征点的map格式
     * @return std::vector<cv::Point2f>
     */
    std::vector<cv::Point2f> culFeatureV(std::vector<int> &IDs, std::vector<cv::Point2f> &PTs,
                                         std::map<int, cv::Point2f> &curIdPtMap, 
                                         std::map<int, cv::Point2f> &preIdPtMap);

private:
    // 上一帧图像
    cv::Mat pre_img_;
    // 当前帧图像
    cv::Mat cur_img_;

    // 上一帧跟踪的特征点
    std::vector<cv::Point2f> pre_pts_;
    // 当前帧跟踪的特征点
    std::vector<cv::Point2f> cur_pts_;
    // 新增的特征点
    std::vector<cv::Point2f> new_pts_;
    // 特征点的速度
    std::vector<cv::Point2f> pts_velocity_;
    // 特征点的id，是区别同一帧特征点的唯一方法
    std::vector<int> cur_ids_;
    // 使用map来记录当前特征点
    std::map<int, cv::Point2f> cur_id_pt_map_;
    // 使用map来记录上一帧特征点
    std::map<int, cv::Point2f> pre_id_pt_map_;
    // 特征点的id，随着时间不断递增
    int pt_id_ = 0;
    // 记录帧差时间
    double dt_ = 1 / 60;
};