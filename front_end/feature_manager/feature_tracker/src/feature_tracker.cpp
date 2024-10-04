/**
 * @file feature_tracker.cpp
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-09-21
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "feature_tracker.h"
using namespace cv;
using namespace std;
using namespace Eigen;

/**
 * @brief 用于记录特征点的id
 *
 * @param v
 * @param masK_status
 */
void reduceVector(vector<int> &v, vector<uchar> masK_status)
{
    int j = 0;
    for (int i = 0; i < int(v.size()); i++)
        if (masK_status[i])
            v[j++] = v[i];
    v.resize(j);
}

std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> FeatureTracker::trackImage(
    const cv::Mat &_img)
{
    // 开始写入最终结果，以向外返回
    /* 结构说明：{FeatureID, vector<{CamID, 像素信息(xyz_uv_VxVy)}>} */
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> feats_frame;
    cur_img_ = _img;
    // 如果当前帧不是初始帧
    if (pre_pts_.size() > 0)
    {
        vector<uchar> lk_status; /*这个变量记录了LK追踪成功与否，1表示成功，0为失败*/
        vector<float> lk_err;    /*追踪成功的点的某种误差，*/
        // 光流法实现特征点跟踪
        cv::calcOpticalFlowPyrLK(cur_img_, pre_img_, cur_pts_, pre_pts_, cur_pts_, lk_err, cv::Size(21, 21), 1,
                                 cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        reduceVector(cur_ids_, lk_status);
    }
    // 初始帧或者跟踪特征点数量不够时
    const int max_new_features = 100 - static_cast<int>(cur_pts_.size());
    if (max_new_features > 0)
        cv::goodFeaturesToTrack(cur_img_, new_pts_, max_new_features, 0.01, 10);
    else
        new_pts_.clear();

    // 更新特征点的编号以及特征点
    for (auto &pxl : new_pts_)
    {
        cur_pts_.push_back(pxl);
        cur_ids_.push_back(pt_id_++);
    }
    pts_velocity_ = culFeatureV(cur_ids_, cur_pts_, cur_id_pt_map_, pre_id_pt_map_);

    // 更新前后帧
    pre_img_ = cur_img_;
    pre_id_pt_map_ = cur_id_pt_map_;
    pre_pts_ = cur_pts_;

    // 保存特征点的像素位置，速度
    for (int i = 0; i < cur_ids_.size(); i++)
    {
        int feat_id = cur_ids_[i];
        double point_x = cur_pts_[i].x;
        double point_y = cur_pts_[i].y;
        double point_z = 1;

        // 未修正畸变的像素点位置（忽略）
        double point_u = cur_pts_[i].x;
        double point_v = cur_pts_[i].y;

        // 速度
        double v_x = pts_velocity_[i].x;
        double v_y = pts_velocity_[i].y;

        Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
        xyz_uv_velocity << point_x, point_y, point_z, point_u, point_v, v_x, v_y;
        feats_frame[feat_id].emplace_back(0, xyz_uv_velocity);
    }
}

vector<Point2f> FeatureTracker::culFeatureV(vector<int> &IDs, vector<Point2f> &PTs,
                                            map<int, Point2f> &curIdPtMap,
                                            map<int, Point2f> &preIdPtMap)
{
    vector<Point2f> pts_v;
    curIdPtMap.clear();
    for (int i = 0; i < IDs.size(); i++)
    {
        curIdPtMap.insert(make_pair(IDs[i], PTs[i]));
    }
    // 计算
    if (!preIdPtMap.empty())
    {
        for (unsigned int i = 0; i < PTs.size(); i++)
        {
            std::map<int, cv::Point2f>::iterator it;
            it = preIdPtMap.find(IDs[i]);
            if (it != preIdPtMap.end())
            {
                double v_x = (PTs[i].x - it->second.x) / dt_;
                double v_y = (PTs[i].y - it->second.y) / dt_;
                pts_velocity_.push_back(cv::Point2f(v_x, v_y));
            }
            else
                pts_velocity_.push_back(cv::Point2f(0, 0));
        }
    }
    else
    {
        for (unsigned int i = 0; i < cur_pts_.size(); i++)
        {
            pts_velocity_.push_back(cv::Point2f(0, 0));
        }
    }
    return pts_velocity_;
}