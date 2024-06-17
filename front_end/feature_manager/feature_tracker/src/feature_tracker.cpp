/**
 * @file feature_tracker.cpp
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-05
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "feature_tracker.h"
using namespace cv;
using namespace std;

void FeatureTracker::update()
{
    // 获取当前时刻的两帧图像
    vector<Mat> images;
    images = p_camera->getImage();
    int iamges_size = images.size() - 1;
    Mat cur_image = images[iamges_size];
    Mat last_image = images[iamges_size - 1];
    

    if (last_pts.size() == 0)
        cv::goodFeaturesToTrack(cur_image, last_pts, 8, 0.01, 0.1);

    cv::calcOpticalFlowPyrLK(last_image, cur_image, last_pts, cur_pts_ /*输出的追踪结果，允许给出初值*/,
                             states, err, cv::Size(21, 21) /*指定金字塔每层上的搜索窗口的size*/, 1 /*指定金字塔层数为2层*/,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01) /*指定搜索的终止条件*/,
                             cv::OPTFLOW_USE_INITIAL_FLOW /*指定启用cur_pts_中给定的初值，以便加速搜索*/);
    for (auto it : cur_pts_)
    {
        static int id = 0;
        pt_uv_in_image_.insert(pair<int, cv::Point2f>(id, it));
        id += 1;
    }

    
}