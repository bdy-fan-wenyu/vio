/**
 * @file estimator.cpp
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-10-01
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "estimator.h"
#include <algorithm>
using namespace std;
using namespace cv;
using namespace Eigen;

void Estimator::inputImu(double t, double dt, Vector3d linearAcc, Vector3d angularVel)
{
    gyr_buf_.push_back( angularVel);
    acc_buf_.push_back( linearAcc);
}

void Estimator::processImu(double t, double dt, Vector3d linearAcc, Vector3d angularVel)
{
    imu_.setAngleVelocity(angularVel);
    imu_.setLinearVelocity(linearAcc);
    imu_.getmidPVQ();
    Pwb[frame_count_] = imu_.getPwb()[imu_.getPwb().size() - 1];
    Vwb[frame_count_] = imu_.getVwb()[imu_.getVwb().size() - 1];
    Rwb[frame_count_] = imu_.getQwb()[imu_.getQwb().size() - 1];
}

void Estimator::inputImage(double t, double dt, Mat &img)
{
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double, 7, 1>>>> img_features;
    img_features = feature_tracker_.trackImage(img);
    feature_buf_.push(make_pair(t, img_features));
}

void Estimator::processImage(double t, const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> &img_feats)
{
    if (flag_solver_type_ == 0)
    {
        initStructure();
        slideWindow();
        if (frame_count_ < WINDOW_SIZE)
        {
            frame_count_++; /* 唯一改变取值的地方，意味着索引的最大取值正是WIN_SIZE */
            int prev_frame = frame_count_ - 1;
            Pwb[frame_count_] = Pwb[prev_frame];
            Vwb[frame_count_] = Vwb[prev_frame];
            Rwb[frame_count_] = Rwb[prev_frame];
            Bas[frame_count_] = Bas[prev_frame];
            Bgs[frame_count_] = Bgs[prev_frame];
        }
    }
    else
    {
        Eigen::Vector3d point_3d;
        feature_manager_.triangulate(frame_count_, Pwb, Rwb, Tbc, Rbc, point_3d);
        slideWindow();
    }
}

void Estimator::slideWindow()
{
    if (frame_count_ == WINDOW_SIZE)
    {
        for (int i = 0; i < WINDOW_SIZE; i++)
        {
            Rwb[i].swap(Rwb[i + 1]);
            Pwb[i].swap(Pwb[i + 1]);
            acc_buf_[i].swap(acc_buf_[i + 1]);
            gyr_buf_[i].swap(gyr_buf_[i + 1]);

            Vwb[i].swap(Vwb[i + 1]);
            Bas[i].swap(Bas[i + 1]);
            Bgs[i].swap(Bgs[i + 1]);
        }
    }
}