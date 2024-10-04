/**
 * @file feature.cpp
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-05
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "feature.h"
using namespace Eigen;
using namespace cv;
using namespace std;
void FeatureManager::triangulate(int frameCnt, Vector3d Pwb[], Matrix3d Rwb[], Vector3d tbc, Matrix3d rbc, Vector3d &point_3d)
{
    for (auto feature : features_)
    {
        frameCnt = feature.start_frame;
        // 计算当前帧相机相对于世界坐标系的Rwc Pwc
        Matrix<double, 4, 3> cur_camera_pose;
        Vector3d Rwc_cur = Rwb[frameCnt] * rbc;
        Vector3d Pwc_cur = Pwb[frameCnt] + Rwb[frameCnt] * rbc;
        cur_camera_pose.leftCols<3>() = Rwc_cur.transpose();
        cur_camera_pose.leftCols<1>() = -Rwc_cur.transpose() * Pwc_cur;

        // 计算下一帧相机相对于世界坐标的Rwc Pwc
        Matrix<double, 4, 3> next_camera_pose;
        Vector3d Rwc_next = Rwb[frameCnt + 1] * rbc;
        Vector3d Pwc_next = Pwb[frameCnt + 1] + Rwb[frameCnt + 1] * rbc;
        next_camera_pose.leftCols<3>() = Rwc_next.transpose();
        next_camera_pose.leftCols<1>() = -Rwc_next.transpose() * Pwc_next;

        // 特征点当前的像素坐标
        Vector2d point_uv_cur = feature.feature_per_frame[0].point;
        // 特征点下一帧的像素坐标
        Vector2d point_uv_next = feature.feature_per_frame[1].point;

        // 三角测量法
        Matrix<double, 4, 4> design_matrix = Matrix4d::Zero();
        design_matrix.row(0) = point_uv_cur[0] * cur_camera_pose.row(2) - cur_camera_pose.row(0);
        design_matrix.row(1) = point_uv_cur[1] * cur_camera_pose.row(2) - cur_camera_pose.row(1);
        design_matrix.row(2) = point_uv_next[0] * next_camera_pose.row(2) - next_camera_pose.row(0);
        design_matrix.row(3) = point_uv_next[1] * next_camera_pose.row(2) - next_camera_pose.row(1);

        Eigen::Vector4d triangulated_point;
        triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
        point_3d(0) = triangulated_point(0) / triangulated_point(3);
        point_3d(1) = triangulated_point(1) / triangulated_point(3);
        point_3d(2) = triangulated_point(2) / triangulated_point(3); /*通过三角化，得到了特征的3D坐标*/
    }
}