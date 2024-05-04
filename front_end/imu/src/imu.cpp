/**
 * @file imu.cpp
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#include "imu.h"
void Imu::setLinearVelocity(Eigen::Vector3d cur_linear_v)
{
    linear_acc_.push_back(cur_linear_v);
}

std::vector<Eigen::Vector3d> Imu::getLinearVelocity()
{
    return linear_acc_;
}

void Imu::setAngleVelocity(Eigen::Vector3d cur_angle_v)
{
    angle_vel_.push_back(cur_angle_v);
}

void Imu::getmidPVQ()
{
    // 获取角速度队列的大小
    int buf_angle_w_size = angle_vel_.size() - 1;

    // 中值法计算当前时刻的角速度
    Eigen::Vector3d cur_mid_w;
    cur_mid_w = (angle_vel_[buf_angle_w_size] + angle_vel_[buf_angle_w_size - 1]) / 2.f;

    // 获取当前时刻的四元数
    Eigen::Quaterniond dq;
    Eigen::Vector3d dtheta_half = (cur_mid_w)*dt_ / 2.0;
    dq.w() = 1;
    dq.x() = dtheta_half.x();
    dq.y() = dtheta_half.y();
    dq.z() = dtheta_half.z();

    // 计算当前时刻的qwb
    Eigen::Quaterniond cur_q_wb = Q_wb_[buf_angle_w_size - 1] * dq;

    // 中值法计算当前时刻的a
    Eigen::Vector3d cur_mid_a;
    cur_mid_a = cur_q_wb * cur_mid_w + g_;

    // 计算当前时刻的Vwb
    Eigen::Vector3d cur_v_wb = V_wb_[buf_angle_w_size - 1] + cur_mid_a * dt_;

    // 计算当前时刻的Pwb
    Eigen::Vector3d cur_p_wb = P_wb_[buf_angle_w_size - 1] + V_wb_[buf_angle_w_size - 1] * dt_ + cur_mid_a * dt_ * dt_ / 2;

    // 更新PVQ队列
    P_wb_.push_back(cur_p_wb);
    V_wb_.push_back(cur_v_wb);
    Q_wb_.push_back(cur_q_wb);
}
