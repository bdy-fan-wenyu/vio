/**
 * @file imu.h
 * @author 范文宇 (2643660853@qq.com)
 * @brief
 * @version 1.0
 * @date 2024-05-04
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
class Imu
{
  Imu()
  {
    linear_acc_.reserve(1000);
    angle_vel_.reserve(1000);
    P_wb_.reserve(1000);
    Q_wb_.reserve(1000);
    V_wb_.reserve(1000);
    dt_ = 0.001;
    g_ = Eigen::Vector3d(0, 0, -9.8);
  }

  ~Imu();

public:
  // 得到中值积分结果
  void
  getmidPVQ();

public:
  // 更新线速度
  void setLinearVelocity(Eigen::Vector3d);
  // 获取线速度
  std::vector<Eigen::Vector3d> getLinearVelocity();
  // 更新角速度
  void setAngleVelocity(Eigen::Vector3d);
  // 获取角速度
  std::vector<Eigen::Vector3d> getAngleVelocity();
  // 获取Pwb
  std::vector<Eigen::Vector3d> getPwb();
  // 获取Qwb
  std::vector<Eigen::Quaterniond> getQwb();
  // 获取Vwb
  std::vector<Eigen::Vector3d> getVwb();

private:
  // 储存imu直出的线速度
  std::vector<Eigen::Vector3d>
      linear_acc_;
  // 储存imu直出的角速度
  std::vector<Eigen::Vector3d> angle_vel_;
  // 储存imu的Pwb
  std::vector<Eigen::Vector3d> P_wb_;
  // 储存imu的Qwb
  std::vector<Eigen::Quaterniond> Q_wb_;
  // 储存imu的Vwb
  std::vector<Eigen::Vector3d> V_wb_;
  // 帧差间隔 @todo
  float dt_;
  // 重力 @todo
  Eigen::Vector3d g_;
};
typedef std::shared_ptr<Imu> imu_ptr;