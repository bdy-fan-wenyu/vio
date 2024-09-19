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
/**
 * @brief 小孔成像相机模型（不考虑畸变），主要实现
 *        ①世界坐标系到像素坐标系的转换
 *        ②世界坐标系到归一化坐标系的转换
 *        ③计算相机的外参矩阵
 */
class Camera
{

public:
    /**
     * @brief 构造函数
     *
     * @param camera_name 相机名字
     * @param image_width 图像宽度
     * @param image_height 图像高度
     * @param fx 焦点x
     * @param fy 焦点y
     * @param cx 像元大小x
     * @param cy 像元大小y
     */
    Camera(const string &camera_name, double image_width, double image_height,
           double fx, double fy, double cx, double cy)
        : image_width_(image_width), image_height_(image_height),
          fx_(fx), fy_(fy), cx_(cx), cy_(cy) {

          };
    /**
     * @brief 析构函数
     *
     */
    ~Camera() = default;
    /**
     * @brief 根据RT和路标点的世界坐标，在相机模型中计算出路标点的像素坐标
     *
     * @param q 世界坐标系到相机坐标的坐标变换，四元数
     * @param t 世界坐标系到相机坐标的坐标变换，位移量
     * @param P_w 路标点的世界坐标
     * @param p_uv 路标点的像素坐标
     */
    void spaceToPlane(const Quaternionf &q, const Eigen::Matrix<double, 3, 1> &t,
                      const Eigen::Matrix<double, 3, 1> &P_w, Eigen::Matrix<double, 2, 1> &p_uv);

    /**
     * @brief 像素坐标系到相机归一化坐标系的转化
     *
     * @param p_uv 路标点的像素坐标
     * @param p_n 路标点的归一化坐标
     */
    void liftProjective(const Eigen::Matrix<double, 2, 1> &p_uv, Eigen::Matrix<double, 2, 1> &p_n);
    /**
     * @brief 根据路标点的
     * 
     * @param objectpoints 
     * @param imagepoints 
     * @param tec 
     * @param rec 
     */
    void estimateExtrinsics(const std::vector<cv::Point3f> &objectpoints, const std::vector<cv::Point2f> &imagepoints,
                            cv::Mat tec, cv::Mat rec);

private:
    double image_width_;
    double image_height_;
    double fx_;
    double fy_;
    double cx_;
    double cy_;
};
typedef std::shared_ptr<Camera> camera_ptr;