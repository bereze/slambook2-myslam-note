/**
 * @file feature.h
 * @author your name (you@domain.com)
 * @brief
 *  Feature类最主要的信息是自身在图像中的2D位置，此外，还包括异常点标志位，
 * 是否是在左侧相机提取的标志位。
 *  可以通过一个Feature对象访问持有它的Frame和它对应的地图点。
 *  注意，Frame和MapPoint的实际持有权归地图所有，为避免shared_ptr产生的循环引用，
 * 这里使用了weak_ptr。
 * @version 0.1
 * @date 2021-04-27
 *
 * @copyright Copyright (c) 2021
 *
 */

#pragma once
#ifndef MYSLAM_FEATURE_H
#define MYSLAM_FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"

namespace myslam {

struct Frame;
struct MapPoint;

/**
 * @brief 2D特征点，在三角化后会被关联到一个地图点
 *
 */
struct Feature {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;        // 持有该feature的frame
    cv::KeyPoint position_;             // 2D位置
    std::weak_ptr<MapPoint> map_point_; // 关联到的地图点

    bool is_outlier_ = false; // 是否为异常点
    bool is_on_left_image_ = true; // true-在左图中提取的，false-在右图中提取的

public:
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
        : frame_(frame), position_(kp) {}
};

} // namespace myslam

#endif MYSLAM_FEATURE_H