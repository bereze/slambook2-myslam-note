/**
 * @file frame.h
 * @author  luo (you@domain.com)
 * @brief  Frame帧设计，含有id、位姿、左右图像及左右图像的特征点等。
 * @version 0.1
 * @date 2021-04-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include "myslam/common_include.h"

namespace myslam {

// forward declare
struct MapPoint;
struct Feature;

/**
 * @brief 帧
 * 每一帧分配独立id，关键帧分配关键帧id
 */
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frame> Ptr;

    unsigned long id_ = 0;          // id of this frame
    unsigned long keyframe_id_ = 0; // id of key frame
    bool is_keyframe_ = false;      // 是否为关键帧
    double time_stamp_;             // 时间戳
    SE3 pose_;                      // Tcw形式Pose
    std::mutex pose_mutex_;         // Pose数据锁
    cv::Mat left_img_, right_img_;  // stereo images

    // extracted features in left image
    std::vector<std::shared_ptr<Feature>> features_left_;
    // corresponding features in right image, set to nullptr if no corresponding
    std::vector<std::shared_ptr<Feature>> features_right_;

public:
    Frame() {}
    Frame(long id, double time_stamp, const SE3 &pose, const Mat &left,
          const Mat &right);

    // set and get pose, thread safe
    // 因为pose会被前后端同时访问，需要加锁
    SE3 Pose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    // 设置关键帧并分配关键帧id
    void SetKeyFrame();

    // 工厂构建模式，分配id
    static std::shared_ptr<Frame> CreateFrame();
};
} // namespace myslam

#endif // MYSLAM_FRAME_H