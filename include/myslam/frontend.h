#pragma once
#ifndef MYSLAM_FRONTEND_H
#define MYSLAM_FRONTEND_H

#include <opencv2/features2d.hpp>

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/map.h"

namespace myslam {

class Backend;
class Viewer;

enum class FrontendStatus { INITING, TRACKING_GOOD, TRACKING_BAD, LOST };

/**
 * @brief 前端类
 * 估计当前帧Pose，在满足关键帧条件时向地图加入关键帧并触发后端优化
 */
class Frontend {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Frontend> Ptr;

    Frontend();

    // 外部接口，添加一个帧并计算其定位结果
    bool AddFrame(Frame::Ptr frame);

    // set函数
    void SetMap(Map::Ptr map) { map_ = map; }
    void SetBackend(std::shared_ptr<Backend> backend) { backend_ = backend; }
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

    FrontendStatus GetStatus() const { return status_; }

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    // data
    FrontendStatus status_ = FrontendStatus::INITING;

    Frame::Ptr current_frame_ = nullptr; // 当前帧
    Frame::Ptr last_frame_ = nullptr;    // 上一帧
    Camera::Ptr camera_left_ = nullptr;  // 左侧相机
    Camera::Ptr camera_right_ = nullptr; // 右侧相机

    Map::Ptr map_ = nullptr;
    std::shared_ptr<Backend> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_; // 当前帧与上一帧的相对运动，用于估计当前帧Pose初值

    int tracking_inliers_ = 0; // inliers，用于判定新的关键帧

    // params
    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    // utilities
    cv::Ptr<cv::GFTTDetector> gftt_; // feature detector in opencv

    // functions
    /**
     * @brief Track in normal mode
     *
     * @return true if success
     */
    bool Track();

    /**
     * @brief Reset when lost
     *
     * @return true if success
     */
    bool Reset();

    /**
     * @brief Track with last frame
     *
     * @return int num of tracked points
     */
    int TrackLastFrame();

    /**
     * @brief Estimate current frame's pose
     *
     * @return int num of inliers
     */
    int EstimateCurrentPose();

    /**
     * @brief Set current frame as a keyframe and insert it into backend
     *
     * @return true if success
     */
    bool InsertKeyFrame();

    /**
     * @brief Try init the fronted with stereo images saved in current_frame_
     *
     * @return true if success
     */
    bool StereoInit();

    /**
     * @brief Detect features in left image in current_frame_
     * keypoints will be saved in current_frame_
     *
     * @return int
     */
    int DetectFeatures();

    /**
     * @brief Find the corresponding features in right image of current_frame_
     *
     * @return int num of features found
     */
    int FindFeaturesInRight();

    /**
     * @brief Build the initial map with single image
     *
     * @return true if success
     */
    bool BuildInitMap();

    /**
     * @brief Triangulate the 2D points in current_frame_
     *
     * @return int num of triangulated points
     */
    int TriangulateNewPoints();

    /**
     * @brief Set the features in keyframe as new observation of the map points
     *
     */
    void SetObservationsForKeyFrame();
};
} // namespace myslam

#endif