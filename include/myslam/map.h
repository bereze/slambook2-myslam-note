/**
 * @file map.h
 * @author your name (you@domain.com)
 * @brief
 * 地图类，实际持有Frame和MapPoint对象，以散列表形式记录了所有的关键帧和对应的地图点，
 * 同时维护一个被激活的关键帧和地图点的窗口，这里以只保留最新的7个关键帧为激活策略
 * @version 0.1
 * @date 2021-04-27
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once
#ifndef MAP_H
#define MAP_H

#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"

namespace myslam {

/**
 * @brief 地图
 * 和地图的交互：
 * 前端调用InsertKeyFrame和InsertMapPoint插入新帧和地图点，
 * 后端维护地图的结构，判定outlier/剔除等
 */
class Map {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

    Map() {}

    // 增加一个关键帧
    void InsertKeyFrame(Frame::Ptr frame);
    // 增加一个地图点
    void InsertMapPoint(MapPoint::Ptr map_point);

    // 获取所有地图点
    LandmarksType GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }
    // 获得所有关键帧
    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    // 获得激活的地图点
    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }
    // 获得激活的关键帧
    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    // 清理map中观测数量为0的点
    void CleanMap();

private:
    std::mutex data_mutex_;
    LandmarksType landmarks_;        // 所有地图点
    LandmarksType active_landmarks_; // 激活的地图点
    KeyframesType keyframes_;        // 所有关键帧
    KeyframesType active_keyframes_; // 激活的关键帧

    Frame::Ptr current_frame_ = nullptr;

    // settings
    int num_active_keyframes_ = 7; // 激活的关键帧数量

    // 将旧的关键帧置为不活跃状态
    void RemoveOldKeyFrame();
};

} // namespace myslam

#endif