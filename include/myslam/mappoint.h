#pragma once
#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam {

struct Frame;
struct Feature;

/**
 * @brief 地图点类，根据特征点三角化得到
 *
 */
struct MapPoint {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long id_ = 0;
    bool is_outlier_ = false;
    Vec3 pos_ = Vec3::Zero(); // 世界坐标
    std::mutex data_mutex_;
    int observed_times_ = 0;                        // 被观测到的次数
    std::list<std::weak_ptr<Feature>> observation_; // 记录了被哪些Feature观测到

public:
    MapPoint() {}

    MapPoint(long id, Vec3 position);

    Vec3 Pos() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return pos_;
    }

    void SetPos(const Vec3 &pos) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        pos_ = pos;
    }

    void AddObservation(std::shared_ptr<Feature> feature) {
        std::unique_lock<std::mutex> lck(data_mutex_);
        observation_.push_back(feature);
        observed_times_++;
    }

    void RemoveObservation(std::shared_ptr<Feature> feature);

    std::list<std::weak_ptr<Feature>> GetObs() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return observation_;
    }

    // factory function
    static MapPoint::Ptr CreateNewMappoint();
};

} // namespace myslam

#endif
