#include "myslam/map.h"
#include "myslam/feature.h"

namespace myslam {

void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;
    if (keyframes_.find(frame->keyframe_id_) == keyframes_.end()) {
        keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if (active_keyframes_.size() > num_active_keyframes_) {
        RemoveOldKeyFrame();
    }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    if (landmarks_.find(map_point->id_) == landmarks_.end()) {
        landmarks_.insert(std::make_pair(map_point->id_, map_point));
        active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

void Map::RemoveOldKeyFrame() {
    if (current_frame_ == nullptr)
        return;
    // 寻找与当前帧最近和最远的两个关键帧
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    SE3 Twc = current_frame_->Pose().inverse();
    for (auto &kf : active_keyframes_) {
        if (kf.second == current_frame_)
            continue;
        double dis = (kf.second->Pose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_thres = 0.2; // 最近阈值
    Frame::Ptr frame_to_remove = nullptr;
    if (min_dis < min_dis_thres) {
        // 如果存在很近的帧，优先删掉最近的
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        // 删掉最远的
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "remove keyframe " << frame_to_remove->keyframe_id_;
    // 删除关键帧和地图点
    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for (auto feature : frame_to_remove->features_left_) {
        auto mp = feature->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(
                feature); // 将要删除的关键帧所观测到的地图点对应的观测剃除
        }
    }
    for (auto feature : frame_to_remove->features_right_) {
        if (feature == nullptr)
            continue;
        auto mp = feature->map_point_.lock();
        if (mp) {
            mp->RemoveObservation(feature);
        }
    }
    CleanMap();
}

/**
 * @brief 清理map中观测数量为0的地图点
 *
 */
void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    for (auto it = active_landmarks_.begin(); it != active_landmarks_.end();) {
        if (it->second->observed_times_ == 0) {
            it = active_landmarks_.erase(it);
        } else {
            ++it;
        }
    }
    LOG(INFO) << "Removed" << cnt_landmark_removed << " active landmarks";
}
} // namespace myslam