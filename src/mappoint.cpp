#include "myslam/mappoint.h"
#include "myslam/feature.h"

namespace myslam {

MapPoint::MapPoint(long id, Vec3 position) : id_(id), pos_(position) {}

MapPoint::Ptr MapPoint::CreateNewMappoint() {
    static long factory_id = 0;
    MapPoint::Ptr new_mappoint(new MapPoint);
    new_mappoint->id_ = factory_id++;
    return new_mappoint;
}

void MapPoint::RemoveObservation(std::shared_ptr<Feature> feature) {
    std::unique_lock<std::mutex> lck(data_mutex_);
    for (auto iter = observation_.begin(); iter != observation_.end(); iter++) {
        if (iter->lock() == feature) {
            observation_.erase(iter);
            feature->map_point_.reset(); // 将地图点指针清空
            observed_times_--;
            break;
        }
    }
}
} // namespace myslam