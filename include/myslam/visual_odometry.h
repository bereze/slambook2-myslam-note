#ifndef MYSLAM_VISION_ODOMETRY_H
#define MYSLAM_VISION_ODOMETRY_H

#include "myslam/backend.h"
#include "myslam/common_include.h"
#include "myslam/dataset.h"
#include "myslam/frontend.h"
#include "myslam/viewer.h"

namespace myslam {
/**
 * @brief VO 对外接口
 *
 */
class VisualOdometry {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<VisualOdometry> Ptr;

    // 根据配置文件进行实例化
    VisualOdometry(std::string &config_path);

    // 初始化
    bool Init();

    // 启动VO
    void Run();

    // 前进
    bool Step();

    // 获取前端状态
    FrontendStatus GetFrontendStatus() const { return frontend_->GetStatus(); }

private:
    bool inited_ = false;
    std::string config_file_path_;

    Frontend::Ptr frontend_ = nullptr;
    Backend::Ptr backend_ = nullptr;
    Map::Ptr map_ = nullptr;
    Viewer::Ptr viewer_ = nullptr;

    Dataset::Ptr dataset_ = nullptr;
};
} // namespace myslam

#endif