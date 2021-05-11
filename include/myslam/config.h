#pragma once
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * @brief 配置类，使用SetParameterFile确定配置文件
 * 然后用Get得到对应值
 * 单例模式
 */
class Config {
public:
    ~Config();

    // set a new config file
    static bool SetParameterFile(const std::string &filename);

    // access the parameters
    template <typename T> static T Get(const std::string &key) {
        return T(Config::config_->file_[key]);
    }

private:
    static std::shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {} // private构造函数实现单例模式
};
} // namespace myslam

#endif