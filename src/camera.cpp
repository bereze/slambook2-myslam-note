#include "myslam/camera.h"

namespace myslam {

// todo: 这里pose_和Tcw不一样吗？
Vec3 Camera::world2camera(const Vec3 &p_w, const SE3 &Tcw) {
    return pose_ * Tcw * p_w; // 4x4 * 4x4 * 3x1
}

Vec3 Camera::camera2world(const Vec3 &p_c, const SE3 &Tcw) {
    return Tcw.inverse() * pose_inv_ * p_c;
}

Vec2 Camera::camera2pixel(const Vec3 &p_c) {
    return Vec2(fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
                fy_ * p_c(1, 0) / p_c(2, 0) + cy_);
}

Vec3 Camera::pixel2camera(const Vec2 &p_p, double depth) {
    return Vec3((p_p(0, 0) - cx_) * depth / fx_,
                (p_p(1, 0) - cy_) * depth / fy_, depth);
}

Vec2 Camera::world2pixel(const Vec3 &p_w, const SE3 &Tcw) {
    return camera2pixel(world2camera(p_w, Tcw));
}

Vec3 Camera::pixel2world(const Vec2 &p_p, const SE3 &Tcw, double depth) {
    return camera2world(pixel2camera(p_p, depth), Tcw);
}

} // namespace myslam
