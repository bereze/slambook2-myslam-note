#ifndef MYSLAM_ALGORITHM_H
#define MYSLAM_ALGORITHM_H

#include "myslam/common_include.h"

namespace myslam {

/**
 * @brief Linear triangulation with SVD
 * [math] p^ * P * X = 0
 *
 * 其中，
 * p = [u, v, 1]^T 为相机归一化坐标，即输入points；
 * P = K * T = K * [R|t] = [P1, P2, P3]^T_3x4 为相机的位姿，即输入poses；
 * X = [x, y, z, 1]^T 为地图点的3D坐标，即输出pt_world。
 *
 * p^ * P = |v * P3 - P2| =
 * A，事实上最后一行可由前两行得到，因此一个帧可以形成两个方程。 |u * P3 - P1|
 *          |u * P2 - P1|
 *
 * 对A进行SVD分解得 A = U * Σ * V^T， AV = UΣ，将 AX = 0 放入SVD分解中，
 * 即 A * [v1, ..., X] = U * [s1, ...,
 * 0]，因此SVD的V矩阵的最后一列即为所求的地图点的3D齐次坐标。
 *
 * 另外，由于噪声存在，A通常是满秩的，这使得假设0空间对应的X就不存在了，
 * 因此，在解SVD时，需要判断奇异值的s3 >> s4 是否成立，若不成立则三角化失败。
 *
 * @param poses poses
 * @param points points in normalized plane
 * @param pt_world triangulated point in the world
 * @return true if success
 */
inline bool triangulation(const std::vector<SE3> &poses,
                          const std::vector<Vec3> &points, Vec3 &pt_world) {
    MatXX A(2 * poses.size(), 4);
    VecX b(2 * poses.size());
    b.setZero();
    // 构造A矩阵
    for (size_t i = 0; i < poses.size(); ++i) {
        Mat34 m = poses[i].matrix().block<3, 4>(0, 0);
        // A.block<w, h>(x, y) 为矩阵A中从(x,y)开始的尺寸为(w,h)的子矩阵
        A.block<1, 4>(2 * i, 0) =
            points[i][0] * m.row(2) - m.row(0); // u * P3 - P1
        A.block<1, 4>(2 * i + 1, 0) =
            points[i][1] * m.row(2) - m.row(1); // v * P3 - P2
    }
    // 求解A的SVD分解
    auto svd = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);
    // V矩阵的最后一列即为所求，注意结果可能不是齐次的，需要齐次化
    pt_world = (svd.matrixV().col(3) / svd.matrixV()(3, 3)).head<3>();

    // 判定三角化是否成功
    if (svd.singularValues()[3] / svd.singularValues()[2] < 1e-2) {
        return true;
    }
    return false;
}

inline Vec2 toVec2(const cv::Point2f p) { return Vec2(p.x, p.y); }
} // namespace myslam

#endif