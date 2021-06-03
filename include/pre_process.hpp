/*
 * @Author: your name
 * @Date: 2021-06-02 10:14:00
 * @LastEditTime: 2021-06-03 11:16:43
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /paper_solver/include/pre_process.hpp
 */

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class MyLaser
{
private:
    template <typename T>
    T depth2_xyz(T x, T y, T z)
    {
        return x * x + y * y + z * z;
    }

public:
    template <typename T>
    void preProcess(pcl::PointCloud<T> &laser_zvision_in, pcl::PointCloud<T> &laser_zvision_out)
    {
        uint16_t pts_size = laser_zvision_in.size();
        laser_zvision_out.clear();
        laser_zvision_out.resize(pts_size);

        for (uint16_t idx = 0; idx < pts_size; idx++) {
            if (!std::isfinite(laser_zvision_in.points[idx].x) ||
                !std::isfinite(laser_zvision_in.points[idx].y) ||
                !std::isfinite(laser_zvision_in.points[idx].z)) {
                continue;
            }

            if (laser_zvision_in.points[idx].x == 0) continue;

            float depth_sq2 = depth2_xyz(laser_zvision_in.points[idx].x,
                                         laser_zvision_in.points[idx].y,
                                         laser_zvision_in.points[idx].z);

            if (depth_sq2 < min_allow_dis_ * min_allow_dis_) continue;
            if ((laser_zvision_in.points[idx].intensity / depth_sq2) < min_sigma_) continue;
            laser_zvision_out.points[pass_pc_size_++] = laser_zvision_in.points[idx];
        }
        laser_zvision_out.resize(pass_pc_size_);
        laser_zvision_out.height = 1;
        laser_zvision_out.width = pass_pc_size_;
        laser_zvision_out.is_dense = true;
    }

private:
    float min_allow_dis_ = 2.0f;
    float min_sigma_ = 7e-3;
    uint16_t pass_pc_size_ = 0u;
};
