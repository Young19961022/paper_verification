#ifndef __PCL_TOOLS_HPP__
#define __PCL_TOOLS_HPP__
#include <iostream>
#include <stdio.h>
#include <math.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Eigen>

#define min_f( a, b, c ) ( fminf( a, fminf( b, c ) ) )
#define max_f( a, b, c ) ( fmaxf( a, fmaxf( b, c ) ) )

namespace PCL_TOOLS
{
using namespace std;
struct Pt_compare {
    //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
    template < typename _T >
    inline bool operator()(const _T &a, const _T &b)
    {
        return ((a.x < b.x) || (a.x == b.x && a.y < b.y) || ((a.x == b.x) && (a.y == b.y) && (a.z < b.z)));
    }

    template < typename _T >
    bool operator()(const _T &a, const _T &b) const
    {
        return (a.x == b.x) && (a.y == b.y) && (a.z == b.z);
    }
};

struct Pt_hasher {
    template < typename _T >
    std::size_t operator()(const _T &k) const
    {
        return ((std::hash< float >()(k.x) ^ (std::hash< float >()(k.y) << 1)) >> 1) ^ (std::hash< float >()(k.z) << 1);
    }
};

struct Eigen_pt_compare {
    //inline bool operator()( const pcl::PointXYZ& a,  const pcl::PointXYZ & b)
    template < typename _T >
    inline bool operator()(const _T &a, const _T &b)
    {
        return ((a(0) < b(0)) || (a(0) == b(0) && a(1) < b(1)) || ((a(0) == b(0)) && (a(1) == b(1)) && (a(2) < b(2))));
    }

    template < typename _T >
    bool operator()(const _T &a, const _T &b) const
    {
        return (a(0) == b(0)) && (a(1) == b(1)) && (a(2) == b(2));
    }
};

struct Eigen_pt_hasher {
    template < typename _T >
    std::size_t operator()(const _T &k) const
    {
        return ((std::hash< float >()(k(0)) ^ (std::hash< float >()(k(1)) << 1)) >> 1) ^ (std::hash< float >()(k(2)) << 1);
    }
};

template < typename TT, typename PointType >
Eigen::Matrix< TT, 3, 1 > pcl_pt_to_eigen(const PointType &pt)
{
    return Eigen::Matrix< TT, 3, 1 >((TT) pt.x, (TT) pt.y, (TT) pt.z);
}

template < typename T >
Eigen::Matrix< float, 3, 1 > pcl_pt_to_eigenf(const T &pt)
{
    return pcl_pt_to_eigen<float>(pt);
}

template < typename T >
Eigen::Matrix< double, 3, 1 > pcl_pt_to_eigend(const T &pt)
{
    return pcl_pt_to_eigen<double>(pt);
}


template < typename TT, typename T >
TT eigen_to_pcl_pt(const T &pt)
{
    TT res_pt;
    res_pt.x = pt(0);
    res_pt.y = pt(1);
    res_pt.z = pt(2);
    return res_pt;
}

template < typename PointType, typename T >
pcl::PointCloud<PointType> eigen_pt_to_pcl_pointcloud(const vector<T> &eigen_pt_vec)
{
    pcl::PointCloud<PointType> pcl_pc_vec;
    pcl_pc_vec.resize(eigen_pt_vec.size());
    for (size_t i = 0; i < eigen_pt_vec.size() ; i++) {
        pcl_pc_vec[ i ] = eigen_to_pcl_pt< PointType >(eigen_pt_vec[ i ]);
    }
    return pcl_pc_vec;
}


template < typename T, typename PointType >
pcl::PointCloud< PointType > pointcloud_transfrom(pcl::PointCloud< PointType > pcl_pt_in, Eigen::Matrix< T, 3, 3 > tranfrom_R, Eigen::Matrix< T, 3, 1 > tranfrom_T)
{
    pcl::PointCloud< PointType > pcl_pc_res;
    pcl_pc_res.resize(pcl_pt_in.size());
    //cout << "Pointcloud_transfrom_T: \r\n " << tranfrom_T.transpose() << endl;
    //cout << "Pointcloud_transfrom_R: \r\n " << tranfrom_R << endl;
    for (size_t i = 0; i < pcl_pt_in.size(); i++) {
        auto eigen_pt = PCL_TOOLS::pcl_pt_to_eigen< T, PointType >(pcl_pt_in.points[ i ]);
        pcl_pc_res.points.push_back(PCL_TOOLS::eigen_to_pcl_pt< PointType>(tranfrom_R * eigen_pt + tranfrom_T));
    }
    return pcl_pc_res;
}

int save_PCLXYZI_to_txt(const std::string &filename, const pcl::PointCloud<pcl::PointXYZI>::ConstPtr pcl_pt_vec, int if_append = 0)
{
    std::fstream fos;
    if (if_append > 0) {
        fos.open(filename.c_str(), std::ios_base::app | std::ios_base::out);
    } else {
        fos.open(filename.c_str(), std::ios_base::out);
    }

    if (fos.is_open()) {
        for (size_t idx = 0; idx < pcl_pt_vec->size(); idx++) {
            fos << pcl_pt_vec->points[idx].x << ","
                << pcl_pt_vec->points[idx].y << ","
                << pcl_pt_vec->points[idx].z << ","
                << pcl_pt_vec->points[idx].intensity
                << std::endl;
        }
        fos.flush();
        fos.close();
        std::cout << "Save points to " << filename << " finish, points number =  " <<  pcl_pt_vec->size() << endl;
        return 0;
    } else {
        return -1;
    }
}

template < typename T >
int load_from_pcd_file(const char *file_name, pcl::PointCloud< T > &target_cloud)
{
    if (pcl::io::loadPCDFile< T >(file_name, target_cloud) == -1) {
        PCL_ERROR("Couldn't read file %s \n", file_name);
        return (0);
    } else {
        cout << "Get points number:" << target_cloud.size() << endl;
        return 1;
    }
};

void rgb2hsv(const unsigned char &src_r, const unsigned char &src_g, const unsigned char &src_b, unsigned char &dst_h, unsigned char &dst_s, unsigned char &dst_v)
{
    float r = src_r / 255.0f;
    float g = src_g / 255.0f;
    float b = src_b / 255.0f;

    float h, s, v; // h:0-360.0, s:0.0-1.0, v:0.0-1.0

    float max = max_f(r, g, b);
    float min = min_f(r, g, b);

    v = max;

    if (max == 0.0f) {
        s = 0;
        h = 0;
    } else if (max - min == 0.0f) {
        s = 0;
        h = 0;
    } else {
        s = (max - min) / max;

        if (max == r) {
            h = 60 * ((g - b) / (max - min)) + 0;
        } else if (max == g) {
            h = 60 * ((b - r) / (max - min)) + 120;
        } else {
            h = 60 * ((r - g) / (max - min)) + 240;
        }
    }

    if (h < 0)
        h += 360.0f;

    dst_h = (unsigned char)(h / 2);        // dst_h : 0-180
    dst_s = (unsigned char)(s * 255);      // dst_s : 0-255
    dst_v = (unsigned char)(v * 255);      // dst_v : 0-255
}

void hsv2rgb(const unsigned char &src_h, const unsigned char &src_s, const unsigned char &src_v, unsigned char &dst_r, unsigned char &dst_g, unsigned char &dst_b)
{
    // Online color picker: https://alloyui.com/examples/color-picker/hsv
    float h = src_h * 2.0f;   // 0-360
    float s = src_s / 255.0f; // 0.0-1.0
    float v = src_v / 255.0f; // 0.0-1.0

    float r = 0, g = 0, b = 0; // 0.0-1.0

    int   hi = (int)(h / 60.0f) % 6;
    float f = (h / 60.0f) - hi;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));

    switch (hi) {
    case 0:
        r = v, g = t, b = p;
        break;
    case 1:
        r = q, g = v, b = p;
        break;
    case 2:
        r = p, g = v, b = t;
        break;
    case 3:
        r = p, g = q, b = v;
        break;
    case 4:
        r = t, g = p, b = v;
        break;
    case 5:
        r = v, g = p, b = q;
        break;
    }
    //cout << "[" << r << " ," << g << " ," << b << " ]" ;
    dst_r = (unsigned char)(r * 255);      // dst_r : 0-255
    dst_g = (unsigned char)(g * 255);      // dst_r : 0-255
    dst_b = (unsigned char)(b * 255);      // dst_r : 0-255
    //cout << "[" << ( int ) dst_r << " ," << ( int ) dst_g << " ," << ( int ) dst_b << " ]" << endl;
}

pcl::PointCloud< pcl::PointXYZRGBA > PCL_XYZI_to_RGBA(pcl::PointCloud< pcl::PointXYZI > &pt_in, float alpha = 0.1)
{
    int                                  size = pt_in.size();
    float                                min_val = 3e8;
    float                                max_val = -3e8;
    pcl::PointCloud< pcl::PointXYZRGBA > pt_out;
    pt_out.resize(size);
    for (int i = 0; i < size; i++) {
        min_val = fminf(min_val, pt_in[ i ].intensity);
        max_val = fmaxf(max_val, pt_in[ i ].intensity);
        pt_out[ i ].x = pt_in[ i ].x;
        pt_out[ i ].y = pt_in[ i ].y;
        pt_out[ i ].z = pt_in[ i ].z;
        pt_out[ i ].a = alpha;
    }
    std::cout << "Input point size = " << size << std::endl;
    printf("Intensity min_max value =  [%.2f, %.2f] \r\n", min_val, max_val);
    unsigned char r = 0, g = 0, b = 0 ;
    float         h = 0, s = 0, v = 0 ;
    float         max_min_val_diff = max_val - min_val;
    s = 255.0;
    v = 255.0;
    for (int i = 0; i < size; i++) {
        r = g = b = 0;
        h = 240.0 * (pt_in[ i ].intensity - min_val) / max_min_val_diff;
        hsv2rgb(h, s, v, r, g, b);
        //cout << "[" << h << " ," << s << " ," << v << " ] ";
        //cout << "[" << ( int ) r << " ," << ( int ) g << " ," << ( int ) b << " ]" << endl;
        pt_out[ i ].r = r;
        pt_out[ i ].g = g;
        pt_out[ i ].b = b;
    }
    return pt_out;
}

template < typename T, typename PointType >
vector< Eigen::Matrix< T, 3, 1 > > pcl_pts_to_eigen_pts(const typename pcl::PointCloud< PointType >::Ptr input_cloud)
{
    vector< Eigen::Matrix< T, 3, 1 > > out_res;
    size_t                             pc_size = input_cloud->size();
    out_res.resize(pc_size);
    for (size_t i = 0; i < pc_size; i++) {
        //out_res[ i ] << input_cloud->points[ i ]x, input_cloud->points[ i ].y, input_cloud->points[ i ].z;
        out_res[i] << input_cloud->points[i].x, input_cloud->points[i].y, input_cloud->points[i].z;
    }
    return out_res;
}

struct cloud_point_index_idx {
    unsigned int idx;
    unsigned int cloud_point_index;

    cloud_point_index_idx(unsigned int idx_, unsigned int cloud_point_index_) : idx(idx_), cloud_point_index(cloud_point_index_) {}
    bool operator < (const cloud_point_index_idx &p) const
    {
        return (idx < p.idx);
    }
};

template<class PointT>
class VoxelGrid
{
private:
    Eigen::Vector4f leaf_size_;
    Eigen::Array4f inverse_leaf_size_;
    Eigen::Vector4i min_b_, max_b_, div_b_, divb_mul_;
    uint32_t min_points_per_voxel_;
    typename pcl::PointCloud<PointT>::Ptr input_;

public:
    VoxelGrid() :
        leaf_size_(Eigen::Vector4f::Zero()),
        inverse_leaf_size_(Eigen::Array4f::Zero()),
        min_b_(Eigen::Vector4i::Zero()),
        max_b_(Eigen::Vector4i::Zero()),
        div_b_(Eigen::Vector4i::Zero()),
        divb_mul_(Eigen::Vector4i::Zero()),
        min_points_per_voxel_(0)
    {}

    void setInputCloud(const typename pcl::PointCloud<PointT>::Ptr input)
    {
        input_ = input;
    }

    void setLeafSize(float lx, float ly, float lz)
    {
        leaf_size_[0] = lx;
        leaf_size_[1] = ly;
        leaf_size_[2] = lz;
        // Avoid division errors
        if (leaf_size_[3] == 0)
            leaf_size_[3] = 1;
        // Use multiplications instead of divisions
        inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
    }

    void getMinMax3D(const pcl::PointCloud<PointT> &cloud,
                     Eigen::Vector4f &min_pt, Eigen::Vector4f &max_pt)
    {
        Eigen::Array4f min_p, max_p;
        min_p.setConstant(FLT_MAX);
        max_p.setConstant(-FLT_MAX);

        // If the data is dense, we don't need to check for NaN
        if (cloud.is_dense) {
            for (size_t i = 0; i < cloud.size(); ++i) {
                pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
                min_p = min_p.min(pt);
                max_p = max_p.max(pt);
            }
        }
        // NaN or Inf values could exist => check for them
        else {
            for (size_t i = 0; i < cloud.size(); ++i) {
                // Check if the point is invalid
                if (!pcl_isfinite(cloud.points[i].x) ||
                    !pcl_isfinite(cloud.points[i].y) ||
                    !pcl_isfinite(cloud.points[i].z))
                    continue;
                pcl::Array4fMapConst pt = cloud.points[i].getArray4fMap();
                min_p = min_p.min(pt);
                max_p = max_p.max(pt);
            }
        }
        min_pt = min_p;
        max_pt = max_p;
    }

    void filter(pcl::PointCloud<PointT> &output)
    {
        // Has the input dataset been set already?
        if (!input_) {
            PCL_WARN("[myvoxelgrid] No input dataset given!\n");
            output.width = output.height = 0;
            output.points.clear();
            return;
        }

        output.height = 1;
        output.is_dense = true;

        Eigen::Vector4f min_p, max_p;
        getMinMax3D(*input_, min_p, max_p);

        // Compute the minimum and maximum bounding box values
        min_b_[0] = static_cast<int>(floor(min_p[0] * inverse_leaf_size_[0]));
        max_b_[0] = static_cast<int>(floor(max_p[0] * inverse_leaf_size_[0]));
        min_b_[1] = static_cast<int>(floor(min_p[1] * inverse_leaf_size_[1]));
        max_b_[1] = static_cast<int>(floor(max_p[1] * inverse_leaf_size_[1]));
        min_b_[2] = static_cast<int>(floor(min_p[2] * inverse_leaf_size_[2]));
        max_b_[2] = static_cast<int>(floor(max_p[2] * inverse_leaf_size_[2]));

        // Compute the number of divisions needed along all axis
        div_b_ = max_b_ - min_b_ + Eigen::Vector4i::Ones();
        div_b_[3] = 0;

        // Check that the leaf size is not too small, given the size of the data
        if ((div_b_[0] * div_b_[1] * div_b_[2]) > static_cast<int64_t>(std::numeric_limits<int32_t>::max())) {
            PCL_WARN("myvoxelgrid] Leaf size is too small for the input dataset. Integer indices would overflow.");
            output = *input_;
            return;
        }

        // Set up the division multiplier
        divb_mul_ = Eigen::Vector4i(1, div_b_[0], div_b_[0] * div_b_[1], 0);

        // Storage for mapping leaf and pointcloud indexes
        std::vector<cloud_point_index_idx> index_vector;
        index_vector.reserve(input_->size());

        for (size_t i = 0; i < input_->size(); i++) {
            if (!input_->is_dense)
                // Check if the point is invalid
                if (!pcl_isfinite(input_->points[i].x) ||
                    !pcl_isfinite(input_->points[i].y) ||
                    !pcl_isfinite(input_->points[i].z))
                    continue;
            // x轴上的体素id
            int ijk0 = static_cast<int>(floor(input_->points[i].x * inverse_leaf_size_[0]) - min_b_[0]);
            // y轴上的体素id
            int ijk1 = static_cast<int>(floor(input_->points[i].y * inverse_leaf_size_[1]) - min_b_[1]);
            // z轴上的体素id
            int ijk2 = static_cast<int>(floor(input_->points[i].z * inverse_leaf_size_[2]) - min_b_[2]);

            // Compute the centroid leaf index
            // 三维变一维
            int idx = ijk0 * divb_mul_[0] + ijk1 * divb_mul_[1] + ijk2 * divb_mul_[2];
            index_vector.push_back(cloud_point_index_idx(static_cast<unsigned int>(idx), i));
        }
        // 按照体素id，从小到大排序，体素id一样的点就被放在一起了
        std::sort(index_vector.begin(), index_vector.end(), std::less<cloud_point_index_idx>());

        unsigned int total = 0;
        unsigned int index = 0;
        std::vector<std::pair<unsigned int, unsigned int> > first_and_last_indices_vector;
        first_and_last_indices_vector.reserve(index_vector.size());

        while (index < index_vector.size()) {
            unsigned int i = index + 1;
            while (i < index_vector.size() && index_vector[i].idx == index_vector[index].idx)
                ++i;
            if (i - index >= min_points_per_voxel_) {
                ++total;
                first_and_last_indices_vector.push_back(std::pair<unsigned int, unsigned int> (index, i));
            }
            index = i;
        }
        output.points.resize(total);
        index = 0;
        for (unsigned int voxel_num = 0; voxel_num < first_and_last_indices_vector.size(); ++voxel_num) {
            // calculate centroid - sum values from all input points, that have the same idx value in index_vector array
            unsigned int first_index = first_and_last_indices_vector[voxel_num].first;
            unsigned int last_index = first_and_last_indices_vector[voxel_num].second;
            // 使用一个体素格子里的第一个点作为这个体素格子的代表
            output.points[index] = input_->points[index_vector[first_index].cloud_point_index];
            ++index;
        }
        output.width = static_cast<uint32_t>(output.points.size());
    }
};

} // namespace PCL_TOOLS
#endif
