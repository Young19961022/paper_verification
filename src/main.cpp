/*
 * @Author: scout
 * @Date: 2021-06-01 16:09:08
 * @LastEditTime: 2021-06-03 14:06:51
 * @LastEditors: Please set LastEditors
 * @Description: 验证林家荣先生的paper2里特征向量组成的旋转矩阵R的作用
 * @Paper2 link: https://arxiv.org/abs/1909.11811
 * @FilePath: /paper_solver/src/main.cpp
 */
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "cell_map.hpp"
#include "pcl_tools.hpp"
#include "pre_process.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCDReader reader;
    reader.read<pcl::PointXYZI> ("../pcd/test.pcd", *cloud);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> outlier_sor;
    outlier_sor.setInputCloud(cloud);
    outlier_sor.setMeanK(50);
    outlier_sor.setStddevMulThresh(0.8);
    outlier_sor.filter(*cloud);

    MyLaser laser_filter;
    laser_filter.preProcess(*cloud, *cloud_filtered);

    Points_cloud_map<float> pt_cell_map_full;
    std::set<Points_cloud_map<float>::Mapping_cell_ptr> cell_vec;

    pt_cell_map_full.append_cloud(
        PCL_TOOLS::pcl_pts_to_eigen_pts<float, pcl::PointXYZI>(cloud_filtered->makeShared()),
        &cell_vec);
    Maps_keyframe<float> keyframe_need_process;
    keyframe_need_process.add_cells(cell_vec);
    keyframe_need_process.update_features_of_each_cells(); // 计算cell的特征
    Eigen::Matrix3f rotate_matrix = keyframe_need_process.analyze(); // 计算直方图，并返回 平面特征的法向量组成的协方差矩阵，特征分解后的特征向量矩阵的逆
    cout << "rotate_matrix = \n" << rotate_matrix << endl;

    pcl::visualization::PCLVisualizer viewer("viewer");
    viewer.setBackgroundColor(0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZI>(cloud_filtered, "sample cloud");
    uint16_t idx = 0;
    float half_resolution = 0.25f;

    std::string line_name = "line";
    std::string plane_name = "plane";
    uint16_t counter = 0;
    for (const auto &current_cell : cell_vec) {
        counter ++;
        std::string cube = "cube" + std::to_string(idx++);

        Eigen::Vector3f center = current_cell->get_center();
        Eigen::Vector3f semi_feature_length = 0.6 * rotate_matrix * current_cell->m_feature_vector;
        if (counter >= 5) {
            pcl::PointXYZ p1(center(0), center(1), center(2));
            pcl::PointXYZ p2(center(0) + semi_feature_length(0), center(1) + semi_feature_length(1), center(2) + semi_feature_length(2));
            if (current_cell->m_feature_type == e_feature_line) {
                // 箭头的头 指向 p2
                // 线特征为红色
                viewer.addArrow(p2, p1, 1, 0, 0, false, line_name.append(std::to_string(idx)));
            } else if (current_cell->m_feature_type == e_feature_plane) {
                // 箭头的头 指向 p2
                // 平面特征为绿色
                viewer.addArrow(p2, p1, 0, 1, 0, false, plane_name.append(std::to_string(idx)));
            }
            counter = 0;
        }

        float x_min = center(0) - half_resolution;
        float x_max = center(0) + half_resolution;
        float y_min = center(1) - half_resolution;
        float y_max = center(1) + half_resolution;
        float z_min = center(2) - half_resolution;
        float z_max = center(2) + half_resolution;
        double r = 1.0, g = 0.5, b = 0.5;
        viewer.addCube(x_min, x_max, y_min, y_max, z_min, z_max, r, g, b, cube);
        // 设置cube的渲染参数
        viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, cube);
    }

    cv::Mat dispaly_line_img, dispaly_plane_img;
    cv::eigen2cv(keyframe_need_process.m_feature_img_line, dispaly_line_img);
    cv::eigen2cv(keyframe_need_process.m_feature_img_plane, dispaly_plane_img);

    cv::namedWindow("dispaly_line_img", cv::WINDOW_GUI_NORMAL);
    cv::namedWindow("dispaly_plane_img", cv::WINDOW_GUI_NORMAL);
    cv::imshow("dispaly_line_img", dispaly_line_img);
    cv::imshow("dispaly_plane_img", dispaly_plane_img);

    // 设置点云显示的尺寸大小
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    // 添加坐标系，尺寸为1.0，X (红色)、Y(绿色)、Z (蓝色)
    viewer.addCoordinateSystem(1.0);
    // 设置相机参数使用户从设置的角度和方向观察点云
    viewer.setCameraPosition(-21.969, 5.02249, 21.8913, 0.547659, -0.0585467, 0.834651);

    while (!viewer.wasStopped()) {
        cv::waitKey(1);
        viewer.spinOnce(100); // 回调函数。允许鼠标、键盘等交互操作，才能正常显示点云
    }

    return 0;
}