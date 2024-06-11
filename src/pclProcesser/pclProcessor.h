#include <iostream>
#include <thread>
#include <vector>

#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <pcl-1.13/pcl/point_types.h>
#include <pcl-1.13/pcl/io/pcd_io.h>
#include <pcl-1.13/pcl/search/search.h>
#include <pcl-1.13/pcl/search/kdtree.h>

#include <pcl-1.13/pcl/filters/extract_indices.h>

// #include<pcl-1.13/pcl/visualization/cloud_viewer.h>
#include <pcl-1.13/pcl/filters/filter_indices.h>
#include <pcl-1.13/pcl/segmentation/region_growing_rgb.h>

#include <pcl-1.13/pcl/features/moment_of_inertia_estimation.h>

#include <pcl/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>

#include <pcl-1.13/pcl/common/pca.h>
#include <pcl-1.13/pcl/common/transforms.h>
#include <pcl-1.13/pcl/common/impl/io.hpp>

#include <pcl-1.13/pcl/visualization/point_cloud_color_handlers.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl-1.13/pcl/sample_consensus/sac_model_line.h>
#include <pcl-1.13/pcl/sample_consensus/ransac.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <pcl-1.13/pcl/visualization/pcl_visualizer.h>


using namespace std::chrono_literals;
using namespace std;

void Growing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudd);

void color_based_RegionGrowing(std::vector<cv::Vec6f> Points);

void GrowingAndDetecting(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudd);

void GrowingNormalBase(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudd);

void houghLineDetection(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cloud);

std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> colorGrowing(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

void visualizeCloud(vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>);

void addBoundingBox(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb,int i);

void color_based_RegionGrowing(std::vector<cv::Vec6f> Points);

void CompressPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);

// void visualizePointCloud(const std::map<float, std::vector<Eigen::Vector4f>>& classifiedVectors);
