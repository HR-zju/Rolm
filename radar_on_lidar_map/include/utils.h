//
// Created by myk on 2021/11/9.
//

#ifndef RADAR_LIDAR_SCANCONTEXT_UTILS_H
#define RADAR_LIDAR_SCANCONTEXT_UTILS_H

#include <dirent.h>
#include <fstream>
#include <sstream>
#include <iostream>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>//可视化
#include <pcl/visualization/cloud_viewer.h>
#include <queue>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>
#include <boost/bind.hpp>
#include <opencv/cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;

struct PointXYZIRT {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    int ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

std::vector<std::string> ouster_file_list_;// lidar
std::vector<string> radarpolar_file_list_;
std::string data_folder_path_ = "/media/myk/ubuntu18/bakup-18/dataset/MulRan/Kaist03";
float radar_resolution = 0.0432;// resolution of radar bins in meters per bin
int range_bins = 3360;          // CIR204 todo check
int N, H;

//radar
const int PCL_THREAD = 0; // 80
const int MAX_ANGLE = 140;
const bool radarimageprocessing = false;
const int gaussian_value = 5;
const int threshold_value = 62;
const int dilation_value = 2;
const bool grid = true;
int grid_width;
int grid_height;
int grid_stepSize_m = 1000 / (0.0432 * 2);
const int colormap = 10;
const double grid_opacity = 0.5;

// scancontex
const int PC_NUM_RING = 20;       // 20 in the original paper (IROS 18)
const int PC_NUM_SECTOR = 60;     // 60 in the original paper (IROS 18)
const double PC_MAX_RADIUS = 80.0;// 80 meter max in the original paper (IROS 18)
const double LIDAR_HEIGHT = 1.0;  // lidar height :
float rotation_angle = 0.0;       //rotation
const double SEARCH_RATIO = 1.0;  // for fast comparison, no Brute-force, but search 10 % is okay. // not was in the original conf paper, but improved ver.
int valiad_cols = 0;
// cart
const int Resolution = 200;

double search_time = 5;
std::vector<sensor_msgs::PointCloud2> lidar_candidate_pcl;

float xy2theta(const float &_x, const float &_y);

// scancontext
Eigen::MatrixXd pcl2Scancontex(const pcl::PointCloud<pcl::PointXYZI> _cloud);

void pcl2Scancontex(const pcl::PointCloud<pcl::PointXYZI> _cloud, Eigen::MatrixXd &scan_result);

Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc);

Eigen::MatrixXd circshift(Eigen::MatrixXd &_mat, int _num_shift);

int fastAlignUsingVkey(Eigen::MatrixXd &_vkey1, Eigen::MatrixXd &_vkey2);

double distDirectSC(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2);

std::pair<double, int> distanceBtnScanContext(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2);

std::pair<double, int> distanceBtnCartScanContext(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2);

void cloud2densMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud, cv::Mat &_des_map, const cv::Point2d _center);


#endif//RADAR_LIDAR_SCANCONTEXT_UTILS_H
