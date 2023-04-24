//
// Created by myk on 2021/12/30.
//

/**
 * 该例程将发布turtle1/cmd_vel话题，消息类型geometry_msgs::Twist
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

std::mutex radar_mutex;
std::string output_path = "/home/myk/dev/study/radar/output/zju-03/old_polar";
std::fstream polar_file;

void radar_local_callback(const sensor_msgs::PointCloud2ConstPtr &_radarCLoud) {

    pcl::PointCloud<pcl::PointXYZI> pcl_;
//    if (_radarCLoud->fields.size() == 4) {
        radar_mutex.lock();
        pcl::PointCloud<pcl::PointXYZI>::Ptr thisKeyFrame(
                new pcl::PointCloud<pcl::PointXYZI>());
        pcl::fromROSMsg(*_radarCLoud, *thisKeyFrame);

        pcl::PointCloud<pcl::PointXYZI>::Ptr save_pcl(new pcl::PointCloud<pcl::PointXYZI>);
        *save_pcl = *thisKeyFrame;
        pcl::VoxelGrid<pcl::PointXYZI> sor;
        sor.setLeafSize (0.1f, 0.1f, 0.1f);
        sor.setInputCloud (thisKeyFrame);
        sor.filter (*save_pcl);
        std::cout << "save " << save_pcl->size() << std::endl;
        radar_mutex.unlock();
//    }
}

void radarPolarCallback(sensor_msgs::Image _polar) {
    radar_mutex.lock();
    cv_bridge::CvImagePtr polar_ptr = cv_bridge::toCvCopy(_polar, sensor_msgs::image_encodings::TYPE_8UC1);
    cv::Mat polar_flip = polar_ptr->image;
//    int64_t time = ros::Time::now().toNSec();
    int64_t time = _polar.header.stamp.toNSec();
//    std::cout << "time: " << time << std::endl;
    static int timtim =0;
//    std::string file_name = std::to_string(time);
//    cv::imwrite(output_path + "/" + file_name + ".png", polar_flip);
    std::string file_name = std::to_string(timtim++);
    cv::imwrite(output_path + "/" + file_name + ".png", polar_flip);
    std::cout << "writing " << time << std::endl;
    polar_file << std::to_string(time) << std::endl;
    radar_mutex.unlock();
}


int main(int argc, char **argv)
{
	// ROS节点初始化
	ros::init(argc, argv, "pclfiltertest");

	// 创建节点句柄
	ros::NodeHandle n;
    polar_file.open(output_path+"/../polartime.txt", std::ios_base::out | std::ios_base::trunc);
	// 创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist，队列长度10
//    ros::Subscriber subRadarCloud = n.subscribe<sensor_msgs::PointCloud2>("/yeti_cloud_local", 100,
//                                                                           radar_local_callback);
    ros::Subscriber subRadarPolar = n.subscribe<sensor_msgs::Image>("/Navtech/Polar", 10000, radarPolarCallback);
    ros::spin();
	return 0;
}
