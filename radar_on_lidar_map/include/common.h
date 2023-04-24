//
// Created by myk on 2021/11/18.
//

#ifndef SRC_COMMON_H
#define SRC_COMMON_H
#include <cmath>

#include <queue>
#include <pcl/point_types.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "sophus/se3.hpp"
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

typedef pcl::PointXYZI PointType;

inline double rad2deg(double radians)
{
    return radians * 180.0 / M_PI;
}

inline double deg2rad(double degrees)
{
    return degrees * M_PI / 180.0;
}

struct tPose6D {
    double timestamp;
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
    tPose6D(double _time, double _x, double _y,
            double _z, double _roll, double _p, double _yaw)
            :timestamp(_time), x(_x), y(_y), z(_z),
            roll(_roll), pitch(_p), yaw(_yaw){}
    tPose6D(){}
};

tPose6D getOdom(nav_msgs::Odometry::ConstPtr _odom) {
    auto tt = _odom->header.stamp.toSec();
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w))
            .getRPY(roll, pitch, yaw);
    return tPose6D{tt, tx, ty, tz, roll, pitch, yaw};
} // getOdom


pcl::PointCloud<PointType>::Ptr
win2mid(const pcl::PointCloud<PointType>::Ptr &cloudIn, const tPose6D &tf, const tPose6D &refer) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur =
            pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    Eigen::Affine3f transRefer =
            pcl::getTransformation(refer.x, refer.y, refer.z, refer.roll, refer.pitch, refer.yaw);
    transRefer = transRefer.inverse();

    int numberOfCores = 8;
#pragma omp parallel for num_threads(numberOfCores) // todo to learn
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        PointType temp;

        temp.x = transCur(0, 0) * pointFrom.x +
                 transCur(0, 1) * pointFrom.y +
                 transCur(0, 2) * pointFrom.z + transCur(0, 3);
        temp.y = transCur(1, 0) * pointFrom.x +
                 transCur(1, 1) * pointFrom.y +
                 transCur(1, 2) * pointFrom.z + transCur(1, 3);
        temp.z = transCur(2, 0) * pointFrom.x +
                 transCur(2, 1) * pointFrom.y +
                 transCur(2, 2) * pointFrom.z + transCur(2, 3);

        cloudOut->points[i].x = transRefer(0, 0) * temp.x +
                                transRefer(0, 1) * temp.y +
                                transRefer(0, 2) * temp.z + transRefer(0, 3);
        cloudOut->points[i].y = transRefer(1, 0) * temp.x +
                                transRefer(1, 1) * temp.y +
                                transRefer(1, 2) * temp.z + transRefer(1, 3);
        cloudOut->points[i].z = transRefer(2, 0) * temp.x +
                                transRefer(2, 1) * temp.y +
                                transRefer(2, 2) * temp.z + transRefer(2, 3);


        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}


pcl::PointCloud<PointType>::Ptr
local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const tPose6D &tf) {
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur =
            pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);

    int numberOfCores = 16;
#pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i) {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0, 0) * pointFrom.x +
                                transCur(0, 1) * pointFrom.y +
                                transCur(0, 2) * pointFrom.z + transCur(0, 3);
        cloudOut->points[i].y = transCur(1, 0) * pointFrom.x +
                                transCur(1, 1) * pointFrom.y +
                                transCur(1, 2) * pointFrom.z + transCur(1, 3);
        cloudOut->points[i].z = transCur(2, 0) * pointFrom.x +
                                transCur(2, 1) * pointFrom.y +
                                transCur(2, 2) * pointFrom.z + transCur(2, 3);
        cloudOut->points[i].intensity = pointFrom.intensity;
    }

    return cloudOut;
}

Sophus::SE3d tPose6D2SE3(const tPose6D _pose) {
    Eigen::Matrix3d rotation_matrix1;
    rotation_matrix1 = Eigen::AngleAxisd(_pose.yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(_pose.pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(_pose.roll, Eigen::Vector3d::UnitX());
    Sophus::SE3d TEMP(rotation_matrix1, Eigen::Vector3d(_pose.x, _pose.y, _pose.z));
    return TEMP;
}

tPose6D SE32tPose6D(const Sophus::SE3d _se3) {
    Eigen::Quaterniond q(_se3.rotationMatrix());
    Eigen::Vector3d eulerAngle = q.matrix().eulerAngles(2, 1, 0);
    return tPose6D(0.0, _se3.translation().x(), _se3.translation().y(), _se3.translation().z(), eulerAngle[2],
                   eulerAngle[1], eulerAngle[0]);
}

#endif //SRC_COMMON_H
