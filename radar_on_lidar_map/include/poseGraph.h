//
// Created by myk on 2021/11/27.
//

#ifndef SRC_POSEGRAPH_H
#define SRC_POSEGRAPH_H

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "common.h"

/////////////////////////////////////////////////////
// gtsam
static int num_gt_constrain = 0;
bool gtSAMgraphMade = false;
gtsam::NonlinearFactorGraph gtSAMgraph;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;  // 优化结果
std::vector<tPose6D> keyframePosesUpdated;
std::mutex keyframePosesUpdated_mutex;

double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;
std::mutex mtxRecentPose;
tPose6D recentOptimizedPose6D;

// noise
gtsam::noiseModel::Diagonal::shared_ptr priorNoise;
gtsam::noiseModel::Diagonal::shared_ptr odomNoise;
gtsam::noiseModel::Diagonal::shared_ptr gpsNoise;
gtsam::noiseModel::Diagonal::shared_ptr mapNoise;
gtsam::noiseModel::Base::shared_ptr robustLoopNoise;
gtsam::noiseModel::Base::shared_ptr robustGPSNoise;

tPose6D odom_pose_prev{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init
tPose6D odom_pose_curr{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero


std::mutex poseGraph_mutex;

// calib
//tPose6D base2radar(0.0, 1.5, -0.04, 1.97, 0.0, 0.0, 0.9 / 180.0 * M_PI);
tPose6D base2radar(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9 / 180.0 * M_PI);
tPose6D base2lidar(0.0, 1.7042, -0.021, 1.8047, 0.0001 / 180.0 * M_PI, 0.0003 / 180.0 * M_PI, 179.6654 / 180.0 * M_PI);
Sophus::SE3d base2radar_T, base2lidar_T, lidar2radar_T;
Eigen::Matrix3d R_radar2enu = Eigen::Matrix3d::Zero();
std::vector<std::string> lidar_files;

std::vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>> gtfactor;
queue<bool> isgtfactorUseful;
queue<tPose6D> radar2lidar_Pose6D;
std::mutex gtfactor_mx, aff_mtx;

// gps factor
bool useGPS = false;
bool usegt = true;
sensor_msgs::NavSatFix::ConstPtr currGPS;
bool hasGPSforThisKF = false;
bool gpsOffsetInitialized = false;
double gpsAltitudeInitOffset = 0.0;


gtsam::Pose3 Pose6DtoGTSAMPose3(const tPose6D &p) {
    return gtsam::Pose3(gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw),
                        gtsam::Point3(p.x, p.y, p.z));
} // Pose6DtoGTSAMPose3

void initNoises(void) {
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = gtsam::noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
//    odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
//    odomNoiseVector6 << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
    odomNoise = gtsam::noiseModel::Diagonal::Variances(odomNoiseVector6);

    gtsam::Vector gpsNoiseVector3(3);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    gpsNoiseVector3 << 1e-20, 1e-20, 1e-20;
    gpsNoise = gtsam::noiseModel::Diagonal::Variances(gpsNoiseVector3);

    gtsam::Vector mapNoiseVector6(6);
//    mapNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2;
    mapNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    mapNoise = gtsam::noiseModel::Diagonal::Variances(mapNoiseVector6);

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(
            6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore,
            loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(
                    1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but
            // Cauchy is empirically good.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6));
    // gtsam 噪声模型

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop
    // clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3);  // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY,
            gpsAltitudeNoiseScore; // means only caring altitude here. (because
    // LOAM-like-methods tends to be asymptotically
    // flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(
                    1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but
            // Cauchy is empirically good.
            gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3));

} // initNoises

void updatePoses(void) {
    keyframePosesUpdated_mutex.lock();
    for (int node_idx = 0; node_idx < int(isamCurrentEstimate.size() - num_gt_constrain - 1 /*map 原点*/);
         node_idx++) {
        tPose6D &p = keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('o', node_idx)).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('o', node_idx)).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('o', node_idx)).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('o', node_idx)).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('o', node_idx)).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(gtsam::Symbol('o', node_idx)).rotation().yaw();
        if (node_idx == int(isamCurrentEstimate.size() - num_gt_constrain - 2)) {
            recentOptimizedX = p.x;
            recentOptimizedY = p.y;
            recentOptimizedPose6D = tPose6D(0.0, p.x, p.y, p.z, p.roll, p.pitch, p.yaw);
        }
    }
    cout << "gt constrain have " << num_gt_constrain << endl;
    keyframePosesUpdated_mutex.unlock();
} // updatePoses

//pose graph opt
void runISAM2opt(void) {
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();


    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
}
////////////////////////////////////////////////////////////////////////

#endif //SRC_POSEGRAPH_H
