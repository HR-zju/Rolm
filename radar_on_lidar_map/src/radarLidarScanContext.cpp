#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/NavSatFix.h>
#include <thread>
#include <deque>
#include <memory>

//#include "sophus/se3.hpp"

// gtsam
#include "utils.h"
#include "common.h"
#include "poseGraph.h"
#include "fileplayer.h"

#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

// check
#include <pcl/io/pcd_io.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace std;
using namespace pcl;

// param
int num_keyFrame;  // must be even number
string data_path;
string output_path;
bool generate_map;
bool is_baseKey = true;  // base keyframe without Top twenty
bool is_midKey = false;  //
std::pair<string, pcl::PointCloud<PointXYZIRT>> lidar_org;
std::pair<string, cv::Mat> radar_org;
std::mutex radar_mutex, lidar_mutex, gps_mutex, odom_mutex, win_mutex, radarkeyset_mutex, radarWin_mutex, radarMap_mtx, Icp_mtx;
std::deque<sensor_msgs::PointCloud2> radarBuf;
std::queue<sensor_msgs::PointCloud2> lidarBuf;
std::queue<sensor_msgs::NavSatFix::ConstPtr> gpsBuf;
std::queue<nav_msgs::Odometry::Ptr> odometryBuf;
std::vector<pair<pcl::PointCloud<PointType>, tPose6D>> radarWindow;

std::queue<pair<double, pcl::PointCloud<PointType>>> radarKeySet;
std::queue<pair<pcl::PointCloud<PointType>, tPose6D>> radarWinSet;  // old = radarKeySet
std::queue<pair<pcl::PointCloud<PointType>, std::vector<int>>> IcpradarWinSet;  // old = radarKeySet
std::queue<bool> isradarWinSetUseful;
std::vector<pcl::PointCloud<PointType>::Ptr> radarMapList;
pcl::VoxelGrid<PointType> downSizeFilterMapROLM;
pcl::VoxelGrid<PointType> downSizeLidarMap;
pcl::PointCloud<PointType>::Ptr radarMap(new pcl::PointCloud<PointType>());

//ROS
ros::Publisher pubRadarKeyMap;
ros::Publisher pubLidarKey;
ros::Publisher pubOdomAftROLM, pubPathAftROLM, pubMapAftROLM;

//std::fstream gt_file;
//std::fstream align_file;
static int radar_index = 0;
std::vector<int> key_radar_index;
double timeOdom = 0;

std::vector<tPose6D> keyframePoses;
std::mutex keyframePoses_mutex;
pcl::VoxelGrid<PointType> downSizeFilterRadar;  //

// map groundtruth
template<typename T>
using EigenVecotr = std::vector<T, Eigen::aligned_allocator<T>>;



pcl::KdTreeFLANN<pcl::PointXY> groud_truth_tree;
pcl::KdTreeFLANN<pcl::PointXYZ> groud_truth_tree_xyz;
EigenVecotr<pair<double, Sophus::SE3d>> groud_truth_vector;


// align
const int num_thrown_front = 0;  // must be Even number  // 之前 30
static int count_radar_frame = 0;
const int numnode_bt_kyframe = 1;  // 把这改小
const int numKeyframeInWin = 20;
queue<bool> que_odom;
std::mutex que_odom_mtx;
pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
//queue<pair<pcl::PointCloud<PointType>::Ptr, pcl::PointCloud<PointType>::Ptr>>;
//EigenVecotr<Eigen::MatrixXd> lidarMapVector;
//Eigen::Affine3f correctionLidarFrame;

// current local lidar cloud
void radar_local_callback(const sensor_msgs::PointCloud2ConstPtr &_radarCLoud) {

    pcl::PointCloud<PointType> pcl_;
    if (_radarCLoud->fields.size() == 4) {
        radar_mutex.lock();
        radarBuf.push_back(*_radarCLoud);
        radar_mutex.unlock();
    }
}

// current local lidar cloud
void lidar_callback(const sensor_msgs::PointCloud2ConstPtr &_lidarCloud) {

    pcl::PointCloud<PointType> pcl_radar;
    pcl::fromROSMsg(*_lidarCloud, pcl_radar);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()));//同理，UnitX()，绕x轴；UnitY()，绕y轴
    transform.rotate(Eigen::AngleAxisf(-M_PI, Eigen::Vector3f::UnitX()));//同理，UnitX()，绕x轴；UnitY()，绕y轴 // todo 左手bug

    pcl::transformPointCloud(pcl_radar, pcl_radar, transform);

    sensor_msgs::PointCloud2 lidar_temp;
    pcl::toROSMsg(pcl_radar, lidar_temp);
    lidar_temp.header.frame_id = "/camera_init";
    lidar_temp.header.stamp = _lidarCloud->header.stamp;

    lidar_mutex.lock();
    lidarBuf.push(lidar_temp);
    lidar_mutex.unlock();
}

// laserOdometryHandler
void Odometry_callback(const nav_msgs::Odometry::ConstPtr &_laserOdometry) {
    odom_mutex.lock();
    nav_msgs::Odometry aa = *_laserOdometry;
//    aa.pose.pose.position.y = - aa.pose.pose.position.y;  // todo 左手bug
    nav_msgs::Odometry::Ptr _pus(new nav_msgs::Odometry);
    *_pus = aa;
    odometryBuf.push(_pus);
//    cout << "odometryBuf" << odometryBuf.size() << endl;
    odom_mutex.unlock();
} //

void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &_gps) {
    if (useGPS) {
        gps_mutex.lock();
        gpsBuf.push(_gps);
        gps_mutex.unlock();
    }
} // gpsHandler


/**
 * 1. 存radar对应里程计 点云+odom ->radarWindow
 * 2. pose graph
 * */
void process_data() {
    while (1) {
        while (!radarBuf.empty() && !odometryBuf.empty()) {
            // dismiss num_thrown_front in front
            static bool begin_flag = false;
            if (!begin_flag) {
                if (radarBuf.size() < num_thrown_front) continue;
                else {
                    radar_mutex.lock();
                    radarBuf.clear();
                    radar_mutex.unlock();
                    begin_flag = true;
                    continue;
                }
            }

            // appropriate odom
            while (!odometryBuf.empty() &&
                   odometryBuf.front()->header.stamp.toSec() <
                   radarBuf.front().header.stamp.toSec()) {
                odom_mutex.lock();
                odometryBuf.pop();
                odom_mutex.unlock();
            }
//            cout << "find appropriate odom" << endl;
            if (odometryBuf.empty()) {
                odom_mutex.unlock();
                cout << "no enough odom" << endl;
                break;
            }

            timeOdom = odometryBuf.front()->header.stamp.toSec();

            // perframe
            tPose6D pose_curr = getOdom(odometryBuf.front());
            pose_curr.timestamp = timeOdom;
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(
                    new pcl::PointCloud<PointType>());

            radar_mutex.lock();
            pcl::fromROSMsg(radarBuf.front(), *thisKeyFrame);
            radarBuf.pop_front();
            radar_mutex.unlock();

            thisKeyFrame->width=thisKeyFrame->size();
            thisKeyFrame->height = 1;

            radarWin_mutex.lock();
            radarWindow.push_back(make_pair(*thisKeyFrame, pose_curr));
            radarWin_mutex.unlock();

            odom_mutex.lock();
            odometryBuf.pop();
            odom_mutex.unlock();

            // later
            count_radar_frame++;
            if (count_radar_frame == numnode_bt_kyframe) {

                count_radar_frame = 0;
                keyframePoses_mutex.lock();
                keyframePoses.push_back(pose_curr);
                keyframePoses_mutex.unlock();
                keyframePosesUpdated_mutex.lock();
                keyframePosesUpdated.push_back(pose_curr); // init
                keyframePosesUpdated_mutex.unlock();

                radarMap_mtx.lock();
                radarMapList.push_back(thisKeyFrame);
                radarMap_mtx.unlock();

                ////// gps factor
                double eps = 0.5;
                while (!gpsBuf.empty()) {
                    auto thisGPS = gpsBuf.front();
                    auto thisGPSTime = thisGPS->header.stamp.toSec();
                    if (abs(thisGPSTime - timeOdom) < eps) {
                        currGPS = thisGPS;
                        hasGPSforThisKF = true;
                        break;
                    } else {
                        hasGPSforThisKF = false;
                    }
                    gps_mutex.lock();
                    gpsBuf.pop();
                    gps_mutex.unlock();
                }
                if (!gpsOffsetInitialized) {  // 如果没有定初始GPS
                    if (hasGPSforThisKF) { // if the very first frame  且当找到当前odem最近的GPS
                        gpsAltitudeInitOffset = currGPS->altitude;  // 高度初始化 ... 有啥用么
                        cout << "currGPS->altitude " << currGPS->altitude << endl;
                        gpsOffsetInitialized = true;
                    }
                }
                ///// gps up
                que_odom_mtx.lock();
                que_odom.push(hasGPSforThisKF);
                que_odom_mtx.unlock();
            }

            //// radarkeywindow
            static bool startkeywindow = false;  // align
            if (radarWindow.size() == (num_keyFrame) && !startkeywindow) {
                startkeywindow = true;
                radarWin_mutex.lock();
                radarWindow.clear();
                radarWin_mutex.unlock();
            }
            double max_delta_yaw = 0.0;
            if (radarWindow.size() == num_keyFrame && startkeywindow) {
                radarWin_mutex.lock();
                pcl::PointCloud<PointType>::Ptr
                        windowCloudMap(new pcl::PointCloud<PointType>());
                windowCloudMap->clear();
                for (int j = 0; j < num_keyFrame; ++j) {
                    if (j == num_keyFrame / 2)
                        continue;
                    pcl::PointCloud<PointType>::Ptr
                            temp_cloud(new pcl::PointCloud<PointType>());
                    temp_cloud->points = radarWindow[j].first.points;
                    *windowCloudMap += *win2mid(temp_cloud, radarWindow[j].second,
                                                radarWindow[num_keyFrame / 2].second);
                    double curr_delta_yaw = abs(radarWindow[j].second.yaw - radarWindow[0].second.yaw);
                    if (curr_delta_yaw > max_delta_yaw)
                        max_delta_yaw = curr_delta_yaw;
                }

                // update pose
                tPose6D curr_pose_updated = radarWindow[num_keyFrame / 2].second;

                radarkeyset_mutex.lock();
                radarWinSet.push(
                        make_pair(*windowCloudMap, curr_pose_updated));// radarWindow[num_keyFrame / 2].second));
                if (max_delta_yaw < M_PI / 6.0) isradarWinSetUseful.push(true);
                else {isradarWinSetUseful.push(false);
                cout << "弯头过大"<< endl;}
//                isradarWinSetUseful.push(true);
                radarkeyset_mutex.unlock();
                radarWindow.clear();
                radarWin_mutex.unlock();

                startkeywindow = false;
            }
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


/**
 * posegraph节点添加和优化线程
 **/
void process_posegraph() {
  ros::Rate rate(40);
    while (ros::ok()) {
      rate.sleep();
        while (!keyframePoses.empty() && !que_odom.empty() || !gtfactor.empty() && !radar2lidar_Pose6D.empty()) {
            if (!keyframePoses.empty() && !que_odom.empty()) { // todo 这个gt删掉？  && gtfactor.empty()
                que_odom_mtx.lock();
                bool hasGPSforThisKF = que_odom.front();
                que_odom.pop();
                que_odom_mtx.unlock();

                //// pose graph
                static int odom_between_node = -1;
                odom_between_node++;
                static Sophus::SE3d per_prior;
                if (!gtSAMgraphMade /* prior node*/) {
                    const int init_node_idx = 0;
                    ///
                    gtsam::Pose3 map_poseOrigin = Pose6DtoGTSAMPose3(SE32tPose6D(groud_truth_vector.at(init_node_idx).second));
                    gtsam::Pose3 poseFrom =  gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));
                    gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                    {
                        gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('m', init_node_idx), map_poseOrigin,
                                                                        priorNoise));
                        initialEstimate.insert(gtsam::Symbol('m', init_node_idx), map_poseOrigin);

                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                gtsam::Symbol('m', init_node_idx), gtsam::Symbol('o', init_node_idx),
                                poseFrom.between(poseTo),
                                odomNoise));
                        initialEstimate.insert(gtsam::Symbol('o', init_node_idx), poseTo);
                        runISAM2opt();

                    }
                    poseGraph_mutex.unlock();
                    gtSAMgraphMade = true;
                } else {
//                    const int prev_node_idx = keyframePoses.size() - 2;
//                    const int curr_node_idx = keyframePoses.size() - 1;
//                    static int key_radar_factor_idx = 0;
//                    // becuase cpp starts with 0 (actually this index could be any
//                    // number, but for simple implementation, we follow sequential
//                    // indexing)
//                    gtsam::Pose3 poseFrom =
//                            Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx)); //线性分类器
//                    gtsam::Pose3 poseTo =
//                            Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));
//
//
//                    Sophus::SE3d poseFrom_T = tPose6D2SE3(keyframePoses.at(prev_node_idx)); //recentOptimizedPose6D
//                    Sophus::SE3d poseTo_T = tPose6D2SE3(keyframePoses.at(curr_node_idx));
//                    keyframePosesUpdated_mutex.lock();
//                    Sophus::SE3d es_T = tPose6D2SE3(recentOptimizedPose6D) * poseFrom_T.inverse() * poseTo_T;
//                    keyframePosesUpdated_mutex.unlock();
//                    gtsam::Pose3 es_pose = Pose6DtoGTSAMPose3(SE32tPose6D(es_T));
//                    if (abs(poseFrom.between(poseTo).x()) > 10 || abs(poseFrom.between(poseTo).y()) > 10){
//                        poseTo = poseFrom;
//                    }
//                    poseGraph_mutex.lock();
//                    {
//                        // odom factor
//                        gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
//                                gtsam::Symbol('o', prev_node_idx), gtsam::Symbol('o', curr_node_idx),
//                                poseFrom.between(poseTo),
//                                odomNoise));
//                        if (hasGPSforThisKF) {
//                            double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
//                            mtxRecentPose.lock();
//                            gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY,
//                                                        curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set)
//                            mtxRecentPose.unlock();
//                            gtSAMgraph.add(
//                                    gtsam::GPSFactor(gtsam::Symbol('o', curr_node_idx), gpsConstraint,
//                                                     robustGPSNoise));  // robustGPSNoise
//                            cout << "GPS factor added at node " << curr_node_idx << endl;
//                        }
//                        initialEstimate.insert(gtsam::Symbol('o', curr_node_idx), es_pose);
//                        runISAM2opt();
//                    }
//                    poseGraph_mutex.unlock();
//
//                    if (curr_node_idx % 100 == 0)
//                        cout << "posegraph odom node " << curr_node_idx << " added." << endl;

                    static int count_odom_factors = 1;
//
//                    const int prev_node_idx = keyframePoses.size() - 2;
//                    const int curr_node_idx = keyframePoses.size() - 1;
                    while (keyframePoses.size() > count_odom_factors) {
                        const int prev_node_idx = count_odom_factors - 1;
                        const int curr_node_idx = count_odom_factors++;

                        gtsam::Pose3 poseFrom =
                                Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx)); //线性分类器
                        gtsam::Pose3 poseTo =
                                Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));


                        Sophus::SE3d poseFrom_T = tPose6D2SE3(keyframePoses.at(prev_node_idx)); //recentOptimizedPose6D
                        Sophus::SE3d poseTo_T = tPose6D2SE3(keyframePoses.at(curr_node_idx));
                        keyframePosesUpdated_mutex.lock();
                        Sophus::SE3d es_T = tPose6D2SE3(recentOptimizedPose6D) * poseFrom_T.inverse() * poseTo_T;
                        keyframePosesUpdated_mutex.unlock();
                        gtsam::Pose3 es_pose = Pose6DtoGTSAMPose3(SE32tPose6D(es_T));
                        if (abs(poseFrom.between(poseTo).x()) > 30 || abs(poseFrom.between(poseTo).y()) > 30) {
                            poseTo = poseFrom;
                        }
                        poseGraph_mutex.lock();
                        {
                            // odom factor
                            gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                                    gtsam::Symbol('o', prev_node_idx), gtsam::Symbol('o', curr_node_idx),
                                    poseFrom.between(poseTo),
                                    odomNoise));
                            if (hasGPSforThisKF) {
                                double curr_altitude_offseted = currGPS->altitude - gpsAltitudeInitOffset;
                                mtxRecentPose.lock();
                                gtsam::Point3 gpsConstraint(recentOptimizedX, recentOptimizedY,
                                                            curr_altitude_offseted); // in this example, only adjusting altitude (for x and y, very big noises are set)
                                mtxRecentPose.unlock();
                                gtSAMgraph.add(
                                        gtsam::GPSFactor(gtsam::Symbol('o', curr_node_idx), gpsConstraint,
                                                         robustGPSNoise));  // robustGPSNoise
                                cout << "GPS factor added at node " << curr_node_idx << endl;
                            }
//                            cout << "add odom node: " << curr_node_idx << endl;
                            initialEstimate.insert(gtsam::Symbol('o', curr_node_idx), es_pose);
                            runISAM2opt();
                        }
                        poseGraph_mutex.unlock();

                        if (curr_node_idx % 100 == 0)
                            cout << "posegraph odom node " << curr_node_idx << " added." << endl;
                    }
                }
            }

//            else if (usegt &&
//                     keyframePoses.size() > (numKeyframeInWin / numnode_bt_kyframe) * (num_gt_constrain)) {
            else if(usegt){
                // posegraph constrain  todo 是否可以新开线程?
                Sophus::SE3d curr_gt = gtfactor.front();
                bool isgtuse = isgtfactorUseful.front();

                // pub gt tf to check
                tPose6D checkgt = SE32tPose6D(curr_gt);
                nav_msgs::Odometry gt_tf;
                gt_tf.header.frame_id = "/camera_init";
                gt_tf.child_frame_id = "/aft_pgo";
                gt_tf.header.stamp =
                        ros::Time::now();

                gt_tf.pose.pose.position.x = checkgt.x;
                gt_tf.pose.pose.position.y = checkgt.y;
                gt_tf.pose.pose.position.z = checkgt.z;
                gt_tf.pose.pose.orientation =
                    tf::createQuaternionMsgFromRollPitchYaw(
                        checkgt.roll, checkgt.pitch,checkgt.yaw);

                pubOdomAftROLM.publish(gt_tf);

                gtfactor_mx.lock();
//                gtfactor.pop();
                vector<Sophus::SE3d, Eigen::aligned_allocator<Sophus::SE3d>>(gtfactor.begin() + 1, gtfactor.end()).swap(gtfactor);
                isgtfactorUseful.pop();
                gtfactor_mx.unlock();

                tPose6D radarOnLidarMap;
                aff_mtx.lock();
                radarOnLidarMap = radar2lidar_Pose6D.front();
                radar2lidar_Pose6D.pop();
                aff_mtx.unlock();

                gtsam::Pose3 gtpose =
                        Pose6DtoGTSAMPose3(SE32tPose6D(curr_gt));
                gtsam::Pose3 poseFrom01 =  // curr to target
                        Pose6DtoGTSAMPose3(radarOnLidarMap);
                gtsam::Pose3 poseFrom02 =  // curr to target
                        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
                                      gtsam::Point3(0.0, 0.0, 0.0));
                gtsam::Pose3 poseTo =  // 0?
                        gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
                                      gtsam::Point3(0.0, 0.0, 0.0));

                // 如果真值与里程计偏航角相差30以上 不优化 todo
                keyframePosesUpdated_mutex.lock();
                tPose6D odom_factor = keyframePosesUpdated[
                        (num_gt_constrain + 1) * (numKeyframeInWin / numnode_bt_kyframe) - 6]; //2
                keyframePosesUpdated_mutex.unlock();
                gtsam::Pose3 odompose = Pose6DtoGTSAMPose3(odom_factor);
                cout << "odompose.between(gtpose).x() " << odompose.between(gtpose).x() << endl;
                cout << "odompose.between(gtpose).y() " << odompose.between(gtpose).y() << endl;
                if (abs(odompose.between(gtpose).x())>10)  {  isgtuse = false;
                cout << "倒退太多1" << endl;}  // DCC2 增加
                if (abs(odompose.between(gtpose).y())>10)  { isgtuse = false;
                    cout << "倒退太多2" << endl;}  // DCC2 增加

                // 掉头或者钝角转弯不要56
                static Sophus::SE3d last_constrain;
                static Sophus::SE3d curr_constrain;
                static bool init_turn_around = false;
                if (!init_turn_around){
                  last_constrain = curr_gt * tPose6D2SE3(radarOnLidarMap).inverse();
                  init_turn_around = true;
                }
                else{
                  curr_constrain = curr_gt * tPose6D2SE3(radarOnLidarMap).inverse();
                  Sophus::SE3d delta_tran = last_constrain.inverse() * curr_constrain;
                  cout << "yaw " << SE32tPose6D(delta_tran).yaw <<endl;
                  cout << "pitch " << SE32tPose6D(delta_tran).pitch <<endl;
                  cout << "roll " << SE32tPose6D(delta_tran).roll <<endl;

//                  if(abs(SE32tPose6D(delta_tran).roll)>M_PI*0.5){
//                    if(abs(SE32tPose6D(delta_tran).yaw)<M_PI*0.3333){
//                        isgtuse = false;
//                        cout << "突然掉头1" << endl;
//                    }
//                  }
//                  else{
//                    if (abs(SE32tPose6D(delta_tran).yaw)>M_PI*0.6667){
//                        isgtuse = false;
//                        cout << "突然掉头2" << endl;
//                    }
//                  }

                  last_constrain = curr_constrain;
                }

//                recentOptimizedPose6D
                cout << "====" << endl;

                Sophus::SE3d yaw_change = tPose6D2SE3(recentOptimizedPose6D).inverse() * curr_constrain;
                cout << "yaw " << SE32tPose6D(yaw_change).yaw <<endl;
                cout << "pitch " << SE32tPose6D(yaw_change).pitch <<endl;
                cout << "roll " << SE32tPose6D(yaw_change).roll <<endl;
                if(abs(SE32tPose6D(yaw_change).roll)>M_PI*0.5){
                    if(abs(SE32tPose6D(yaw_change).yaw)<M_PI*0.3333){
                        isgtuse = false;
                        cout << "优化掉头1" << endl;
                    }
                }
                else{
                    if (abs(SE32tPose6D(yaw_change).yaw)>M_PI*0.6667){
                        isgtuse = false;
                        cout << "优化掉头2" << endl;
                    }
                }

                num_gt_constrain++;
                poseGraph_mutex.lock();
                {
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(
                            gtsam::Symbol('g', (num_gt_constrain - 1)),
                            gtsam::Symbol('o', num_gt_constrain * (numKeyframeInWin / numnode_bt_kyframe) - 6),  //2
                            poseFrom01.between(poseTo),
                            mapNoise));
                    static int count = 1;
                    cout << "id: " << count++<<endl;
                    cout << " posegraph " << radarOnLidarMap.yaw << endl;
                    cout << "isuseful " << isgtuse << endl;
                    if (isgtuse)
                        gtSAMgraph.add(
                                gtsam::PriorFactor<gtsam::Pose3>(gtsam::Symbol('g', (num_gt_constrain - 1)), gtpose,
                                                                 priorNoise));
                    initialEstimate.insert(gtsam::Symbol('g', (num_gt_constrain - 1)), gtpose);
                    runISAM2opt();
                }
                poseGraph_mutex.unlock();
            }
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }
    }
}

// 匹配地图点,进行重定位
void process_localization() {
    ros::Rate rate(20);
    while (ros::ok()) {
      rate.sleep();
        while (!radarWinSet.empty()) {
            pair<pcl::PointCloud<PointType>, tPose6D> curr_radar_key = radarWinSet.front();
            bool isradaruseful = isradarWinSetUseful.front();

            radarkeyset_mutex.lock();
            radarWinSet.pop();
            isradarWinSetUseful.pop();
            radarkeyset_mutex.unlock();

            // find related ground truth
            std::vector<int> point_search_id;
            std::vector<float> point_search_sq_dis;
            pcl::PointXYZ pose_point;

            // update pose
            tPose6D radar_key_aft_rolm;
            keyframePosesUpdated_mutex.lock();

            radar_key_aft_rolm = keyframePosesUpdated[(num_gt_constrain + 1) * (numKeyframeInWin / numnode_bt_kyframe) -
                                                      6]; //2
            keyframePosesUpdated_mutex.unlock();
            pose_point.x = radar_key_aft_rolm.x;
            pose_point.y = radar_key_aft_rolm.y;
            pose_point.z = radar_key_aft_rolm.z;

            groud_truth_tree_xyz.nearestKSearch(pose_point, 50, point_search_id, point_search_sq_dis);

            ifstream file;
            Eigen::MatrixXd radarContext(20,60);
            pcl2Scancontex(curr_radar_key.first, radarContext);
            std::pair<double, int> candi_scancontext_result = make_pair(10000.0, 0);

            int final_gt_idx = 0;
            double similar_lidar_timestamp = 0;
            for (auto candiLidar_idx:point_search_id) {
                if(candiLidar_idx >= lidar_files.size() || candiLidar_idx<0) continue;
                string current_file_name = output_path + "/lidarMap/" +
                                           lidar_files[candiLidar_idx] + ".png";
                cv::Mat lidarContextMap = cv::imread(current_file_name, 0);
                if ( !lidarContextMap.data) continue;
                cout << "current_file_name " << current_file_name << endl;

                Eigen::MatrixXd lidarContext;
                cv::cv2eigen(lidarContextMap, lidarContext);
                std::pair<double, int> sc_dist_result = distanceBtnScanContext(lidarContext, radarContext);
                if (sc_dist_result.first < candi_scancontext_result.first) {
                    candi_scancontext_result.first = sc_dist_result.first;
                    candi_scancontext_result.second = sc_dist_result.second;
                    similar_lidar_timestamp = atof(lidar_files[candiLidar_idx].c_str());
                    final_gt_idx = candiLidar_idx;
                }
            }
            // 找到了最近的radar candi_scancontext_result final_gt_idx  similar_lidar_timestamp
            // 最相似的lidar时间final_gt_idx
            /// 横向scan
            // y方向15m 185
            // 逆时 50多
            pcl::PointCloud<PointType>::Ptr cloud_lidar(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr cloud_radar(new pcl::PointCloud<PointType>());
            cloud_lidar->clear();
            if (pcl::io::loadPCDFile<pcl::PointXYZI>(output_path+"/lidarPcd/"+lidar_files[final_gt_idx]+".pcd",
                                                     *cloud_lidar) == -1) //* load the file
            {
              PCL_ERROR ("Couldn't read file lidar.pcd \n");
              return;
            }

            // 旋转scan结果
            Eigen::Matrix4f rotation_z = Eigen::Matrix4f::Identity();//定义绕Z轴的旋转矩阵，并初始化为单位阵
            double angle_z = candi_scancontext_result.second * 6.0 / 180.0 * M_PI;//(candi_scancontext_result.second - PC_NUM_SECTOR) * M_PI / 30.0;// M_PI / 2;//旋转90°

            rotation_z(0, 0) = cos(angle_z);
            rotation_z(0, 1) = -sin(angle_z);
            rotation_z(1, 0) = sin(angle_z);
            rotation_z(1, 1) = cos(angle_z);
            rotation_z(1,3) = 0;
            pcl::transformPointCloud(curr_radar_key.first, *cloud_radar, rotation_z);

            cv::Mat radar_density(Resolution,Resolution, CV_64FC1); // gray img
            cv::Mat lidar_density(Resolution,Resolution, CV_64FC1); // gray img
            radar_density *=0;
            lidar_density *=0;
            const cv::Point2d center(Resolution/2.0, Resolution/2.0);

            cloud2densMap(cloud_radar, radar_density, center);
            cloud2densMap(cloud_lidar, lidar_density, center);

            Eigen::MatrixXd radar_den_E;
            cv::cv2eigen(radar_density, radar_den_E);
            Eigen::MatrixXd lidar_den_E;
            cv::cv2eigen(lidar_density, lidar_den_E);

            std::pair<double, int> cc_dist_result = distanceBtnCartScanContext(lidar_den_E, radar_den_E);
            Eigen::Matrix4f trans_y = Eigen::Matrix4f::Identity();//定义绕Z轴的旋转矩阵，并初始化为单位阵
            if(cc_dist_result.second > 180)
              trans_y(1,3) = cc_dist_result.second - Resolution;
            else if (abs(cc_dist_result.second) <= 15)
              trans_y(1,3) = cc_dist_result.second;
            pcl::transformPointCloud(*cloud_radar, *cloud_radar, trans_y);




            Sophus::SE3d curr_gt = groud_truth_vector[final_gt_idx].second;

            // 发布两个点云校验一下
            sensor_msgs::PointCloud2 lidarMsg;
            sensor_msgs::PointCloud2 radarMsg;
            pcl::toROSMsg(curr_radar_key.first, radarMsg);
            radarMsg.header.frame_id = "/camera_init";
            pubRadarKeyMap.publish(radarMsg);
            pcl::toROSMsg(*cloud_lidar, lidarMsg);
            lidarMsg.header.frame_id="/camera_init";
            pubLidarKey.publish(lidarMsg);

            // 保存关键点云
            static int num=0;
            pcl::PCDWriter writer;
//            writer.write("/home/myk/dev/study/radar/output/pcd/radar/"+std::to_string(num)+".pcd",curr_radar_key.first);
//            writer.write("/home/myk/dev/study/radar/output/pcd/lidar/"+std::to_string(num++)+".pcd",*cloud_lidar);


            // 针对桥上的情况
            cout << "cloud_radar: " << cloud_radar->points.size() << endl;
            cout << "cloud_lidar: " << cloud_lidar->points.size() << endl;

            if(cloud_lidar->size()<(cloud_radar->size()/5)){
                isradaruseful = false; // DCC2时候去掉了
                cout << "点云太稀疏" << endl;
            }

            // todo 增加icp
//            pcl::VoxelGrid<pcl::PointXYZI> sor;
//
//            pcl::PointCloud<PointType>::Ptr curr_radar(new pcl::PointCloud<PointType>);
//
//            pcl::PointCloud<PointType>::Ptr target_lidar(new pcl::PointCloud<PointType>());
//            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
//            *curr_radar = *cloud_radar;
//            *target_lidar = *cloud_lidar;
//            icp.setInputSource(curr_radar);
//            icp.setInputTarget(target_lidar);
//            icp.align(*unused_result);
//            icp.getFinalTransformation();

            static int count = 1;
            cout << "id: " << count++ << endl;
            cout<<"角度 "<< candi_scancontext_result.second << "  分数 " << candi_scancontext_result.first << endl;
            cout<<"平移 "<< trans_y(1, 3) << "分数 " << cc_dist_result.first << endl;
            if (candi_scancontext_result.second>=5 && candi_scancontext_result.second<=25 ||
                candi_scancontext_result.second>=35 && candi_scancontext_result.second<=55)
            {
                isradaruseful = false;
                cout << "奇怪的对齐" << endl;
            }
            if(candi_scancontext_result.first > 0.8) {
                isradaruseful = false; // DCC2时候去掉了
                cout << "旋转分数太低" << endl;
            }
            if (cc_dist_result.first > 0.9) isradaruseful = false; //  DCC2时候去掉了
            if (abs(trans_y(1, 3))>8) {
                isradaruseful = false;
                cout << "平移太多" << endl;

            }

//            Eigen::AngleAxisd rotation_vector(candi_scancontext_result.second * 6.0 / 180.0 * M_PI,
//                                              Eigen::Vector3d(0, 0, 1));
//            Sophus::SE3d radar2lidar_T2(rotation_vector.toRotationMatrix(), Eigen::Vector3d(0, trans_y(1,3), 0));
//            tPose6D radar2lidar = SE32tPose6D(radar2lidar_T2);

            /// 这里替换到
//            aff_mtx.lock();
//            radar2lidar_Pose6D.push(radar2lidar);
//            aff_mtx.unlock();
            /// 这里替换=

            gtfactor_mx.lock();
            gtfactor.push_back(curr_gt);// * radar2lidar_T);
            isgtfactorUseful.push(isradaruseful);
            gtfactor_mx.unlock();

          std::vector<int> align_results(3);
          align_results[0] = candi_scancontext_result.second;
          align_results[1] = trans_y(1,3);
          align_results[2] = final_gt_idx;
          Icp_mtx.lock();
          IcpradarWinSet.push(make_pair(*cloud_radar,align_results)); // 加的是旋转后的radar pcl
          Icp_mtx.unlock();
        }
    }
}

// icp
void process_Icp(){
  ros::Rate rate(20);
  pcl::PointCloud<PointType>::Ptr radar_cloud(new pcl::PointCloud<PointType>());
  std::vector<int> align_result(3);
  while(ros::ok()){
    rate.sleep();
    while(!IcpradarWinSet.empty() ){
      static int count = 1;
      if (!isgtfactorUseful.front()){
        tPose6D radar2lidar;
        aff_mtx.lock();
        radar2lidar_Pose6D.push(radar2lidar);
        aff_mtx.unlock();

        Icp_mtx.lock();
        IcpradarWinSet.pop();
        Icp_mtx.unlock();

        cout << count++ << " radar2lidar " << radar2lidar.yaw << endl;
        break;

      }

      Icp_mtx.lock();
      *radar_cloud = IcpradarWinSet.front().first;
      align_result = IcpradarWinSet.front().second;
      IcpradarWinSet.pop();
      Icp_mtx.unlock();

      pcl::PointCloud<PointType>::Ptr cloud_lidar(new pcl::PointCloud<PointType>());
      pcl::PointCloud<PointType>::Ptr per_cloud_lidar(new pcl::PointCloud<PointType>());
      cloud_lidar->clear();
      per_cloud_lidar->clear();
//      if (pcl::io::loadPCDFile<pcl::PointXYZI>(output_path+"/lidarPcd/"+lidar_files[align_result[2]]+".pcd",
//                                               *cloud_lidar) == -1) //* load the file
//      {
//        PCL_ERROR ("Couldn't read file lidar.pcd \n");
//        return;
//      }
      // 读取10帧组成submap
      clock_t start, end;
      start = clock();
      for (int num_cloud = -10; num_cloud <= 10 ; num_cloud+=10) {
        int lidar_idx = align_result[2] + num_cloud;
        if(lidar_idx < 0 || lidar_idx > lidar_files.size()) continue;
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(output_path+"/lidarPcd/"+lidar_files[lidar_idx]+".pcd",
                                                 *per_cloud_lidar) == -1) //* load the file
        {
          PCL_ERROR ("Couldn't read file lidar.pcd \n");
          return;
        }
        *cloud_lidar += *win2mid(per_cloud_lidar, SE32tPose6D(groud_truth_vector[lidar_idx].second), SE32tPose6D(groud_truth_vector[align_result[2]].second));
        per_cloud_lidar->clear();
      }
      end = clock();
      double endtime=(double)(end-start)/CLOCKS_PER_SEC;
      cout << "read lidar pcd cost " << endtime*1000 << endl;

      start = clock();
      pcl::PointCloud<PointType>::Ptr target_lidar(new pcl::PointCloud<PointType>());
      *target_lidar = *cloud_lidar;
      downSizeLidarMap.setInputCloud(cloud_lidar);
      downSizeLidarMap.filter(*target_lidar);

      pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
      icp.setInputSource(radar_cloud);
      icp.setInputTarget(target_lidar);
      icp.align(*unused_result);
      icp.getFinalTransformation();
      end = clock();
      endtime=(double)(end-start)/CLOCKS_PER_SEC;
      cout << "icp cost " << endtime*1000 << endl;
      cout << "icp.getFinalTransformation() \n" << icp.getFinalTransformation() << endl;

      Eigen::AngleAxisd rotation_vector(align_result[0] * 6.0 / 180.0 * M_PI,
                                        Eigen::Vector3d(0, 0, 1));
      cout << count << " align_result " << align_result[0] << endl;
//      if (align_result[1] > Resolution/2) align_result[1] -= Resolution;
      Sophus::SE3d radar2lidar_T2(rotation_vector.toRotationMatrix(), Eigen::Vector3d(0, align_result[1], 0));

      Eigen::Matrix4d transformation_matrix = icp.getFinalTransformation().cast<double>();
      Eigen::Quaterniond q(transformation_matrix.block<3,3>(0,0));
      q.normalize();
      Eigen::Vector3d t;
      t = transformation_matrix.block<3,1>(0,3);
//      t[2] =0;
      Sophus::SE3d icpT(q, t);
//      radar2lidar_T2 * icp.getFinalTransformation();
      tPose6D radar2lidar = SE32tPose6D(radar2lidar_T2.inverse() * icpT.inverse());

      aff_mtx.lock();
      radar2lidar_Pose6D.push(radar2lidar);
      aff_mtx.unlock();


      cout << count++ << " radar2lidar " << radar2lidar.yaw << endl;
    }
  }

}

void publish_car_model(ros::Time t,Eigen::Vector3d t_w_car, Eigen::Quaterniond q_w_car)
{
    // 旋转
    Eigen::Vector3d eulerAngle(0,-M_PI/2,M_PI/2);
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitZ()));

    Eigen::Quaterniond quaternion;
    quaternion=yawAngle*pitchAngle*rollAngle;


    visualization_msgs::MarkerArray markerArray_msg;
    visualization_msgs::Marker car_mesh;
    car_mesh.header.stamp = t;
    car_mesh.header.frame_id = "/camera_init";
    car_mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
    car_mesh.action = visualization_msgs::Marker::ADD;
    car_mesh.id = 0;

    car_mesh.mesh_resource = "package://radar_on_lidar_map/model/car.dae";

    Eigen::Matrix3d rot;
    rot << 0, 0, -1, 0, -1, 0, -1, 0, 0;

    Eigen::Quaterniond Q;
    Q = q_w_car * rot * quaternion;
    car_mesh.pose.position.x    = t_w_car.x();
    car_mesh.pose.position.y    = t_w_car.y();
    car_mesh.pose.position.z    = t_w_car.z();
    car_mesh.pose.orientation.w = Q.w();
    car_mesh.pose.orientation.x = Q.x();
    car_mesh.pose.orientation.y = Q.y();
    car_mesh.pose.orientation.z = Q.z();

    car_mesh.color.a = 1.0;
    car_mesh.color.r = 1.0;
    car_mesh.color.g = 0.0;
    car_mesh.color.b = 0.0;

    float major_scale = 2.0;

    car_mesh.scale.x = major_scale;
    car_mesh.scale.y = major_scale;
    car_mesh.scale.z = major_scale;
    markerArray_msg.markers.push_back(car_mesh);
    pub_car.publish(markerArray_msg);
}


void pubMap(void) {
    int SKIP_FRAMES = 2;
    int counter = 0;

    radarMap->clear();

    keyframePosesUpdated_mutex.lock();
    radarMap_mtx.lock();
    for (int node_idx = int(keyframePosesUpdated.size())-1; node_idx < int(keyframePosesUpdated.size());
         node_idx++) {
        if (counter % SKIP_FRAMES == 0) {
	    pcl::PointCloud<PointType>::Ptr inliner(new pcl::PointCloud<PointType>());
		for(auto point:radarMapList[node_idx]->points){
			if(point.x*point.x+point.y*point.y<900) inliner->points.push_back(point);
		}
            *radarMap += *local2global(inliner,
                                       keyframePosesUpdated[node_idx]);
        }
        counter++;
    }



    pcl::PointCloud<PointType>::Ptr radarMapDP(new pcl::PointCloud<PointType>);
    downSizeFilterMapROLM.setInputCloud(radarMap);
    downSizeFilterMapROLM.setLeafSize(0.5f, 0.5f, 0.5f);
    *radarMapDP = *radarMap;
    downSizeFilterMapROLM.filter(*radarMapDP);

    keyframePosesUpdated_mutex.unlock();
    radarMap_mtx.unlock();
    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*radarMapDP, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "/camera_init";
    pubMapAftROLM.publish(laserCloudMapPGOMsg);
}

void pubPath(void) {
    if (keyframePosesUpdated.empty()) return;
    nav_msgs::Odometry odomAftROLM;
    nav_msgs::Path pathAftROLM;
    pathAftROLM.header.frame_id = '/camera_init';
    // todo
    keyframePosesUpdated_mutex.lock();
    for (int node_idx = 0; node_idx < int(keyframePosesUpdated.size()) - 1;
         node_idx++) {
        tPose6D &pose_est =
                keyframePosesUpdated.at(node_idx); // upodated poses

        nav_msgs::Odometry odomAftROLMthis;
        odomAftROLMthis.header.frame_id = "/camera_init";
        odomAftROLMthis.child_frame_id = "/aft_pgo";
        odomAftROLMthis.header.stamp =
                ros::Time().fromSec(pose_est.timestamp);
        odomAftROLMthis.pose.pose.position.x = pose_est.x;
        odomAftROLMthis.pose.pose.position.y = pose_est.y;
        odomAftROLMthis.pose.pose.position.z = pose_est.z;
        odomAftROLMthis.pose.pose.orientation =
                tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch,
                                                        pose_est.yaw);
        odomAftROLM = odomAftROLMthis;

        geometry_msgs::PoseStamped poseStampAftROLM;
        poseStampAftROLM.header = odomAftROLMthis.header;
        poseStampAftROLM.pose = odomAftROLMthis.pose.pose;

        pathAftROLM.header.stamp = odomAftROLMthis.header.stamp;
        pathAftROLM.header.frame_id = "/camera_init";
        pathAftROLM.poses.push_back(poseStampAftROLM);
    }
    keyframePosesUpdated_mutex.unlock();
//    pubOdomAftROLM.publish(odomAftROLM); // last pose
    pubPathAftROLM.publish(pathAftROLM); // poses

    // todo Ignoring transform?
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftROLM.pose.pose.position.x,
                                    odomAftROLM.pose.pose.position.y,
                                    odomAftROLM.pose.pose.position.z));
    q.setW(odomAftROLM.pose.pose.orientation.w);
    q.setX(odomAftROLM.pose.pose.orientation.x);
    q.setY(odomAftROLM.pose.pose.orientation.y);
    q.setZ(odomAftROLM.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftROLM.header.stamp,
                                          "/camera_init", "/aft_rolm"));
    Eigen::Quaterniond car_q(odomAftROLM.pose.pose.orientation.w, odomAftROLM.pose.pose.orientation.x, odomAftROLM.pose.pose.orientation.y, odomAftROLM.pose.pose.orientation.z);
    car_q.normalize();
    Eigen::Vector3d car_t(odomAftROLM.pose.pose.position.x,odomAftROLM.pose.pose.position.y,odomAftROLM.pose.pose.position.z);

    publish_car_model(odomAftROLM.header.stamp, car_t, car_q);

}

void process_pubPath() {
    float hz = 5.0;
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if (!keyframePosesUpdated.empty()) {
            pubPath();
            pubMap();
        }
    }
}

void process_writePose() {
    while (1) {
        char c = getchar();
        if (c == 'g') {
            keyframePosesUpdated_mutex.lock();
            outputPath(keyframePosesUpdated);
            keyframePosesUpdated_mutex.unlock();
            cout << "save vio_gps_graph to" << output_path << "/results.txt" << endl;
            cout << "save gps_truth to" << output_path << "/global_truth.txt" << endl;
        }
    }
    std::chrono::milliseconds dura(5);
    std::this_thread::sleep_for(dura);
}


int main(int arc, char **argv) {

    ros::init(arc, argv, "laserPGO");
    ros::NodeHandle nh;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZI>);

    nh.param<int>("num_keyFrame", num_keyFrame, 10);
    nh.param<string>("data_path", data_path);
    nh.param<string>("output_path", output_path);
    nh.param<bool>("generate_map", generate_map, false);

    data_path = "/media/myk/mykData/Mulran/Kaist/Kaist03";
    output_path = "/media/myk/2T1/radardata/HUANGLONG";


    gtsam::ISAM2Params parameters;  // gtsam
    parameters.relinearizeThreshold = 0.01;
    parameters.factorization = gtsam::ISAM2Params::QR;
    parameters.relinearizeSkip = 1;

    isam = new gtsam::ISAM2(parameters);
    initNoises(); // gtasam一些噪声设置

    // yetiodom 关键点云输出接收
    ros::Subscriber subRadarCloud = nh.subscribe<sensor_msgs::PointCloud2>("/yeti_cloud_local", 100,
                                                                           radar_local_callback);
    // yetiodom poses
    ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/yeti_odom", 100, Odometry_callback);
    ros::Subscriber subGPS = nh.subscribe<sensor_msgs::NavSatFix>("/gps/fix", 100, gpsHandler);

    pubRadarKeyMap = nh.advertise<sensor_msgs::PointCloud2>("/radar_key_map", 100);
    pubLidarKey = nh.advertise<sensor_msgs::PointCloud2>("/lidar_key_pointcloud", 100);
    pubOdomAftROLM = nh.advertise<nav_msgs::Odometry>("/aft_rolm_odom", 100);  // 回环后的里程计
    pubPathAftROLM = nh.advertise<nav_msgs::Path>("/aft_rolm_path", 100);
    pubOdom_gt = nh.advertise<nav_msgs::Odometry>("/gt_odom", 100);
    pubPath_gt = nh.advertise<nav_msgs::Path>("/gt_path", 100);
    ros::Publisher pubCircle = nh.advertise<sensor_msgs::PointCloud2>("/circle", 100);
    pubMapAftROLM = nh.advertise<sensor_msgs::PointCloud2>("/aft_rolm_map", 100);
    pub_car = nh.advertise<visualization_msgs::MarkerArray>("/car_model", 1000);

    ros::Publisher publidarmap = nh.advertise<sensor_msgs::PointCloud2>("/lidarmap", 100);

    // calib
    Eigen::Matrix3d rotation_matrix1;
    rotation_matrix1 = Eigen::AngleAxisd(base2radar.yaw, Eigen::Vector3d::UnitZ()) *
                       Eigen::AngleAxisd(base2radar.pitch, Eigen::Vector3d::UnitY()) *
                       Eigen::AngleAxisd(base2radar.roll, Eigen::Vector3d::UnitX());
    Sophus::SE3d TEMP(rotation_matrix1, Eigen::Vector3d(base2radar.x, base2radar.y, base2radar.z));
    Sophus::SE3d TEMP2 = tPose6D2SE3(base2radar);
    base2radar_T = TEMP2;
    base2lidar_T = tPose6D2SE3(base2lidar);
    lidar2radar_T = base2radar_T * base2lidar_T.inverse();  // todo 校验tPose6D2SE3有没有问题

    if (generate_map){ // 真值列表减去第一帧的位置偏移
      gt_file.open(output_path+"/gt.txt", ios_base::out | ios_base::trunc);
        map_pose_file.open(output_path+"/mapPose.txt", ios_base::out | ios_base::trunc);
        readLiadarfilenames(data_path+"/sensor_data/Ouster", lidar_files, "bin");
        // todo 输出地图pose
        generateMapPose(data_path + "/global_pose.csv", lidar_files);
        importLidarMap(data_path, output_path, lidar_files);
        map_pose_file.close();
        gt_file.close();
        return 0;
    }

    // init
    key_radar_index.clear();
    results_file.open(output_path+"/results.txt", ios_base::out | ios_base::trunc);
    readGroundTurth(output_path + "/mapPose.txt", groud_truth_vector,
                    groud_truth_tree_xyz);
    lidar_files.clear();
    readLiadarfilenames(output_path+"/lidarMap", lidar_files,"png");
    downSizeLidarMap.setLeafSize(0.2f, 0.2f, 0.2f);
    // 读如pcd并发布
    pcl::PointCloud<PointType>::Ptr cloud_lidar(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr pub_lidar(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr color_lidar(new pcl::PointCloud<PointType>());
    cloud_lidar->clear();
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(output_path+"/global_cloud.pcd",
                                             *cloud_lidar) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file lidar.pcd \n");
        return 0;
    }
    for (auto point:cloud_lidar->points) {
        pcl::PointXYZI pp = point;
        pp.intensity = point.z/10.0;

        color_lidar->push_back(pp);
    }
    *pub_lidar = *cloud_lidar;
    downSizeLidarMap.setInputCloud(color_lidar);
    downSizeLidarMap.filter(*pub_lidar);
    pub_lidar->width = pub_lidar->size();
    pub_lidar->height = 1;
    sensor_msgs::PointCloud2 lidarMapMsg;
    pcl::toROSMsg(*pub_lidar, lidarMapMsg);
    lidarMapMsg.header.frame_id = "/camera_init";
    publidarmap.publish(lidarMapMsg);



    /// icp setting
    icp.setMaxCorrespondenceDistance(3.0);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(10);

    // threads
    std::thread data_align{process_data};
    std::thread ROLM{process_localization};
    // TODO icp thread
    std::thread ICP{process_Icp};
    std::thread PoseGraph{process_posegraph};
    std::thread pubPath{process_pubPath};
    std::thread writePose{process_writePose};

    // draw a circle
    pcl::PointCloud<PointType> circle;
    for (int i = 0; i < 360; ++i) {
        PointType point;
        point.x = PC_MAX_RADIUS * cos(i / 180.0 * M_PI);
        point.y = PC_MAX_RADIUS * sin(i / 180.0 * M_PI);
        circle.points.push_back(point);
    }
    sleep(5);    //程序延时5s
    sensor_msgs::PointCloud2 circle_Msg;
    pcl::toROSMsg(circle, circle_Msg);
    circle_Msg.header.frame_id = "/camera_init";
//    pubCircle.publish(circle_Msg);

    // google map
    static tf::TransformBroadcaster map_init;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setW(1);
    q.setX(0);
    q.setY(0);
    q.setZ(0);
    transform.setRotation(q);
    map_init.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                                "/map", "/camera_init"));
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);

    std::cout << "Hello, World!" << std::endl;
    ros::spin();
    ROLM.join();
    return 0;
}

// DCC02
