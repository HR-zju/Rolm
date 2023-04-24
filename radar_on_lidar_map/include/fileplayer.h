//
// Created by myk on 2021/12/9.
//

#ifndef SRC_FILEPLAYER_H
#define SRC_FILEPLAYER_H

#include <pcl/kdtree/kdtree_flann.h>

std::fstream gt_file;
std::fstream results_file;
std::fstream map_pose_file;

template<typename T>
using EigenVecotr = std::vector<T, Eigen::aligned_allocator<T>>;
ros::Publisher pubOdom_gt, pubPath_gt, pub_car;

void readGroundTurth(string grounTruth_file, EigenVecotr<pair<double, Sophus::SE3d>> &groud_truth_vector,
                     pcl::KdTreeFLANN<pcl::PointXY> &groud_truth_tree) {

    Eigen::Vector3d initTrans = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initR = Eigen::Matrix3d::Identity();
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);//初始点云对象
    groud_truth_vector.clear();

    FILE *pFile;
    pFile = fopen(grounTruth_file.c_str(), "r");
    double time_stamp;
    double T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23;

    while (fscanf(pFile, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                  &time_stamp, &T00, &T01, &T02, &T03, &T10,
                  &T11, &T12, &T13, &T20, &T21, &T22, &T23) != EOF) {
        Eigen::Matrix4d T;
        T << T00, T01, T02, T03,
                T10, T11, T12, T13,
                T20, T21, T22, T23,
                0.0, 0.0, 0.0, 1.0;
        static bool ifInitOrigin = false;
        static int num_tru = 0;
        if (num_tru < 100) {
            num_tru++;
            continue;
        }
        if (!ifInitOrigin) {
            initTrans << T.block<3, 1>(0, 3);
            initR << T.block<3, 3>(0, 0);
            ifInitOrigin = true;
            R_radar2enu = initR;

            pcl::PointXY point;
            point.x = 0.0;
            point.y = 0.0;
            cloud->points.push_back(point);  // set origin 0,0
            Eigen::Quaterniond q(Eigen::Matrix3d::Identity());
            q.normalize();
            Sophus::SE3d _T(q, Eigen::Vector3d::Zero());
            groud_truth_vector.push_back(std::make_pair(time_stamp, _T));
        } else {
            Eigen::Matrix3d _R;
            Eigen::Vector3d _t;
            _R = T.block<3, 3>(0, 0);
            _t = T.block<3, 1>(0, 3);

            Eigen::Quaterniond q(_R);
            q.normalize();
            Sophus::SE3d _T(q, _t);
            Eigen::Quaterniond q_init(initR);
            q_init.normalize();
            Sophus::SE3d init_T(q_init, initTrans);
//            _T = init_T.inverse() * _T;
            _T.translation() = _T.translation() - initTrans;

//            _T = tPose6D2SE3(base2radar).inverse() * _T;

            pcl::PointXY point;
            point.x = _T.translation().x();
            point.y = _T.translation().y();
            cloud->points.push_back(point);  //base2radar_T

            Eigen::Quaterniond q_out(_T.rotationMatrix());

            // transfer gt to tum
            gt_file << setiosflags(ios::fixed) << setprecision(4)
                    << time_stamp * 1e-9 << " " <<
                    _T.translation().x() << " " << _T.translation().y() << " " << _T.translation().z() << " " <<
                    q_out.x() << " " << q_out.y() << " " << q_out.z() << " " << q_out.w() << endl;

            groud_truth_vector.push_back(make_pair(time_stamp, _T));

        }
    }

    fclose(pFile);
    groud_truth_tree.setInputCloud(cloud);
    ROS_INFO("===read gt finish!===");

    //// pubpath
    nav_msgs::Odometry odom_gt;
    nav_msgs::Path path_gt;
    path_gt.header.frame_id = '/camera_init';

    for (int node_idx = 0; node_idx < groud_truth_vector.size() - 1;
         node_idx++) {
        const Sophus::SE3d &pose_est =
                groud_truth_vector.at(node_idx).second; // upodated poses

        nav_msgs::Odometry gt_odom;
        gt_odom.header.frame_id = "/camera_init";
        gt_odom.child_frame_id = "/aft_pgo";
        gt_odom.header.stamp =
                ros::Time::now();
        gt_odom.pose.pose.position.x = pose_est.translation().x();
        gt_odom.pose.pose.position.y = pose_est.translation().y();
        gt_odom.pose.pose.position.z = pose_est.translation().z();
        Eigen::Quaterniond q(pose_est.rotationMatrix());
        gt_odom.pose.pose.orientation.x = q.x();
        gt_odom.pose.pose.orientation.y = q.y();
        gt_odom.pose.pose.orientation.z = q.z();
        gt_odom.pose.pose.orientation.w = q.w();

        odom_gt = gt_odom;
        if (node_idx < 1000) {
            pubOdom_gt.publish(odom_gt);
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
        }

        geometry_msgs::PoseStamped poseStampAftROLM;
        poseStampAftROLM.header = gt_odom.header;
        poseStampAftROLM.pose = gt_odom.pose.pose;

        path_gt.header.stamp = gt_odom.header.stamp;
        path_gt.header.frame_id = "/camera_init";
        path_gt.poses.push_back(poseStampAftROLM);
    }
    keyframePosesUpdated_mutex.unlock();
    // last pose
    pubPath_gt.publish(path_gt); // poses
}


void readGroundTurth(string grounTruth_file, EigenVecotr<pair<double, Sophus::SE3d>> &groud_truth_vector,
                     pcl::KdTreeFLANN<pcl::PointXYZ> &groud_truth_tree) {

    Eigen::Vector3d initTrans = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initR = Eigen::Matrix3d::Identity();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);//初始点云对象
    groud_truth_vector.clear();

    FILE *pFile;
    pFile = fopen(grounTruth_file.c_str(), "r");
    double time_stamp;
    double T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23;
    double Tx, Ty, Tz, Tqx, Tqy, Tqz, Tqw;

    while (fscanf(pFile, "%lf %lf %lf %lf %lf %lf %lf %lf ",
                  &time_stamp, &Tx, &Ty, &Tz, &Tqx,
                  &Tqy, &Tqz, &Tqw) != EOF) {
        // pclnn
        pcl::PointXYZ point;
        point.x = Tx;
        point.y = Ty;
        point.z = Tz;
        cloud->points.push_back(point);

        Eigen::Quaterniond m_Q(Tqw, Tqx, Tqy, Tqz);
        m_Q.normalize();
        Sophus::SE3d gtT(m_Q, Eigen::Vector3d(Tx, Ty, Tz));
        groud_truth_vector.push_back(make_pair(time_stamp, gtT));
    }

    fclose(pFile);
    groud_truth_tree.setInputCloud(cloud);
    ROS_INFO("===read gt finish!===");

    //// pubpath
    nav_msgs::Odometry odom_gt;
    nav_msgs::Path path_gt;
    path_gt.header.frame_id = '/camera_init';

    for (int node_idx = 0; node_idx < groud_truth_vector.size() - 1;
         node_idx++) {
        const Sophus::SE3d &pose_est =
                groud_truth_vector.at(node_idx).second; // upodated poses

        nav_msgs::Odometry gt_odom;
        gt_odom.header.frame_id = "/camera_init";
        gt_odom.child_frame_id = "/aft_pgo";
        gt_odom.header.stamp =
                ros::Time::now();
        gt_odom.pose.pose.position.x = pose_est.translation().x();
        gt_odom.pose.pose.position.y = pose_est.translation().y();
        gt_odom.pose.pose.position.z = pose_est.translation().z();
        Eigen::Quaterniond q(pose_est.rotationMatrix());
        gt_odom.pose.pose.orientation.x = q.x();
        gt_odom.pose.pose.orientation.y = q.y();
        gt_odom.pose.pose.orientation.z = q.z();
        gt_odom.pose.pose.orientation.w = q.w();

        odom_gt = gt_odom;
        if (node_idx < 1000) {
            pubOdom_gt.publish(odom_gt);
            std::chrono::milliseconds dura(5);
            std::this_thread::sleep_for(dura);
        }

        geometry_msgs::PoseStamped poseStampAftROLM;
        poseStampAftROLM.header = gt_odom.header;
        poseStampAftROLM.pose = gt_odom.pose.pose;

        path_gt.header.stamp = gt_odom.header.stamp;
        path_gt.header.frame_id = "/camera_init";
        path_gt.poses.push_back(poseStampAftROLM);
    }
    keyframePosesUpdated_mutex.unlock();
    // last pose
    pubPath_gt.publish(path_gt); // poses
    std::chrono::milliseconds dura(10);
    std::this_thread::sleep_for(dura);
}

static inline bool exists(const std::string &name) {
    struct stat buffer;
    return !(stat(name.c_str(), &buffer) == 0);
}

struct less_than_img {
    inline bool operator()(const std::string &img1, const std::string &img2) {
        std::vector<std::string> parts;
        boost::split(parts, img1, boost::is_any_of("."));
        int64 i1 = std::stoll(parts[0]);
        boost::split(parts, img2, boost::is_any_of("."));
        int64 i2 = std::stoll(parts[0]);
        return i1 < i2;
    }
};

void readLiadarfilenames(std::string path, std::vector<std::string> &files, std::string extension) {
    DIR *dirp = opendir(path.c_str());
    struct dirent *dp;
    while ((dp = readdir(dirp)) != NULL) {
        if (exists(dp->d_name)) {
            if (!extension.empty()) {
                std::vector<std::string> parts;
                boost::split(parts, dp->d_name, boost::is_any_of("."));
                if (parts[parts.size() - 1].compare(extension) != 0)
                    continue;
            }
            std::vector<std::string> time_only;
            boost::split(time_only, dp->d_name, boost::is_any_of("."));
            files.push_back(time_only[0]);
        }
    }
    // Sort files in ascending order of time stamp
    std::sort(files.begin(), files.end(), less_than_img());
}

void outputPath(const std::vector<tPose6D> &_vPose) {
//    results_file.clear();
//    for (auto pose:_vPose) {
//        Eigen::Quaterniond q(tPose6D2SE3(pose).rotationMatrix());
//        results_file << setiosflags(ios::fixed) << setprecision(4) <<
//                     pose.timestamp << " " <<
//                     pose.x << " " << pose.y << " " << pose.z << " " <<
//                     q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
//    }
    results_file.clear();
    for (auto pose:_vPose) {
        Eigen::Vector3d eulerAngle = (tPose6D2SE3(pose).rotationMatrix()).eulerAngles(2,1,0);
        if(abs(eulerAngle(1)) > M_PI/2.0 || abs(eulerAngle(2)) > M_PI/2.0  ) eulerAngle(0) = eulerAngle(0)+M_PI;
        std::cout << "rpy: " <<  eulerAngle(2) << " " << eulerAngle(1) << " " << eulerAngle(0) << std::endl;
        Eigen::AngleAxisd yaw(Eigen::AngleAxisd (eulerAngle(0), Eigen::Vector3d::UnitZ()));
        Eigen::Quaterniond q;
        q = Eigen::AngleAxisd (eulerAngle(0), Eigen::Vector3d::UnitZ())
            *  Eigen::AngleAxisd (0.0, Eigen::Vector3d::UnitY())
            *  Eigen::AngleAxisd (0.0, Eigen::Vector3d::UnitX());
//        q = yaw;
        results_file << setiosflags(ios::fixed) << setprecision(4) <<
                     pose.timestamp << " " <<
                     pose.x << " " << pose.y << " " << /*pose.z*/ 0.0 << " " <<
                     q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
}

void importLidarMap(std::string _data_file, std::string _out_file, std::vector<std::string> _lidar_files) {
    ifstream file;
    for (auto file_name:_lidar_files) {
        string current_file_name = _data_file + "/sensor_data/Ouster" +
                                   "/" + file_name + ".bin";
        pcl::PointCloud<PointType> ref_cloud;
        ref_cloud.clear();
        file.open(current_file_name, ios::in | ios::binary);
        int k = 0;
        while (!file.eof()) {
            PointType point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity),
                      sizeof(float));
//                point.ring = (k % 64) + 1;
            k = k + 1;

            Eigen::Vector3d temp_point(point.x, point.y, point.z);
            temp_point = lidar2radar_T.rotationMatrix() * temp_point;
            point.x = temp_point[0];
            point.y = temp_point[1];
            point.z = 2; // temp_point[2];
            if (temp_point[2] > -0.5)  // without ground
                ref_cloud.points.push_back(point);
        }
        file.close();
        Eigen::MatrixXd lidarContext(20,60);
//      Eigen::Matrix3d lidarContext3;
        pcl2Scancontex(ref_cloud, lidarContext);
//        _lidarMapVector.push_back(lidarContext);
        cv::Mat cvLidarMap;
        cv::eigen2cv(lidarContext, cvLidarMap);
        cv::imwrite(_out_file + "/lidarMap/" + file_name + ".png", cvLidarMap);
        cout << "write " << file_name << endl;

        ref_cloud.height = 1;
        ref_cloud.width = ref_cloud.points.size();
        ref_cloud.points.resize(ref_cloud.width * ref_cloud.height);
        pcl::io::savePCDFileASCII (_out_file+"/lidarPcd/"+file_name+".pcd", ref_cloud);

    }
    cout << "===finish import map===" << endl;
}

void generateMapPose(string grounTruth_file, std::vector<std::string> &_lidar_files) {

    Eigen::Vector3d initTrans = Eigen::Vector3d::Zero();
    Eigen::Matrix3d initR = Eigen::Matrix3d::Identity();
    pcl::PointCloud<pcl::PointXY>::Ptr cloud(new pcl::PointCloud<pcl::PointXY>);  // 初始点云对象

    FILE *pFile;
    pFile = fopen(grounTruth_file.c_str(), "r");
    double time_stamp;
    double T00, T01, T02, T03, T10, T11, T12, T13, T20, T21, T22, T23;

    while (fscanf(pFile, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf",
                  &time_stamp, &T00, &T01, &T02, &T03, &T10,
                  &T11, &T12, &T13, &T20, &T21, &T22, &T23) != EOF) {

        Eigen::Matrix4d T;
        T << T00, T01, T02, T03,
                T10, T11, T12, T13,
                T20, T21, T22, T23,
                0.0, 0.0, 0.0, 1.0;
        static bool ifInitOrigin = false;
        static int num_tru = 0;
        if (num_tru < 100) {
            num_tru++;
            continue;
        }
        if (!ifInitOrigin) {
            // 判断时间
            while (time_stamp > std::atof(_lidar_files[0].c_str())) {
                _lidar_files.erase(_lidar_files.begin(), _lidar_files.begin() + 1);
            }
            _lidar_files.erase(_lidar_files.end() - 5, _lidar_files.end());

            initTrans << T.block<3, 1>(0, 3);
            initR << T.block<3, 3>(0, 0);
            ifInitOrigin = true;
            R_radar2enu = initR;

            pcl::PointXY point;
            point.x = 0.0;
            point.y = 0.0;
            cloud->points.push_back(point);  // set origin 0,0
            Eigen::Quaterniond q(Eigen::Matrix3d::Identity());
            q.normalize();
            Sophus::SE3d _T(q, Eigen::Vector3d::Zero());
        } else {

            Eigen::Matrix3d _R;
            Eigen::Vector3d _t;
            _R = T.block<3, 3>(0, 0);
            _t = T.block<3, 1>(0, 3);

            Eigen::Quaterniond q(_R);
            q.normalize();
            Sophus::SE3d _T(q, _t);
            Eigen::Quaterniond q_init(initR);
            q_init.normalize();
            Sophus::SE3d init_T(q_init, initTrans);
//            _T = init_T.inverse() * _T;
            _T.translation() = _T.translation() - initTrans;

//            _T = tPose6D2SE3(base2radar).inverse() * _T;

            pcl::PointXY point;
            point.x = _T.translation().x();
            point.y = _T.translation().y();
            cloud->points.push_back(point);  // base2radar_T

            Eigen::Quaterniond q_out(_T.rotationMatrix());

            // transfer gt to tum
            gt_file << setiosflags(ios::fixed) << setprecision(10)
                    << time_stamp * 1e-9 << " " <<
                    _T.translation().x() << " " <<
                    _T.translation().y() << " " <<
                    _T.translation().z() << " " <<
                    q_out.x() << " " << q_out.y() << " "
                    << q_out.z() << " " << q_out.w() << endl;

            static int num_radar = 0;
            cout << "num_radar " << num_radar << endl;
            if (num_radar > _lidar_files.size()) return;
            if (std::atof(_lidar_files[num_radar].c_str()) - time_stamp < 1e6) {
//                cout << "_lidar_files " << std::atof(_lidar_files[num_radar].c_str()) << endl;
//                cout << "time_stamp " << time_stamp << endl;
                map_pose_file << setiosflags(ios::fixed) << setprecision(10)
                              << time_stamp * 1e-9 << " " <<
                              _T.translation().x() << " " <<
                              _T.translation().y() << " " <<
                              _T.translation().z() << " " <<
                              q_out.x() << " " << q_out.y() << " " <<
                              q_out.z() << " " << q_out.w() << endl;
                num_radar++;
            }
        }
    }


    fclose(pFile);

    ROS_INFO("===read gt finish!===");
}

void readLidarBin(string _file_name, pcl::PointCloud<PointType> &_pcl){
    ifstream file;
    file.open(_file_name, ios::in | ios::binary);
    while (!file.eof()) {
        PointType point;
        file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
        file.read(reinterpret_cast<char *>(&point.intensity),
                  sizeof(float));
//                point.ring = (k % 64) + 1;

        Eigen::Vector3d temp_point(point.x, point.y, point.z);
        temp_point = lidar2radar_T.rotationMatrix() * temp_point;
        point.x = temp_point[0];
        point.y = temp_point[1];
        point.z = 1; // temp_point[2];
        if (temp_point[2] > -0.5)  // without ground
            _pcl.points.push_back(point);
    }
    file.close();
}

#endif //SRC_FILEPLAYER_H
