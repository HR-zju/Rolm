//////
////// Created by myk on 2021/12/10.
//
//#include <iostream>
//#include <vector>
//#include <dirent.h>
//#include <boost/algorithm/string.hpp>
//#include <pcl_conversions/pcl_conversions.h>
//#include <string>
//#include <opencv2/core/core.hpp>
//
//typedef pcl::PointXYZI PointType;
//
//using namespace std;
//static inline bool exists(const std::string &name) {
//    struct stat buffer;
//    return !(stat(name.c_str(), &buffer) == 0);
//}
//
//struct less_than_img {
//    inline bool operator()(const std::string &img1, const std::string &img2) {
//        std::vector<std::string> parts;
//        boost::split(parts, img1, boost::is_any_of("."));
//        int64 i1 = std::stoll(parts[0]);
//        boost::split(parts, img2, boost::is_any_of("."));
//        int64 i2 = std::stoll(parts[0]);
//        return i1 < i2;
//    }
//};
//
//void readLiadarfilenames(std::string path, std::vector<std::string> &files, std::string extension) {
//    DIR *dirp = opendir(path.c_str());
//    struct dirent *dp;
//    while ((dp = readdir(dirp)) != NULL) {
//        if (exists(dp->d_name)) {
//            if (!extension.empty()) {
//                std::vector<std::string> parts;
//                boost::split(parts, dp->d_name, boost::is_any_of("."));
//                if (parts[parts.size() - 1].compare(extension) != 0)
//                    continue;
//            }
//            std::vector<std::string> time_only;
//            boost::split(time_only, dp->d_name, boost::is_any_of("."));
//            files.push_back(time_only[0]);
//        }
//    }
//    // Sort files in ascending order of time stamp
//    std::sort(files.begin(), files.end(), less_than_img());
//}
//
//// 生成lidar map
//void importLidarMap(std::string _data_file, std::vector<std::string> _lidar_files){
//    ifstream file;
//    for (auto file_name:_lidar_files) {
//        string current_file_name = _data_file + "/sensor_data/Ouster" +
//                                   "/" + file_name + ".bin";
//        pcl::PointCloud<pcl::PointXYZI> ref_cloud;
//        ref_cloud.clear();
//        file.open(current_file_name, ios::in | ios::binary);
//        int k = 0;
//        while (!file.eof()) {
//            PointType point;
//            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
//            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
//            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
//            file.read(reinterpret_cast<char *>(&point.intensity),
//                      sizeof(float));
////                point.ring = (k % 64) + 1;
//            k = k + 1;
//
//            Eigen::Vector3d temp_point(point.x, point.y, point.z);
//            temp_point = lidar2radar_T.rotationMatrix() * temp_point;
//            point.x = temp_point[0];
//            point.y = temp_point[1];
//            point.z = 2; // temp_point[2];
//            if (temp_point[2] > -0.5)
//                ref_cloud.points.push_back(point);
//        }
//        file.close();
//        Eigen::MatrixXd lidarContext = pcl2Scancontex(ref_cloud);
////        _lidarMapVector.push_back(lidarContext);
//        cv::Mat cvLidarMap;
//        cv::eigen2cv(lidarContext, cvLidarMap);
//        cv::imwrite("/home/myk/dev/study/radar/yeti_odometry/output/radarScancontext/"+file_name+".png", cvLidarMap);
//        cout << "write " << file_name << endl;
//    }
//    cout << "===finish import map===" << endl;
//}
//
//std::vector<std::string> lidar_files;
//
int main(){}
//
//    tPose6D base2radar(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.9 / 180.0 * M_PI);
//    Eigen::Matrix3d rotation_matrix1, rotation_matrix2;
//    rotation_matrix1 = Eigen::AngleAxisd(base2radar.yaw, Eigen::Vector3d::UnitZ()) *
//                       Eigen::AngleAxisd(base2radar.pitch, Eigen::Vector3d::UnitY()) *
//                       Eigen::AngleAxisd(base2radar.roll, Eigen::Vector3d::UnitX());
//    Sophus::SE3d TEMP(rotation_matrix1, Eigen::Vector3d(base2radar.x, base2radar.y, base2radar.z));
//    Sophus::SE3d TEMP2 = tPose6D2SE3(base2radar);
//    base2radar_T = TEMP2;
//    base2lidar_T = tPose6D2SE3(base2lidar);
//    lidar2radar_T = base2radar_T * base2lidar_T.inverse();  // todo 校验tPose6D2SE3有没有问题
//
//    // lidar文件名
//    readLiadarfilenames("/media/myk/ubuntu18/bakup-18/dataset/MulRan/Kaist03/sensor_data/Ouster", lidar_files, "bin");
//
//
//    cout << "hh" << endl;
//}
