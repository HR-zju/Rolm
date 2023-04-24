//
// Created by myk on 2021/12/9.
//

#include "utils.h"

float xy2theta(const float &_x, const float &_y) {
    if ((_x >= 0) & (_y >= 0))
        return (180 / M_PI) * atan(_y / _x);

    if ((_x < 0) & (_y >= 0))
        return 180 - ((180 / M_PI) * atan(_y / (-_x)));

    if ((_x < 0) & (_y < 0))
        return 180 + ((180 / M_PI) * atan(_y / _x));

    if ((_x >= 0) & (_y < 0))
        return 360 - ((180 / M_PI) * atan((-_y) / _x));
}// xy2theta

// scancontext
Eigen::MatrixXd pcl2Scancontex(const pcl::PointCloud<pcl::PointXYZI> _cloud) {
    cv::Mat temp_radar;
    int num_point = _cloud.points.size();
    const int NO_POINT = 0;
    Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
    pcl::PointXYZI pt;
    float azim_angle, azim_range;  // wihtin 2d plane
    int ring_idx, sctor_idx;

    for (int pt_idx = 0; pt_idx < num_point; pt_idx++) {
        pt.x = _cloud.points[pt_idx].x;
        pt.y = _cloud.points[pt_idx].y;
        pt.z = _cloud.points[pt_idx].z + LIDAR_HEIGHT;// naive adding is ok (all points should be > 0).
        pt.intensity = _cloud.points[pt_idx].intensity;
        // xyz to ring, sector
        azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
        azim_angle = xy2theta(pt.x, pt.y);

        // if range is out of roi, pass
        if (azim_range > PC_MAX_RADIUS || azim_range < 2)
            continue;

        //  ceil(azim_range / 80 * 20) 和 20 的较小值   结果 和 1 取大的
        ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
        sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

        // cal the density
        desc(ring_idx - 1, sctor_idx - 1)++;
    }
    for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
        for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
            if (desc(row_idx, col_idx) == NO_POINT)
                desc(row_idx, col_idx) = 0;
    // normalized
    Eigen::MatrixXd::Index maxRow, maxCol;
    double max_dens = desc.maxCoeff(&maxRow, &maxCol);
    desc = 255.0 / max_dens * desc;

    return desc;
}

void pcl2Scancontex(const pcl::PointCloud<pcl::PointXYZI> _cloud, Eigen::MatrixXd &scan_result) {
  cv::Mat temp_radar;
  int num_point = _cloud.points.size();
  const int NO_POINT = 0;
  Eigen::MatrixXd desc = NO_POINT * Eigen::MatrixXd::Ones(PC_NUM_RING, PC_NUM_SECTOR);
  pcl::PointXYZI pt;
  float azim_angle, azim_range;  // wihtin 2d plane
  int ring_idx, sctor_idx;

  for (int pt_idx = 0; pt_idx < num_point; pt_idx++) {
    pt.x = _cloud.points[pt_idx].x;
    pt.y = _cloud.points[pt_idx].y;
    pt.z = _cloud.points[pt_idx].z + LIDAR_HEIGHT;// naive adding is ok (all points should be > 0).
    pt.intensity = _cloud.points[pt_idx].intensity;
    // xyz to ring, sector
    azim_range = sqrt(pt.x * pt.x + pt.y * pt.y);
    azim_angle = xy2theta(pt.x, pt.y);

    // if range is out of roi, pass
    if (azim_range > PC_MAX_RADIUS || azim_range < 2)
      continue;

    //  ceil(azim_range / 80 * 20) 和 20 的较小值   结果 和 1 取大的
    ring_idx = std::max(std::min(PC_NUM_RING, int(ceil((azim_range / PC_MAX_RADIUS) * PC_NUM_RING))), 1);
    sctor_idx = std::max(std::min(PC_NUM_SECTOR, int(ceil((azim_angle / 360.0) * PC_NUM_SECTOR))), 1);

    // cal the density
    desc(ring_idx - 1, sctor_idx - 1)++;
  }
  for (int row_idx = 0; row_idx < desc.rows(); row_idx++)
    for (int col_idx = 0; col_idx < desc.cols(); col_idx++)
      if (desc(row_idx, col_idx) == NO_POINT)
        desc(row_idx, col_idx) = 0;
  // normalized
  Eigen::MatrixXd::Index maxRow, maxCol;
  double max_dens = desc.maxCoeff(&maxRow, &maxCol);
  desc = 255.0 / max_dens * desc;

//  return desc;
  scan_result = desc;
}

///////// scancontext ==========
Eigen::MatrixXd makeSectorkeyFromScancontext(Eigen::MatrixXd &_desc) {
    /*
     * summary: columnwise mean vector
    */
    Eigen::MatrixXd variant_key(1, _desc.cols());
    // 按列搜索
    for (int col_idx = 0; col_idx < _desc.cols(); col_idx++) {
        Eigen::MatrixXd curr_col = _desc.col(col_idx);// 其中某一列
        variant_key(0, col_idx) = curr_col.mean();    // 中值
    }

    return variant_key;
}// SCManager::makeSectorkeyFromScancontext

Eigen::MatrixXd circshift(Eigen::MatrixXd &_mat, int _num_shift) {
    // shift columns to right direction
    assert(_num_shift >= 0);

    if (_num_shift == 0) {
        Eigen::MatrixXd shifted_mat(_mat);
        return shifted_mat;// Early return
    }

    Eigen::MatrixXd shifted_mat = Eigen::MatrixXd::Zero(_mat.rows(), _mat.cols());
    for (int col_idx = 0; col_idx < _mat.cols(); col_idx++) {
        int new_location = (col_idx + _num_shift) % _mat.cols();
        shifted_mat.col(new_location) = _mat.col(col_idx);
    }

    return shifted_mat;

}// circshift

int fastAlignUsingVkey(Eigen::MatrixXd &_vkey1, Eigen::MatrixXd &_vkey2) {
    int argmin_vkey_shift = 0;
    double min_veky_diff_norm = 10000000;
    for (int shift_idx = 0; shift_idx < _vkey1.cols(); shift_idx++) {
        Eigen::MatrixXd vkey2_shifted = circshift(_vkey2, shift_idx);

        Eigen::MatrixXd vkey_diff = _vkey1 - vkey2_shifted;

        double cur_diff_norm = vkey_diff.norm();
        if (cur_diff_norm < min_veky_diff_norm) {
            argmin_vkey_shift = shift_idx;
            min_veky_diff_norm = cur_diff_norm;
        }
    }

    return argmin_vkey_shift;

}// fastAlignUsingVkey

double distDirectSC(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2) {
    int num_eff_cols = 0;// i.e., to exclude all-nonzero sector
    double sum_sector_similarity = 0;
    for (int col_idx = 0; col_idx < _sc1.cols(); col_idx++) {
        Eigen::VectorXd col_sc1 = _sc1.col(col_idx);
        Eigen::VectorXd col_sc2 = _sc2.col(col_idx);

        if ((col_sc1.norm() == 0) | (col_sc2.norm() == 0))// 这条scan没扫描到什么
            continue;                                     // don't count this sector pair.

        // sc1 和 sc2 的夹角 除以（两个模长）
        double sector_similarity = col_sc1.dot(col_sc2) / (col_sc1.norm() * col_sc2.norm());

        sum_sector_similarity = sum_sector_similarity + sector_similarity;
        num_eff_cols = num_eff_cols + 1;
    }

    double sc_sim = sum_sector_similarity / num_eff_cols;
    return 1.0 - sc_sim;

}// distDirectSC

std::pair<double, int> distanceBtnScanContext(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2) {
    // 1. fast align using variant key (not in original IROS18)
    Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
    Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
    int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);// 需要偏移的id
    // 用每列的中值进行快速对齐

    // 四舍五入
    const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols());// a half of search range 并且还只拿了10%
    std::vector<int> shift_idx_search_space{argmin_vkey_shift};       // 用中值算出来的初始偏移
    for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++) {
        shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
        shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
    }
    std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

    // 2. fast columnwise diff  id = shift_idx_search_space
    int argmin_shift = 0;
    double min_sc_dist = 10000000;
    for (int num_shift : shift_idx_search_space) {
        Eigen::MatrixXd sc2_shifted = circshift(_sc2, num_shift);// 将_sc2移动num_shift列
        double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
        if (cur_sc_dist < min_sc_dist) {
            argmin_shift = num_shift;
            min_sc_dist = cur_sc_dist;
        }
    }

    return make_pair(min_sc_dist, argmin_shift);

}// distanceBtnScanContext

std::pair<double, int> distanceBtnCartScanContext(Eigen::MatrixXd &_sc1, Eigen::MatrixXd &_sc2) {
  // 1. fast align using variant key (not in original IROS18)
  Eigen::MatrixXd vkey_sc1 = makeSectorkeyFromScancontext(_sc1);
  Eigen::MatrixXd vkey_sc2 = makeSectorkeyFromScancontext(_sc2);
  int argmin_vkey_shift = fastAlignUsingVkey(vkey_sc1, vkey_sc2);// 需要偏移的id
  // 用每列的中值进行快速对齐

  // 四舍五入
  const int SEARCH_RADIUS = round(0.5 * SEARCH_RATIO * _sc1.cols());// a half of search range 并且还只拿了10%
  std::vector<int> shift_idx_search_space{argmin_vkey_shift};       // 用中值算出来的初始偏移
  for (int ii = 1; ii < SEARCH_RADIUS + 1; ii++) {
    shift_idx_search_space.push_back((argmin_vkey_shift + ii + _sc1.cols()) % _sc1.cols());
    shift_idx_search_space.push_back((argmin_vkey_shift - ii + _sc1.cols()) % _sc1.cols());
  }
  std::sort(shift_idx_search_space.begin(), shift_idx_search_space.end());

  // 2. fast columnwise diff  id = shift_idx_search_space
  int argmin_shift = 0;
  double min_sc_dist = 10000000;
  for (int num_shift : shift_idx_search_space) {
    Eigen::MatrixXd sc2_shifted = circshift(_sc2, num_shift);// 将_sc2移动num_shift列
    double cur_sc_dist = distDirectSC(_sc1, sc2_shifted);
    if (cur_sc_dist < min_sc_dist) {
      argmin_shift = num_shift;
      min_sc_dist = cur_sc_dist;
    }
  }

  return make_pair(min_sc_dist, argmin_shift);

}// distanceBtnScanContext

// cartscan
void cloud2densMap(const pcl::PointCloud<pcl::PointXYZI>::Ptr _point_cloud, cv::Mat &_des_map, const cv::Point2d _center){
  for (auto points:_point_cloud->points) {
    if (points.x*points.x+points.y*points.y>=PC_MAX_RADIUS*PC_MAX_RADIUS) continue;
    cv::Point2d pos_in_img = _center + cv::Point2d(points.x,points.y);
    _des_map.at<double>(floor(pos_in_img.x), floor(pos_in_img.y)) += 1.0;
  }
//  double *max_dens, *min_dens;
//  cv::minMaxIdx(_des_map, max_dens, min_dens);
//  double min_dens_D = *min_dens;
//  _des_map *= 255.0;
//  _des_map = _des_map / min_dens_D;

  // normalized
  Eigen::MatrixXd EdesMap;
  cv::cv2eigen(_des_map, EdesMap);
  Eigen::MatrixXd::Index maxRow, maxCol;
  double max_dens = EdesMap.maxCoeff(&maxRow, &maxCol);
  _des_map = 255.0 / max_dens * _des_map;

}
