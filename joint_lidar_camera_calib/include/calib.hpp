#ifndef CALIB_HPP
#define CALIB_HPP

#include <stdlib.h>
#include <cstdlib>
#include <math.h>
#include <cmath>
#include <map>
#include <unordered_map>
#include <set>
#include <vector>
#include <algorithm>
#include <numeric>

#include <chrono>
#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <std_msgs/Header.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include "ceres/ceres.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

struct vFeature
{
    cv::Point2d p2d;
    int id_3d = 0;
};

struct vPoint
{
    Eigen::Vector3d p3d = Eigen::Vector3d::Zero();
    Eigen::Vector3i color = Eigen::Vector3i::Zero();
    int id = 0;
    bool to_optimize = true;
    Eigen::Matrix3d Hessian = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
};

struct frame
{
    // camera
    cv::Mat image;
    Eigen::Quaterniond C_R; // converts a point from world frame to local frame
    Eigen::Vector3d C_t;
    std::vector<vFeature> vFeatures;

    // LiDAR
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Quaterniond L_R; // different from camera part, converts a point from local frame to world frame
    Eigen::Vector3d L_t;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_query;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree = pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>());
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;
};

class Calib
{
public:
    std::string data_folder = "";
    int frame_num = 0;

    std::vector<frame> frames;
    std::vector<vPoint> vPoints;
    std::map<int, vPoint>map_id_vPoint;
    pcl::PointCloud<pcl::PointXYZRGB> SfM_cloud;

    std::vector<double> Intrin_init = std::vector<double>(9, 0.0);
    Eigen::Matrix3d R_init = Eigen::Matrix3d::Zero();
    Eigen::Vector3d t_init = Eigen::Vector3d::Zero();

    std::vector<double> Intrin_opt = std::vector<double>(9, 0.0);
    Eigen::Matrix3d R_opt = Eigen::Matrix3d::Zero();
    Eigen::Vector3d t_opt = Eigen::Vector3d::Zero();

    bool configuration_loaded = false;
    bool data_loaded = false;

    // Confuguration parameters
    int nearest_neighbor_num = 0;
    double eigen_ratio_thre = 0.0;
    double point_to_plane_dist_thre = 0.0;
    double alpha_sqrt = 0.0;
    bool keep_intrinsic_fixed = false;
    bool save_point_cloud_result = true;
    bool save_calib_result = true;
    bool colorization = true;

    std::string camera_model = "";
    int intrinsic_num = 0;

    Calib(const std::string & data_folder_);
    
    bool load_configuration();

    void print_configuration();

    bool load_data();

    bool load_SfM_data();

    bool load_LiDAR_poses();

    void print_data_information();

    void print_initial_calibration();

    void save_SfM_cloud(const std::string & suffix, 
                        const bool & convert_to_LiDAR_frame,
                        const std::string & extrinsic_type);

    void save_LiDAR_BA_cloud();

    void recover_visual_scale();

    bool calc_plane(pcl::PointCloud<pcl::PointXYZ> & cloud,
                    Eigen::Vector3d & center,
                    Eigen::Vector3d & normal,
                    double & d,
                    double & thickness,
                    double & eigen_ratio_thre,
                    double & eigen_ratio);

    void refine_scale_ext();

    void calculate_point_covariance();

    void joint_calib();

    void print_optimized_calibration();

    void save_opt_calib();

    void colorize_point_cloud(cv::Mat & img, 
                              pcl::PointCloud<pcl::PointXYZ> & cloud_raw,
                              pcl::PointCloud<pcl::PointXYZRGB> & cloud_colorized,
                              std::vector<double> & intrinsics,
                              Eigen::Matrix3d & R,
                              Eigen::Vector3d & t);

    void colorize();
};

struct PointPlaneError1
{
public:
    PointPlaneError1(Eigen::Matrix3d R_,
                    Eigen::Vector3d t_,
                    Eigen::Vector3d point_, 
                    Eigen::Vector3d center_, 
                    Eigen::Vector3d normal_) 
    { 
        point = point_;
        center = center_;
        normal = normal_; 
        R = R_;
        t = t_;
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, const T * scale, T *residuals) const 
    {
        Eigen::Quaternion<T> q_incre{_q[0], _q[1], _q[2], _q[3]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};

        Eigen::Matrix<T, 3, 1> norm{T(normal[0]), T(normal[1]), T(normal[2])};
        Eigen::Matrix<T, 3, 1> center_point{T(center[0]), T(center[1]), T(center[2])};
        Eigen::Matrix<T, 3, 1> p_c{T(point[0]), T(point[1]), T(point[2])};
        Eigen::Matrix<T, 3, 1> p_l_0 = q_incre.toRotationMatrix() * scale[0] * p_c + t_incre;
        Eigen::Matrix<T, 3, 1> p_l_i = R * p_l_0 + t;
        Eigen::Matrix<T, 3, 1> diff = p_l_i - center_point;

        residuals[0] = norm.dot(diff);

        return true;
    }
    static ceres::CostFunction *Create(Eigen::Matrix3d R_,
                                       Eigen::Vector3d t_,
                                       Eigen::Vector3d point_, 
                                       Eigen::Vector3d center_, 
                                       Eigen::Vector3d normal_) 
    {
        return (new ceres::AutoDiffCostFunction<PointPlaneError1, 1, 4, 3, 1>
               (new PointPlaneError1(R_, t_, point_, center_, normal_)));
    }

private:
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    Eigen::Vector3d point;
    Eigen::Vector3d center; 
    Eigen::Vector3d normal;    
};

struct PointPlaneError2
{
public:
    PointPlaneError2(Eigen::Vector3d center_, 
                     Eigen::Vector3d normal_,
                     double scale_) 
    { 
        center = center_;
        normal = normal_; 
        scale = scale_;
    }

    template <typename T>
    bool operator()(const T *_q1, const T *_t1, const T *_q2, const T *_t2, const T *_point, T *residuals) const 
    {
        Eigen::Matrix<T, 3, 1> norm{T(normal[0]), T(normal[1]), T(normal[2])};
        Eigen::Matrix<T, 3, 1> center_point{T(center[0]), T(center[1]), T(center[2])};
        Eigen::Matrix<T, 3, 1> p_c_0(_point[0], _point[1], _point[2]);

        Eigen::Quaternion<T> q_incre1{_q1[0], _q1[1], _q1[2], _q1[3]};
        Eigen::Matrix<T, 3, 1> t_incre1{_t1[0], _t1[1], _t1[2]};
        Eigen::Quaternion<T> q_incre2{_q2[0], _q2[1], _q2[2], _q2[3]};
        Eigen::Matrix<T, 3, 1> t_incre2{_t2[0], _t2[1], _t2[2]};

        Eigen::Matrix<T, 3, 1> p_c_i = q_incre1.toRotationMatrix() * p_c_0 + t_incre1;
        Eigen::Matrix<T, 3, 1> p_l_i = q_incre2.toRotationMatrix() * p_c_i + t_incre2;

        Eigen::Matrix<T, 3, 1> diff = p_l_i - center_point;

        residuals[0] = norm.dot(diff) / scale;

        return true;
    }
    static ceres::CostFunction *Create(Eigen::Vector3d center_, 
                                       Eigen::Vector3d normal_,
                                       double scale_) 
    {
        return (new ceres::AutoDiffCostFunction<PointPlaneError2, 1, 4, 3, 4, 3, 3>
                (new PointPlaneError2(center_, normal_, scale_)));
    }

private:
    Eigen::Vector3d center; 
    Eigen::Vector3d normal;     
    double scale;
};

struct FeatureReprojectionError1 {
    FeatureReprojectionError1(double obs_x, double obs_y, Eigen::Matrix3d R, Eigen::Vector3d t,
                              double f, double cx, double cy, double k1, double k2)
                            : obs_x(obs_x), obs_y(obs_y), R(R), t(t), 
                              f(f), cx(cx), cy(cy), k1(k1), k2(k2) {}
    template <typename T>
    bool operator()(const T *_point, T *residuals) const 
    {
        Eigen::Matrix<T, 3, 1> p_world(_point[0], _point[1], _point[2]);
        Eigen::Matrix<T, 3, 1> p_local = R * p_world + t;

        T xo = p_local[0] / p_local[2];
        T yo = p_local[1] / p_local[2];
        T r2 = xo * xo + yo * yo;
        T r4 = r2 * r2;
        T distortion = 1.0 + k1 * r2 + k2 * r4;

        T xd = xo * distortion;
        T yd = yo * distortion;    
        T ud = f * xd + cx;
        T vd = f * yd + cy;

        residuals[0] = ud - obs_x;
        residuals[1] = vd - obs_y;

        return true;
    }

    static ceres::CostFunction* Create(const double obs_x,
                                       const double obs_y,
                                       const Eigen::Matrix3d R,
                                       const Eigen::Vector3d t,
                                       const double f,
                                       const double cx,
                                       const double cy,
                                       const double k1,
                                       const double k2) {
        return (new ceres::AutoDiffCostFunction<FeatureReprojectionError1, 2, 3>(
                new FeatureReprojectionError1(obs_x, obs_y, R, t, f, cx, cy, k1, k2)));
    }

    double obs_x;
    double obs_y;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    double f;
    double cx;
    double cy;
    double k1;
    double k2;
};

struct FeatureReprojectionError2 {
    FeatureReprojectionError2(double obs_x, double obs_y)
                            : obs_x(obs_x), obs_y(obs_y) {}
    template <typename T>
    bool operator()(const T *_q, const T *_t, const T *_camera, const T *_point, T *residuals) const 
    {
        Eigen::Quaternion<T> q_incre{_q[0], _q[1], _q[2], _q[3]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};

        Eigen::Matrix<T, 3, 1> p_world(_point[0], _point[1], _point[2]);
        Eigen::Matrix<T, 3, 1> p_local = q_incre.toRotationMatrix() * p_world + t_incre;

        T xo = p_local[0] / p_local[2];
        T yo = p_local[1] / p_local[2];
        T r2 = xo * xo + yo * yo;
        T r4 = r2 * r2;
        T distortion = 1.0 + _camera[3] * r2 + _camera[4] * r4;

        T xd = xo * distortion;
        T yd = yo * distortion; 
        T ud = _camera[0] * xd + _camera[1];
        T vd = _camera[0] * yd + _camera[2];

        residuals[0] = ud - obs_x;
        residuals[1] = vd - obs_y;

        return true;
    }

    static ceres::CostFunction* Create(const double obs_x,
                                       const double obs_y) {
        return (new ceres::AutoDiffCostFunction<FeatureReprojectionError2, 2, 4, 3, 5, 3>(
                new FeatureReprojectionError2(obs_x, obs_y)));
    }

    double obs_x;
    double obs_y;
};

Calib::Calib(const std::string & data_folder_)
{
    data_folder = data_folder_;
    if (data_folder.back() != '/')
    {
        data_folder += "/";
    }

    if (!load_configuration())
    {
        std::string msg = "Check configuration file.";
        ROS_ERROR_STREAM(msg.c_str());
        exit(-1);
    }
    print_configuration();

    if (!load_data())
    {
        std::string msg = "Check data files.";
        ROS_ERROR_STREAM(msg.c_str());
        exit(-1);
    }
    print_data_information();

    print_initial_calibration();
};

bool Calib::load_configuration()
{
    std::string config_file_path = data_folder + "config/config.yaml";
    cv::FileStorage config_file(config_file_path, cv::FileStorage::READ);
    if (!config_file.isOpened())
    {
        std::string msg = "Unable to open " + config_file_path;
        ROS_ERROR_STREAM(msg.c_str());
        return false;
    }

    config_file["frame_num"] >> frame_num;
    config_file["nearest_neighbor_num"] >> nearest_neighbor_num;
    config_file["eigen_ratio_thre"] >> eigen_ratio_thre;
    config_file["point_to_plane_dist_thre"] >> point_to_plane_dist_thre;
    config_file["alpha_sqrt"] >> alpha_sqrt;
    config_file["keep_intrinsic_fixed: false"] >> keep_intrinsic_fixed;
    config_file["save_point_cloud_result"] >> save_point_cloud_result;
    config_file["save_calib_result"] >> save_calib_result;
    config_file["colorization"] >> colorization;

    cv::Mat R_init_mat;
    config_file["R_init"] >> R_init_mat;
    R_init << R_init_mat.at<double>(0, 0), R_init_mat.at<double>(0, 1), R_init_mat.at<double>(0, 2),
              R_init_mat.at<double>(1, 0), R_init_mat.at<double>(1, 1), R_init_mat.at<double>(1, 2),
              R_init_mat.at<double>(2, 0), R_init_mat.at<double>(2, 1), R_init_mat.at<double>(2, 2);

    cv::Mat t_init_mat;
    config_file["t_init"] >> t_init_mat;
    t_init << t_init_mat.at<double>(0, 0), t_init_mat.at<double>(1, 0), t_init_mat.at<double>(2, 0);

    configuration_loaded = true;
    return true;
};

void Calib::print_configuration()
{
    std::string msg;
    if (!configuration_loaded)
    {
        msg = "Configuration not loaded. Unable to display.";
        ROS_ERROR_STREAM(msg.c_str());
        return;
    }

    msg = "Configuration:";
    ROS_INFO_STREAM(msg.c_str());
    msg = "--frame_num: " + std::to_string(frame_num);
    ROS_INFO_STREAM(msg.c_str());    
    msg = "--nearest_neighbor_num: " + std::to_string(nearest_neighbor_num);
    ROS_INFO_STREAM(msg.c_str());
    msg = "--eigen_ratio_thre: " + std::to_string(eigen_ratio_thre);
    ROS_INFO_STREAM(msg.c_str());  
    msg = "--point_to_plane_dist_thre: " + std::to_string(point_to_plane_dist_thre);
    ROS_INFO_STREAM(msg.c_str());  
    msg = "--alpha_sqrt: " + std::to_string(alpha_sqrt);
    ROS_INFO_STREAM(msg.c_str());
    msg = "--keep_intrinsic_fixed: " + std::to_string(keep_intrinsic_fixed);
    ROS_INFO_STREAM(msg.c_str());    
    msg = "--save_point_cloud_result: " + std::to_string(save_point_cloud_result);
    ROS_INFO_STREAM(msg.c_str());
    msg = "--save_calib_result: " + std::to_string(save_calib_result);
    ROS_INFO_STREAM(msg.c_str());
    msg = "--colorization: " + std::to_string(colorization);
    ROS_INFO_STREAM(msg.c_str());    

    std::cout << std::endl;
};

bool Calib::load_data()
{   
    // Load camera images and LiDAR point clouds
    for (int i = 0; i < frame_num; ++i)
    {
        frame fr;

        std::string img_path = data_folder + "images/" + std::to_string(i) + ".png";
        fr.image = cv::imread(img_path);
        if (fr.image.empty())
        {
            std::string msg = "Unable to open " + img_path;
            ROS_ERROR_STREAM(msg.c_str());
            return false;
        }

        std::string cloud_path = data_folder + "clouds/" + std::to_string(i) + ".pcd";
        if (pcl::io::loadPCDFile(cloud_path, fr.cloud))
        {
            std::string msg = "Unable to open " + cloud_path;
            ROS_ERROR_STREAM(msg.c_str());
            return false;       
        }
        fr.cloud_query = fr.cloud.makeShared();
        fr.kdtree->setInputCloud(fr.cloud_query);
        fr.pointIdxNKNSearch = std::vector<int>((const int)nearest_neighbor_num, 0);
        fr.pointNKNSquaredDistance = std::vector<float>((const int)nearest_neighbor_num, 0.0);

        frames.push_back(fr);
    }

    // Load visual features, points and camera poses
    if (!load_SfM_data())
    {
        std::string msg = "Check SfM files.";
        ROS_ERROR_STREAM(msg.c_str());
        return false;
    }

    // Load LiDAR BA poses
    if (!load_LiDAR_poses())
    {
        std::string msg = "Check LiDAR pose files.";
        ROS_ERROR_STREAM(msg.c_str());
        return false;        
    }

    data_loaded = true;
    return true;
};

bool Calib::load_SfM_data()
{
    std::string cameras_path = data_folder + "SfM/cameras.txt",
                points_path  = data_folder + "SfM/points3D.txt",
                images_path  = data_folder + "SfM/images.txt";

    std::ifstream cameras_file, points_file, images_file; 
    cameras_file.open(cameras_path);
    points_file.open(points_path);
    images_file.open(images_path);
    if (!cameras_file)
    {
        std::string msg = "Unable to open " + cameras_path;
        ROS_ERROR_STREAM(msg.c_str());
        return false;
    }
    if (!points_file)
    {
        std::string msg = "Unable to open " + points_path;
        ROS_ERROR_STREAM(msg.c_str());
        return false;
    }    
    if (!images_file)
    {
        std::string msg = "Unable to open " + images_path;
        ROS_ERROR_STREAM(msg.c_str());
        return false;
    }

    // Camera model and parameters
    while (!cameras_file.eof())
    {
        std::string line;
        std::getline(cameras_file, line);

        if (line[0] == '#')
        {}
        else if (line == "")
        { break;}        
        else
        {
            int camera_id;
            std::string camera_type;
            int width, height;
            std::stringstream word(line);
            word >> camera_id >> camera_type >> width >> height;
            if (camera_type != "RADIAL")
            {
                std::string msg = "Camera type not supported.";
                ROS_ERROR_STREAM(msg.c_str());
                return false;                
            }
            if ((width != frames[0].image.cols) || (height != frames[0].image.rows))
            {
                std::string msg = "SfM image size is not consistent with loaded images.";
                ROS_ERROR_STREAM(msg.c_str());
                return false;                 
            }
            camera_model = camera_type;
            if (camera_type == "RADIAL")
            {
                intrinsic_num = 5;
            }

            word >> Intrin_init[0] >> Intrin_init[2] >> Intrin_init[3] >> Intrin_init[4] >> Intrin_init[5];
            Intrin_init[1] = Intrin_init[0];
        }
    }
    cameras_file.close();

    // Triangulated 3D points
    while (!points_file.eof())
    {
        std::string line;
        std::getline(points_file, line);

        if (line[0] == '#')
        {}
        else if (line == "")
        { break;}
        else
        {
            int point_id;
            double x, y, z;
            int R, G, B;
            double residual;

            std::stringstream word(line);
            word >> point_id >> x >> y >> z >> R >> G >> B >> residual;

            vPoint vP;
            vP.id = point_id;
            vP.p3d << x, y, z;
            vP.color << R, G, B;

            map_id_vPoint[point_id] = vP;
            vPoints.push_back(vP);
        }
    }
    points_file.close();

    // Camera poses and features
    while (!images_file.eof())
    {
        std::string line;
        std::getline(images_file, line);    

        if (line[0] == '#')
        {}
        else if (line == "")
        { break;}
        else
        {
            int image_id;
            double qw, qx, qy, qz, tx, ty, tz;
            int camera_id;
            std::string image_name;

            std::stringstream word1(line);
            word1 >> image_id
                  >> qw >> qx >> qy >> qz >> tx >> ty >> tz
                  >> camera_id
                  >> image_name;
            word1.clear();

            if (image_id > frame_num)
            {
                std::string msg = "SfM image number is not consistent with frame number.";
                ROS_ERROR_STREAM(msg.c_str());
                return false;
            }

            Eigen::Quaterniond q(qw, qx, qy, qz);
            frames[image_id-1].C_R = q;
            frames[image_id-1].C_t << tx, ty, tz;

            line.clear();
            std::getline(images_file, line);

            std::stringstream word2(line);
            while (!word2.eof())
            {
                double u, v;
                int point_id;
                word2 >> u >> v >> point_id;

                if (point_id > -1)
                {
                    vFeature vF;

                    cv::Point2d p2d(u, v);
                    vF.p2d = p2d;
                    vF.id_3d = point_id;
                    frames[image_id-1].vFeatures.push_back(vF);
                }
            }
        }          
    }
    images_file.close();

    // As COLMAP does not set the first camera frame as the world frame, we need CONVERSION.
    Eigen::Matrix4d T0 = Eigen::Matrix4d::Zero();
    Eigen::Matrix3d R0 = frames[0].C_R.toRotationMatrix();
    Eigen::Vector3d t0 = frames[0].C_t;
    T0.block(0, 0, 3, 3) = R0;
    T0.block(0, 3, 3, 1) = t0;
    T0(3, 3) = 1.0;
    for (int i = 0; i < frame_num; ++i)
    {
        Eigen::Matrix3d R1 = frames[i].C_R.toRotationMatrix();
        Eigen::Vector3d t1 = frames[i].C_t;
        Eigen::Matrix4d T1 = Eigen::Matrix4d::Zero();
        T1.block(0, 0, 3, 3) = R1;
        T1.block(0, 3, 3, 1) = t1;
        T1(3, 3) = 1.0;

        Eigen::Matrix4d Ti = T1 * T0.inverse();
        Eigen::Matrix3d Ri = Ti.block(0, 0, 3, 3);
        Eigen::Vector3d ti = Ti.block(0, 3, 3, 1);
        Eigen::Quaterniond qi(Ri);

        frames[i].C_R = qi;
        frames[i].C_t = ti;
    }

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        Eigen::Vector3d p3d_temp = vPoints[i].p3d;
        vPoints[i].p3d = R0 * p3d_temp + t0;
    }

    return true;
};

bool Calib::load_LiDAR_poses()
{
    std::string LiDAR_pose_path = data_folder + "LiDAR_pose/lidar_poses_BA.txt";

    std::ifstream LiDAR_pose_file; 
    LiDAR_pose_file.open(LiDAR_pose_path);
    if (!LiDAR_pose_file)
    {
        std::string msg = "Unable to open " + LiDAR_pose_path;
        ROS_ERROR_STREAM(msg.c_str());
        return false;
    }

    int frame_idx = 0;
    while (!LiDAR_pose_file.eof())
    {
        std::string line;
        std::getline(LiDAR_pose_file, line);

        if (line == "")
        { break;}

        if (frame_idx == frame_num)
        {
            std::string msg = "LiDAR pose number is not consistent with frame number.";
            ROS_ERROR_STREAM(msg.c_str());
            return false;
        }

        double qx, qy, qz, qw, tx, ty, tz;

        std::stringstream word(line);
        word >> qx >> qy >> qz >> qw >> tx >> ty >> tz;  

        Eigen::Quaterniond q(qw, qx, qy, qz);
        q.normalize();
        frames[frame_idx].L_R = q;
        frames[frame_idx].L_t << tx, ty, tz;
        ++frame_idx;
    }
    LiDAR_pose_file.close();

    return true;
};

void Calib::print_data_information()
{
    std::string msg;
    if (!data_loaded)
    {
        msg = "Data not loaded. Unable to display.";
        ROS_ERROR_STREAM(msg.c_str());
        return;
    }

    msg = "Data information:";
    ROS_INFO_STREAM(msg.c_str());

    msg = "--image size: [width, height] = [" + std::to_string(frames[0].image.cols) + ", " + 
                                                std::to_string(frames[0].image.rows) + "]";
    ROS_INFO_STREAM(msg.c_str());

    if (camera_model == "RADIAL")
    {
       msg = "--camera model: " + camera_model + "; fx = fy, cx, cy, k1, k2";
       ROS_INFO_STREAM(msg.c_str());
    }

    std::vector<int> cloud_sizes;
    for (int i = 0; i < frame_num; ++i)
    {
        cloud_sizes.push_back(frames[i].cloud.points.size());
    }
    std::sort(cloud_sizes.begin(), cloud_sizes.end());
    int sum_size = std::accumulate(cloud_sizes.begin(), cloud_sizes.end(), 0);
    double mean_size = double(sum_size) / double(frame_num);
    msg = "--cloud size: [min, avg, max] = [" + std::to_string(cloud_sizes[0]) + ", " +
                                                std::to_string(mean_size) + ", " +
                                                std::to_string(cloud_sizes.back()) + "]";
    ROS_INFO_STREAM(msg.c_str());

    int vPoints_num = vPoints.size();
    msg = "--visual point number: " + std::to_string(vPoints_num) + "; " 
                                    + std::to_string(double(vPoints_num)/double(frame_num)) + " points per image";
    ROS_INFO_STREAM(msg.c_str());

    int vFeatures_num = 0;
    for (int i = 0; i < frame_num; ++i)
    {
        vFeatures_num += frames[i].vFeatures.size();
    }
    msg = "--visual feature number: " + std::to_string(vFeatures_num) + "; " 
                                      + std::to_string(double(vFeatures_num)/double(vPoints_num)) + " features per point";    
    ROS_INFO_STREAM(msg.c_str());

    std::cout << std::endl;
};

void Calib::print_initial_calibration()
{
    std::string msg;
    msg = "Initial calibration parameters:";
    ROS_INFO_STREAM(msg.c_str());

    std::ostringstream R_stream;
    R_stream << "--rotation: [" << std::setprecision(3) <<
                R_init(0,0) << ", " << R_init(0,1) << ", " << R_init(0,2) << "; " <<
                R_init(1,0) << ", " << R_init(1,1) << ", " << R_init(1,2) << "; " << 
                R_init(2,0) << ", " << R_init(2,1) << ", " << R_init(2,2) << "]";
    msg = R_stream.str();
    ROS_INFO_STREAM(msg.c_str());

    std::ostringstream t_stream;
    t_stream << "--translation: [" << std::setprecision(3) <<
                t_init[0] << ", " << t_init[1] << ", " << t_init[2] << "]";
    msg = t_stream.str();
    ROS_INFO_STREAM(msg.c_str());

    std::ostringstream Intrin_stream;
    if (camera_model == "RADIAL")
    {
        Intrin_stream << "--intrinsics: [f, cx, cy, k1, k2] = [" << std::setprecision(4) <<
                         Intrin_init[0] << ", " <<
                         Intrin_init[2] << ", " <<
                         Intrin_init[3] << ", " <<
                         Intrin_init[4] << ", " <<
                         Intrin_init[5] << "]";
    }
    msg = Intrin_stream.str();
    ROS_INFO_STREAM(msg.c_str());

    std::cout << std::endl;
};

void Calib::save_SfM_cloud(const std::string & suffix,
                           const bool & convert_to_LiDAR_frame,
                           const std::string & extrinsic_type)
{
    if (!save_point_cloud_result)
    { return;}

    SfM_cloud.clear();
    SfM_cloud.width = 0;

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        pcl::PointXYZRGB p;
        Eigen::Vector3d p_vec;

        if (convert_to_LiDAR_frame)
        {
            Eigen::Matrix3d R;
            Eigen::Vector3d t;
            if (extrinsic_type == "init")
            {
                R = R_init.inverse();
                t = -R * t_init;
            }
            else
            {
                R = R_opt.inverse();
                t = -R * t_opt;
            }

            p_vec = R * vPoints[i].p3d + t;
        }
        else
        {
            p_vec = vPoints[i].p3d;
        }

        p.x = p_vec[0];
        p.y = p_vec[1];
        p.z = p_vec[2];
        p.r = vPoints[i].color[0];
        p.g = vPoints[i].color[1];
        p.b = vPoints[i].color[2];
        SfM_cloud.points.push_back(p);
    }
    SfM_cloud.width = vPoints.size();

    std::string SfM_cloud_save_path = data_folder + "result/SfM_cloud_" + suffix + ".pcd";
    pcl::io::savePCDFileBinary(SfM_cloud_save_path, SfM_cloud);

    std::string msg;
    msg = "SfM point cloud saved as " + SfM_cloud_save_path;
    ROS_INFO_STREAM(msg.c_str());
};

void Calib::save_LiDAR_BA_cloud()
{
    pcl::PointCloud<pcl::PointXYZ> LiDAR_cloud;
    for (int i = 0; i < frame_num; ++i)
    {
        Eigen::Matrix3d R_i_to_0(frames[i].L_R);
        Eigen::Vector3d t_i_to_0(frames[i].L_t);

        for (size_t j = 0; j < frames[i].cloud.points.size(); ++j)
        {
            pcl::PointXYZ p = frames[i].cloud.points[j];
            Eigen::Vector3d p_i(p.x, p.y, p.z);
            Eigen::Vector3d p_0 = R_i_to_0 * p_i + t_i_to_0;
            p.x = p_0[0]; p.y = p_0[1]; p.z = p_0[2];
            LiDAR_cloud.points.push_back(p);
        }
    }

    std::string LiDAR_cloud_save_path = data_folder + "result/LiDAR_cloud.pcd";
    pcl::io::savePCDFileBinary(LiDAR_cloud_save_path, LiDAR_cloud);

    std::string msg;
    msg = "LiDAR point clouds saved as " + LiDAR_cloud_save_path;
    ROS_INFO_STREAM(msg.c_str());
};

void Calib::recover_visual_scale()
{
    Eigen::Matrix4d T = Eigen::Matrix4d::Zero();
    T.block(0, 0, 3, 3) = R_init;
    T.block(0, 3, 3, 1) = t_init;
    T(3, 3) = 1.0;
    Eigen::Matrix4d T_inv = T.inverse();
    Eigen::Matrix3d R_inv = T_inv.block(0, 0, 3, 3);
    Eigen::Vector3d t_inv = T_inv.block(0, 3, 3, 1); 

    // Construct equation
    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3,3);

    const int N = 3 * frame_num;
    Eigen::MatrixXd B(N, 1);
    Eigen::MatrixXd C(N, 1);

    for (int i = 0; i < frame_num; ++i)
    {
        Eigen::Matrix3d L_R_i_to_0(frames[i].L_R);
        Eigen::Vector3d L_t_i_to_0(frames[i].L_t);
        
        Eigen::Matrix3d C_R_0_to_i(frames[i].C_R);
        Eigen::Vector3d C_t_0_to_i(frames[i].C_t);
        
        Eigen::Matrix4d C_T_0_to_i = Eigen::Matrix4d::Zero();
        C_T_0_to_i.block(0, 0, 3, 3) = C_R_0_to_i;
        C_T_0_to_i.block(0, 3, 3, 1) = C_t_0_to_i;
        C_T_0_to_i(3, 3) = 1.0;
        Eigen::Matrix4d C_T_i_to_0 = C_T_0_to_i.inverse();
        Eigen::Vector3d C_t_i_to_0 = C_T_i_to_0.block(0, 3, 3, 1);

        B.block(i*3, 0, 3, 1) = R_inv * C_t_i_to_0;
        C.block(i*3, 0, 3, 1) = L_t_i_to_0 - (I - L_R_i_to_0) * t_inv;
    }

    // Solve
    Eigen::MatrixXd D = B.transpose() * B;
    Eigen::MatrixXd V = D.inverse() * B.transpose() * C;
    double scale = V(0, 0);
    std::string msg = "Initial visual scale recovered. Value: " + std::to_string(scale);
    ROS_INFO_STREAM(msg.c_str());

    // Update visual points and camera translations
    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        vPoints[i].p3d *= scale;
    }
    for (int i = 0; i < frame_num; ++i)
    {
        frames[i].C_t *= scale;
    }
};

bool Calib::calc_plane(pcl::PointCloud<pcl::PointXYZ> & cloud,
                Eigen::Vector3d & center,
                Eigen::Vector3d & normal,
                double & d,
                double & thickness,
                double & eigen_ratio_thre,
                double & eigen_ratio)
{
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    center = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
    int points_size = cloud.points.size();
    if (points_size < 10)
    { return false;}

    for (int i = 0; i < points_size; ++i)
    {
        pcl::PointXYZ p = cloud.points[i]; 
        Eigen::Vector3d pv;
        pv << p.x, p.y, p.z;
        covariance += pv * pv.transpose();
        center += pv;
    }
    center = center / double(points_size);
    covariance = covariance / double(points_size) - center * center.transpose();

    Eigen::EigenSolver<Eigen::Matrix3d> es(covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;

    eigen_ratio = evalsReal(evalsMid) / evalsReal(evalsMin);
    if (eigen_ratio > eigen_ratio_thre)
    {
        normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin), evecs.real()(2, evalsMin);
        d = -(normal(0) * center(0) + normal(1) * center(1) + normal(2) * center(2));
        thickness = std::sqrt(evalsReal(evalsMin));

        return true;
    }
    else
    { return false;}
};

void Calib::refine_scale_ext()
{
    double ext[7];
    Eigen::Matrix3d R_inv = R_init.inverse();
    Eigen::Vector3d t_inv = -R_inv * t_init;
    Eigen::Quaterniond q_inv(R_inv);
    ext[0] = q_inv.w();
    ext[1] = q_inv.x();
    ext[2] = q_inv.y();
    ext[3] = q_inv.z();
    ext[4] = t_inv[0];
    ext[5] = t_inv[1];
    ext[6] = t_inv[2];

    double scale[1];
    scale[0] = 1.0;

    for (int iter_round = 0; iter_round < 3; ++iter_round)
    {
        ceres::Problem problem;
        ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
        problem.AddParameterBlock(ext, 4, q_parameterization);
        problem.AddParameterBlock(ext + 4, 3); 
        problem.AddParameterBlock(scale, 1);

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;

        for (size_t i = 0; i < vPoints.size(); ++i)
        {
            Eigen::Vector3d p_c(vPoints[i].p3d);
            Eigen::Vector3d p_l_0 = R_inv * scale[0] * p_c + t_inv;

            for (int j = 0; j < frame_num; ++j)
            {
                Eigen::Matrix4d T_li_l0 = Eigen::Matrix4d::Zero();
                Eigen::Matrix3d R_li_l0(frames[j].L_R);
                Eigen::Vector3d t_li_l0(frames[j].L_t);
                T_li_l0.block(0, 0, 3, 3) = R_li_l0;
                T_li_l0.block(0, 3, 3, 1) = t_li_l0;
                T_li_l0(3, 3) = 1.0;

                Eigen::Matrix4d T_l0_li = T_li_l0.inverse();
                Eigen::Matrix3d R_l0_li = T_l0_li.block(0, 0, 3, 3);
                Eigen::Vector3d t_l0_li = T_l0_li.block(0, 3, 3, 1);
                Eigen::Vector3d p_l_i = R_l0_li * p_l_0 + t_l0_li;

                pcl::PointXYZ p_query(p_l_i[0], p_l_i[1], p_l_i[2]);

                if (frames[j].kdtree->nearestKSearch(p_query, nearest_neighbor_num, 
                    frames[j].pointIdxNKNSearch, frames[j].pointNKNSquaredDistance) > 0)
                {
                    // Discard distant points
                    if (iter_round == 0)
                    {
                        if (frames[j].pointNKNSquaredDistance[0] > 1.0)
                        { continue;}                         
                    }
                    else
                    {
                        if (frames[j].pointNKNSquaredDistance[0] > 0.04)
                        { continue;}           
                    }
                   
                    // Only retain points corresponding to a plane
                    pcl::PointCloud<pcl::PointXYZ> nearest_neighbor_cloud;
                    for (size_t k = 0; k < frames[j].pointIdxNKNSearch.size(); ++k)
                    {
                        int idx = frames[j].pointIdxNKNSearch[k];
                        pcl::PointXYZ p = frames[j].cloud_query->points[idx]; 
                        nearest_neighbor_cloud.points.push_back(p);
                    }  
                    Eigen::Vector3d center, normal;
                    double d, thickness, eigen_ratio;
                    if (calc_plane(nearest_neighbor_cloud, center, normal, d, thickness, 
                                   eigen_ratio_thre, eigen_ratio))
                    {                    
                        ceres::CostFunction *cost_function = PointPlaneError1::Create(R_l0_li, t_l0_li, p_c, center, normal);
                        ceres::LossFunction *loss_function = new ceres::HuberLoss(0.2);
                        problem.AddResidualBlock(cost_function, 
                                                 loss_function,
                                                 ext,
                                                 ext + 4,
                                                 scale);                                  
                    }                             
                }                                      
            } 
        }

        ceres::Solve(options, &problem, &summary);

        q_inv.w() = ext[0];
        q_inv.x() = ext[1];
        q_inv.y() = ext[2];
        q_inv.z() = ext[3];
        t_inv << ext[4], ext[5], ext[6];
        R_inv = q_inv.toRotationMatrix();
    }

    // Update visual points and camera translations
    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        vPoints[i].p3d *= scale[0];
    }
    for (int i = 0; i < frame_num; ++i)
    {
        frames[i].C_t *= scale[0];
    }

    R_init = R_inv.inverse();
    t_init = -R_init * t_inv;

    std::string msg = "Iteratively refined visual scale and extrinsic parameters.";
    ROS_INFO_STREAM(msg.c_str());

    calculate_point_covariance();
};

void Calib::calculate_point_covariance()
{
    const int point_param_size = vPoints.size() * 3;
    double points[point_param_size];
    std::map<int, size_t> map_id_index;

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        points[3 * i + 0] = vPoints[i].p3d[0];
        points[3 * i + 1] = vPoints[i].p3d[1];
        points[3 * i + 2] = vPoints[i].p3d[2];
        map_id_index[vPoints[i].id] = i;
    }

    double camera[(const int)intrinsic_num];
    // 'RADIAL' model
    camera[0] = Intrin_init[0];
    camera[1] = Intrin_init[2];
    camera[2] = Intrin_init[3];
    camera[3] = Intrin_init[4];
    camera[4] = Intrin_init[5];

    ceres::Problem problem;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        problem.AddParameterBlock(points + 3 * i, 3);
    }

    // Construct visual reprojection residuals
    for (size_t i = 0; i < frames.size(); ++i)
    {
        for (size_t j = 0; j < frames[i].vFeatures.size(); ++j)
        {
            cv::Point2d p2d = frames[i].vFeatures[j].p2d;
            double obs_x = p2d.x;
            double obs_y = p2d.y;

            size_t p_idx = map_id_index[frames[i].vFeatures[j].id_3d];
            Eigen::Vector3d _point(points[3 * p_idx + 0], points[3 * p_idx + 1], points[3 * p_idx + 2]);
            Eigen::Matrix3d _R(frames[i].C_R);
            Eigen::Vector3d _t(frames[i].C_t);          

            ceres::CostFunction *cost_function = FeatureReprojectionError1::Create(obs_x, obs_y, _R, _t,
                                                                                   camera[0],
                                                                                   camera[1],
                                                                                   camera[2],
                                                                                   camera[3],
                                                                                   camera[4]);
            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     points + 3 * p_idx);
        }
    }

    ceres::Solve(options, &problem, &summary);  

    double loss = 0.0;
    ceres::CRSMatrix Jacobian;
    problem.Evaluate(ceres::Problem::EvaluateOptions(), &loss, NULL, NULL, &Jacobian);
    for (int i = 0; i < Jacobian.num_rows; ++i)
    {
        Eigen::Vector3d V(Jacobian.values[i * 3 + 0], Jacobian.values[i * 3 + 1], Jacobian.values[i * 3 + 2]);
        Eigen::Matrix3d H = V * V.transpose();
        int j = Jacobian.cols[i * 3 + 0] / 3;
        vPoints[j].Hessian += H;     
    }
    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        if (vPoints[i].Hessian(0, 0) != 0)
        {
            vPoints[i].covariance = vPoints[i].Hessian.inverse();
            if (std::isnan(vPoints[i].covariance(0, 0)) || std::isinf(vPoints[i].covariance(0, 0)))
            { vPoints[i].to_optimize = false;}
        }
    }

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        vPoints[i].p3d[0] = points[3 * i + 0];
        vPoints[i].p3d[1] = points[3 * i + 1];
        vPoints[i].p3d[2] = points[3 * i + 2];
    }    
};

void Calib::joint_calib()
{
    const int point_param_size = vPoints.size() * 3;
    double points[point_param_size];
    std::map<int, size_t> map_id_index;

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        points[3 * i + 0] = vPoints[i].p3d[0];
        points[3 * i + 1] = vPoints[i].p3d[1];
        points[3 * i + 2] = vPoints[i].p3d[2];
        map_id_index[vPoints[i].id] = i;
    }

    double camera[(const int)intrinsic_num];
    // 'RADIAL' model
    camera[0] = Intrin_init[0];
    camera[1] = Intrin_init[2];
    camera[2] = Intrin_init[3];
    camera[3] = Intrin_init[4];
    camera[4] = Intrin_init[5];

    const int pose_param_size = frame_num * 7;
    double poses[pose_param_size];
    for (int i = 0; i < frame_num; ++i)
    {
        poses[7 * i + 0] = frames[i].C_R.w();
        poses[7 * i + 1] = frames[i].C_R.x();
        poses[7 * i + 2] = frames[i].C_R.y();
        poses[7 * i + 3] = frames[i].C_R.z();
        poses[7 * i + 4] = frames[i].C_t[0];
        poses[7 * i + 5] = frames[i].C_t[1];
        poses[7 * i + 6] = frames[i].C_t[2];
    }

    double ext[7];
    Eigen::Matrix3d R_inv = R_init.inverse();
    Eigen::Vector3d t_inv = -R_inv * t_init;
    Eigen::Quaterniond q_inv(R_inv);
    ext[0] = q_inv.w();
    ext[1] = q_inv.x();
    ext[2] = q_inv.y();
    ext[3] = q_inv.z();
    ext[4] = t_inv[0];
    ext[5] = t_inv[1];
    ext[6] = t_inv[2];

    ceres::Problem problem;
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    ceres::Solver::Summary summary; 

    problem.AddParameterBlock(camera, 5);
    problem.AddParameterBlock(ext, 4, q_parameterization);
    problem.AddParameterBlock(ext + 4, 3);

    if (keep_intrinsic_fixed == true)
    {
        problem.SetParameterBlockConstant(camera);
    }
    for (int i = 0; i < frame_num; ++i)
    {
        problem.AddParameterBlock(poses + 7 * i + 0, 4, q_parameterization);
        problem.AddParameterBlock(poses + 7 * i + 4, 3);

        if (i == 0)
        {
            problem.SetParameterBlockConstant(poses);
            problem.SetParameterBlockConstant(poses + 4);         
        }     
    }      
    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        problem.AddParameterBlock(points + 3 * i, 3);
    }

    // Construct point-to-plane residual  
    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        if (vPoints[i].to_optimize == false)
        { continue;}

        Eigen::Quaterniond q_c_l;
        q_c_l.w() = ext[0];
        q_c_l.x() = ext[1];
        q_c_l.y() = ext[2];
        q_c_l.z() = ext[3];
        Eigen::Matrix3d R_c_l(q_c_l);
        Eigen::Vector3d t_c_l(ext[4], ext[5], ext[6]);

        Eigen::Matrix4d T_c_l = Eigen::Matrix4d::Zero();
        T_c_l.block(0, 0, 3, 3) = R_c_l;
        T_c_l.block(0, 3, 3, 1) = t_c_l;
        T_c_l(3, 3) = 1.0;

        Eigen::Vector4d p_c_0(points[3 * i + 0], points[3 * i + 1], points[3 * i + 2], 1.0);

        int corr_plane_num = 0;
        for (int j = 0; j < frame_num; ++j)
        {
            Eigen::Quaterniond q_c0_ci;
            q_c0_ci.w() = poses[7 * j + 0];
            q_c0_ci.x() = poses[7 * j + 1];
            q_c0_ci.y() = poses[7 * j + 2];
            q_c0_ci.z() = poses[7 * j + 3];
            Eigen::Matrix3d R_c0_ci(q_c0_ci);
            Eigen::Vector3d t_c0_ci(poses[7 * j + 4], poses[7 * j + 5], poses[7 * j + 6]);

            Eigen::Matrix4d T_c0_ci = Eigen::Matrix4d::Zero();
            T_c0_ci.block(0, 0, 3, 3) = R_c0_ci;
            T_c0_ci.block(0, 3, 3, 1) = t_c0_ci;
            T_c0_ci(3, 3) = 1.0;       

            Eigen::Matrix4d T_c0_li = T_c_l * T_c0_ci;
            Eigen::Matrix3d R_c0_li = T_c0_li.block(0, 0, 3, 3);

            Eigen::Vector4d p_l_i = T_c0_li * p_c_0;

            pcl::PointXYZ p_query(p_l_i[0], p_l_i[1], p_l_i[2]);

            if (frames[j].kdtree->nearestKSearch(p_query, nearest_neighbor_num,
                frames[j].pointIdxNKNSearch, frames[j].pointNKNSquaredDistance) > 0)
            {
                // Discard distant points
                if (frames[j].pointNKNSquaredDistance[0] > (point_to_plane_dist_thre * point_to_plane_dist_thre))
                {
                    continue;
                }              

                // Only retain points corresponding to a plane
                pcl::PointCloud<pcl::PointXYZ> nearest_neighbor_cloud;
                for (size_t k = 0; k < frames[j].pointIdxNKNSearch.size(); ++k)
                {
                    int idx = frames[j].pointIdxNKNSearch[k];
                    pcl::PointXYZ p = frames[j].cloud_query->points[idx]; 
                    nearest_neighbor_cloud.points.push_back(p);
                }  
                Eigen::Vector3d center, normal;
                double d, thickness, eigen_ratio;
                if (calc_plane(nearest_neighbor_cloud, center, normal, d, thickness, 
                               eigen_ratio_thre, eigen_ratio))
                {
                    double variance = std::abs(normal.transpose() * R_c0_li * vPoints[i].covariance * R_c0_li.transpose() * normal);
                    double scale = alpha_sqrt * std::sqrt(variance); 
                
                    ceres::CostFunction *cost_function = PointPlaneError2::Create(center, normal, scale);
                    problem.AddResidualBlock(cost_function,
                                             NULL,
                                             poses + 7 * j,
                                             poses + 7 * j + 4,
                                             ext, 
                                             ext + 4,
                                             points + 3 * i);
                    ++corr_plane_num;                                           
                }
            }                       
        }

        // Discard points without any corresponding plane
        if (corr_plane_num == 0)
        { vPoints[i].to_optimize = false;}
    }

    // Construct visual reprojection residual
    for (int i = 0; i < frame_num; ++i)
    {
        for (size_t j = 0; j < frames[i].vFeatures.size(); ++j)
        {
            cv::Point2d p2d = frames[i].vFeatures[j].p2d;
            double obs_x = p2d.x;
            double obs_y = p2d.y;

            size_t p_idx = map_id_index[frames[i].vFeatures[j].id_3d];
            if (vPoints[p_idx].to_optimize == false)
            { continue;}

            ceres::CostFunction *cost_function = FeatureReprojectionError2::Create(obs_x, obs_y);
            problem.AddResidualBlock(cost_function,
                                     NULL,
                                     poses + 7 * i,
                                     poses + 7 * i + 4,
                                     camera,
                                     points + 3 * p_idx);
        }
    }

    ceres::Solve(options, &problem, &summary);

    Intrin_opt[0] = camera[0];
    Intrin_opt[1] = camera[0];
    Intrin_opt[2] = camera[1];
    Intrin_opt[3] = camera[2];
    Intrin_opt[4] = camera[3];
    Intrin_opt[5] = camera[4];

    q_inv.w() = ext[0];
    q_inv.x() = ext[1];
    q_inv.y() = ext[2];
    q_inv.z() = ext[3];
    t_inv << ext[4], ext[5], ext[6];
    R_inv = q_inv.toRotationMatrix();

    R_opt = R_inv.inverse();
    t_opt = -R_inv.inverse() * t_inv;

    for (size_t i = 0; i < vPoints.size(); ++i)
    {
        vPoints[i].p3d[0] = points[3 * i + 0];
        vPoints[i].p3d[1] = points[3 * i + 1];
        vPoints[i].p3d[2] = points[3 * i + 2];
    }
    for (int i = 0; i < frame_num; ++i)
    {
        frames[i].C_R.w() = poses[7 * i + 0];
        frames[i].C_R.x() = poses[7 * i + 1];
        frames[i].C_R.y() = poses[7 * i + 2];
        frames[i].C_R.z() = poses[7 * i + 3];
        frames[i].C_t[0] = poses[7 * i + 4];
        frames[i].C_t[1] = poses[7 * i + 5];
        frames[i].C_t[2] = poses[7 * i + 6];
    }

    std::string msg = "Jointly calibrated intrinsic and extrinsic parameters.";
    ROS_INFO_STREAM(msg.c_str());
};

void Calib::print_optimized_calibration()
{
    std::string msg;
    msg = "Optimized calibration parameters:";
    ROS_INFO_STREAM(msg.c_str());

    std::ostringstream R_stream;
    R_stream << "--rotation: [" << std::setprecision(3) <<
                R_opt(0,0) << ", " << R_opt(0,1) << ", " << R_opt(0,2) << "; " <<
                R_opt(1,0) << ", " << R_opt(1,1) << ", " << R_opt(1,2) << "; " << 
                R_opt(2,0) << ", " << R_opt(2,1) << ", " << R_opt(2,2) << "]";
    msg = R_stream.str();
    ROS_INFO_STREAM(msg.c_str());

    std::ostringstream t_stream;
    t_stream << "--translation: [" << std::setprecision(3) <<
                t_opt[0] << ", " << t_opt[1] << ", " << t_opt[2] << "]";
    msg = t_stream.str();
    ROS_INFO_STREAM(msg.c_str());

    std::ostringstream Intrin_stream;
    if (camera_model == "RADIAL")
    {
        Intrin_stream << "--intrinsics: [f, cx, cy, k1, k2] = [" << std::setprecision(4) <<
                         Intrin_opt[0] << ", " <<
                         Intrin_opt[2] << ", " <<
                         Intrin_opt[3] << ", " <<
                         Intrin_opt[4] << ", " <<
                         Intrin_opt[5] << "]";
    }
    msg = Intrin_stream.str();
    ROS_INFO_STREAM(msg.c_str());

    std::cout << std::endl;
};

void Calib::save_opt_calib()
{
    std::string calib_result_path = data_folder + "result/calib_result.txt";
    std::ofstream calib_result_file(calib_result_path);
    calib_result_file << "Intrinsic parameters:" << "\n";
    calib_result_file << "fx:" << Intrin_opt[0] << " fy:" << Intrin_opt[1] << "\n";
    calib_result_file << "cx:" << Intrin_opt[2] << " cy:" << Intrin_opt[3] << "\n";
    calib_result_file << "k1:" << Intrin_opt[4] << " k2:" << Intrin_opt[5] << "\n";
    calib_result_file << "p1:" << Intrin_opt[6] << " p2:" << Intrin_opt[7] << " k3:" << Intrin_opt[8] << "\n\n";
    calib_result_file << "Extrinsic parameters:" << "\n";
    calib_result_file << "--rotation:" << "\n";
    calib_result_file << R_opt << "\n";
    calib_result_file << "--translation:" << "\n";
    calib_result_file << t_opt;
    calib_result_file.close();

    std::string msg = "Calibration result saved to " + calib_result_path;
    ROS_INFO_STREAM(msg.c_str());
};

void Calib::colorize_point_cloud(cv::Mat & img, 
                                 pcl::PointCloud<pcl::PointXYZ> & cloud_raw,
                                 pcl::PointCloud<pcl::PointXYZRGB> & cloud_colorized,
                                 std::vector<double> & intrinsics,
                                 Eigen::Matrix3d & R,
                                 Eigen::Vector3d & t)
{
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << intrinsics[0], 0.0, intrinsics[2],
                                                      0.0, intrinsics[1], intrinsics[3],
                                                      0.0, 0.0, 1.0);
    cv::Mat distCoeff = (cv::Mat_<double>(1, 5) << intrinsics[4], intrinsics[5], intrinsics[6],
                                                   intrinsics[7], intrinsics[8]);
    Eigen::Vector3d euler_angles = R.eulerAngles(2, 1, 0);
    Eigen::AngleAxisd rotation_vector;
    rotation_vector = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());
    cv::Mat r_vec = (cv::Mat_<double>(3, 1) << rotation_vector.angle() * rotation_vector.axis().transpose()[0],
                                               rotation_vector.angle() * rotation_vector.axis().transpose()[1],
                                               rotation_vector.angle() * rotation_vector.axis().transpose()[2]);
    cv::Mat t_vec = (cv::Mat_<double>(3, 1) << t[0], t[1], t[2]);

    std::vector<cv::Point3d> pts_3d;
    std::vector<cv::Point2d> pts_2d;
    for (size_t i = 0; i < cloud_raw.points.size(); ++i)
    {
        pcl::PointXYZ point = cloud_raw.points[i];
        double depth = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
        if (depth > 2.0 && depth < 50.0)
        {
            pts_3d.emplace_back(cv::Point3d(point.x, point.y, point.z));
        }
    }
    cv::projectPoints(pts_3d, r_vec, t_vec, cameraMatrix, distCoeff, pts_2d);

    int image_rows = img.rows;
    int image_cols = img.cols;
    for (size_t i = 0; i < pts_2d.size(); ++i) 
    {
        int u = round(pts_2d[i].x);
        int v = round(pts_2d[i].y);
        if (u >= 1 && u < image_cols-1 && v >= 1 && v < image_rows-1) 
        {
            pcl::PointXYZRGB p;
            p.x = pts_3d[i].x;
            p.y = pts_3d[i].y;
            p.z = pts_3d[i].z;
            p.r = img.at<cv::Vec3b>(v, u)[2];
            p.g = img.at<cv::Vec3b>(v, u)[1];
            p.b = img.at<cv::Vec3b>(v, u)[0];
            cloud_colorized.points.push_back(p);
        }
    }
};

void Calib::colorize()
{
    pcl::PointCloud<pcl::PointXYZRGB> colorization;
    for (int i = 0; i < frame_num; ++i)
    {
        pcl::PointCloud<pcl::PointXYZRGB> colored_cloud;
        colorize_point_cloud(frames[i].image, frames[i].cloud, colored_cloud,
                             Intrin_opt, R_opt, t_opt);
        
        Eigen::Matrix3d R_li_l0(frames[i].L_R);
        Eigen::Vector3d t_li_l0(frames[i].L_t);
        for (size_t j = 0; j < colored_cloud.size(); ++j)
        {
            pcl::PointXYZRGB & p = colored_cloud.points[j];
            Eigen::Vector3d p_vec(p.x, p.y, p.z);
            p_vec = R_li_l0 * p_vec + t_li_l0;
            p.x = p_vec[0];
            p.y = p_vec[1];
            p.z = p_vec[2];

            colorization.points.push_back(p);
        }
    }

    std::string colorization_save_path = data_folder + "result/colorization.pcd";
    pcl::io::savePCDFileBinary(colorization_save_path, colorization);

    std::string msg = "Colorized point cloud saved as " + colorization_save_path;
    ROS_INFO_STREAM(msg.c_str());
}

#endif
