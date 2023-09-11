#include "tools.hpp"
#include <ros/ros.h>
#include <Eigen/Eigenvalues>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseArray.h>
#include <fstream>
#include <random>
#include <ctime>
#include <tf/transform_broadcaster.h>
#include "bavoxel.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <malloc.h>

void read_file(std::vector<IMUST> &x_buf, std::vector<pcl::PointCloud<PointType>::Ptr> &pl_fulls, 
               std::string &file_path, const int &data_num, double & noise_level)
{  
    std::string pose_file_path = file_path + "LiDAR_pose/lidar_poses_regis.txt";
    std::ifstream pose_file;
    pose_file.open(pose_file_path);

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(0.0, 1.0);    

    for(int m = 0; m < data_num; m++)
    {
        std::string filename = file_path + "clouds/" + std::to_string(m) + ".pcd";

        pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
        pcl::PointCloud<pcl::PointXYZ> pl_tem;
        pcl::io::loadPCDFile(filename, pl_tem);
        for (pcl::PointXYZ &pp : pl_tem.points)
        {
            PointType ap;
            double noise = distribution(generator) * noise_level;
            ap.x = pp.x + noise;
            noise = distribution(generator) * noise_level;
            ap.y = pp.y + noise; 
            noise = distribution(generator) * noise_level;
            ap.z = pp.z + noise;
            pl_ptr->push_back(ap);
        }

        pl_fulls.push_back(pl_ptr);

        double qx, qy, qz, qw, tx, ty, tz;
        pose_file >> qx >> qy >> qz >> qw >> tx >> ty >> tz;
        Eigen::Quaterniond q;
        q.x() = qx;
        q.y() = qy;
        q.z() = qz;
        q.w() = qw;
        q.normalize();
        Eigen::Vector3d t(tx, ty, tz);

        IMUST curr;
        curr.R = q.toRotationMatrix(); 
        curr.p = t; 
        x_buf.push_back(curr);
    }

    pose_file.close();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "conduct_BA");
    ros::NodeHandle nh;

    std::vector<IMUST> x_buf;
    std::vector<pcl::PointCloud<PointType>::Ptr> pl_fulls;

    int data_num = 1;
    int iterNum = 1;
    int save_final_cloud = 0;
    nh.param<int>("data_num", data_num, 1);
    nh.param<int>("iterNum", iterNum, 1);
    nh.param<int>("save_final_cloud", save_final_cloud, 0);
    nh.param<double>("voxel_size", voxel_size, 1);
    std::string file_path;
    nh.param<string>("file_path", file_path, "");
    double noise_level = 0.0;
    nh.param<double>("noise_level", noise_level, 0.0);

    read_file(x_buf, pl_fulls, file_path, data_num, noise_level);

    IMUST es0 = x_buf[0];
    for(uint i=0; i<x_buf.size(); i++)
    {
        x_buf[i].p = es0.R.transpose() * (x_buf[i].p - es0.p);
        x_buf[i].R = es0.R.transpose() * x_buf[i].R;
    }

    win_size = x_buf.size();

    for(int iterCount = 0; iterCount < iterNum; iterCount++)
    { 
        std::unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> surf_map;

        for (int i = 0; i < win_size; i++)
        {
            cut_voxel(surf_map, *pl_fulls[i], x_buf[i], i);
        }

        VOX_HESS voxhess;
        for(auto iter = surf_map.begin(); iter != surf_map.end(); iter++)
        {
            iter->second->recut(win_size);
            iter->second->tras_opt(voxhess, win_size);
        }
        
        BALM2 opt_lsv;
        opt_lsv.damping_iter(x_buf, voxhess);

        for(auto iter = surf_map.begin(); iter != surf_map.end();)
        {
            delete iter->second;
            surf_map.erase(iter++);
        }
        surf_map.clear();

        malloc_trim(0);
    }       

    malloc_trim(0);

    // Save BA result in .txt and pcd format(optional)
    std::string pose_file_path = file_path + "LiDAR_pose/lidar_poses_BA.txt";
    std::ofstream pose_file;
    pose_file.open(pose_file_path);
    for (int i = 0; i < data_num; ++i)
    {
        Eigen::Quaterniond q(x_buf[i].R);
        
        if (i == (data_num - 1))
        {
            pose_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
                      << x_buf[i].p[0] << " " << x_buf[i].p[1] << " " << x_buf[i].p[2];     
        }
        else
        {
            pose_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
                      << x_buf[i].p[0] << " " << x_buf[i].p[1] << " " << x_buf[i].p[2] << "\n";
        }
    }

    if (save_final_cloud == 1)
    {
        pcl::PointCloud<pcl::PointXYZI> cloud_all;

        for (int i = 0; i < data_num; ++i)
        {
            Eigen::Matrix3d R = x_buf[i].R;
            Eigen::Vector3d t = x_buf[i].p;

            for (size_t j = 0; j < pl_fulls[i]->points.size(); ++j)
            {
                Eigen::Vector3d p_vec(pl_fulls[i]->points[j].x, pl_fulls[i]->points[j].y, pl_fulls[i]->points[j].z);
                p_vec = R * p_vec + t;
                pcl::PointXYZI p;
                p.x = p_vec[0];
                p.y = p_vec[1];
                p.z = p_vec[2];
                p.intensity = i;
                cloud_all.points.push_back(p);
            }
        }

        std::string save_cloud_path = file_path + "clouds/cloud_all.pcd";
        pcl::io::savePCDFileBinary(save_cloud_path, cloud_all);
    }

    return 0;
}