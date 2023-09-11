#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <fstream>

class LidarPlaneFactor
{
public:
    LidarPlaneFactor(Eigen::Vector3d point_, 
                     Eigen::Vector3d center_, 
                     Eigen::Vector3d normal_) 
    { 
        point = point_;
        center = center_;
        normal = normal_; 
    }

    template <typename T>
    bool operator()(const T *_q, const T *_t, T *residuals) const 
    {
        Eigen::Quaternion<T> q_incre{_q[3], _q[0], _q[1], _q[2]};
        Eigen::Matrix<T, 3, 1> t_incre{_t[0], _t[1], _t[2]};

        Eigen::Matrix<T, 3, 1> norm{T(normal[0]), T(normal[1]), T(normal[2])};
        Eigen::Matrix<T, 3, 1> center_point{T(center[0]), T(center[1]), T(center[2])};
        Eigen::Matrix<T, 3, 1> p_local{T(point[0]), T(point[1]), T(point[2])};
        Eigen::Matrix<T, 3, 1> p_world = q_incre.toRotationMatrix() * p_local + t_incre;
        Eigen::Matrix<T, 3, 1> diff = p_world - center_point;

        residuals[0] = norm.dot(diff);

        return true;
    }
    static ceres::CostFunction *Create(Eigen::Vector3d point_, 
                                       Eigen::Vector3d center_, 
                                       Eigen::Vector3d normal_) 
    {
        return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 1, 4, 3>
                (new LidarPlaneFactor(point_, center_, normal_)));
    }

private:
    Eigen::Vector3d point;
    Eigen::Vector3d center; 
    Eigen::Vector3d normal;     
};


int calc_plane(std::vector<Eigen::Vector3d> & points,
               Eigen::Vector3d & center,
               Eigen::Vector3d & normal,
               double & d,
               float & ratio_thre)
{

    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    center = Eigen::Vector3d::Zero();
    normal = Eigen::Vector3d::Zero();
    int points_size = points.size();

    for(auto pv : points)
    {
        covariance += pv * pv.transpose();
        center += pv;
    }
    center = center / points_size;
    covariance = covariance / points_size - center * center.transpose();
    Eigen::EigenSolver<Eigen::Matrix3d> es(covariance);
    Eigen::Matrix3cd evecs = es.eigenvectors();
    Eigen::Vector3cd evals = es.eigenvalues();
    Eigen::Vector3d evalsReal;
    evalsReal = evals.real();
    Eigen::Matrix3f::Index evalsMin, evalsMax;
    evalsReal.rowwise().sum().minCoeff(&evalsMin);
    evalsReal.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);

    // std::cout << "Result: " << evalsReal(evalsMin) << ", " << evalsReal(evalsMid) << ", " << evalsReal(evalsMax) << std::endl;
    if ((evalsReal(evalsMid) / evalsReal(evalsMin)) > ratio_thre)
    {
        // std::cout << "Is a plane. Ratio: " << (evalsReal(evalsMid) / evalsReal(evalsMin)) << std::endl; // for debug use only
        normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
                  evecs.real()(2, evalsMin);
        d = -(normal(0) * center(0) + normal(1) * center(1) + normal(2) * center(2));

        return 1;
    }
    else
        return 0;
}

void downSamplePointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud_in,
                          pcl::PointCloud<pcl::PointXYZ> & cloud_out,
                          float & leaf_size)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in_pointer = cloud_in.makeShared();
    pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
    downSizeFilter.setInputCloud(cloud_in_pointer);
    downSizeFilter.setLeafSize(leaf_size, leaf_size, leaf_size);
    downSizeFilter.filter(cloud_out);
}

void removeClosePoints(pcl::PointCloud<pcl::PointXYZ> & cloud, float & dist_thre)
{
    pcl::PointCloud<pcl::PointXYZ> temp_cloud = cloud;
    cloud.points.clear();

    for (size_t i = 0; i < temp_cloud.points.size(); ++i)
    {
        pcl::PointXYZ p = temp_cloud.points[i];
        float sqrt_dist = p.x * p.x + p.y * p.y + p.z * p.z;

        if (sqrt_dist >= (dist_thre * dist_thre))
        {
            cloud.points.push_back(p);
        }
    }

    cloud.height = 1;
    cloud.width = cloud.points.size();
}

void registerPointCloud(pcl::PointCloud<pcl::PointXYZ> & cloud1,
                        pcl::PointCloud<pcl::PointXYZ> & cloud2,
                        Eigen::Matrix3d & R, 
                        Eigen::Vector3d & t,
                        int & nn_number, 
                        int & incre_number,
                        float squareDistThre)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_query = cloud1.makeShared();
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>());
    kdtree->setInputCloud(cloud_query);    
    std::vector<int> pointIdxNKNSearch(nn_number);
    std::vector<float> pointNKNSquaredDistance(nn_number);

    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem::Options problem_options;

    ceres::Problem problem(problem_options);

    Eigen::Quaterniond q(R);
    double para_q[4] = {q.x(), q.y(), q.z(), q.w()};
    double para_t[3] = {t[0], t[1], t[2]}; 

    Eigen::Map<Eigen::Quaterniond> m_q = Eigen::Map<Eigen::Quaterniond>(para_q);
    Eigen::Map<Eigen::Vector3d> m_t = Eigen::Map<Eigen::Vector3d>(para_t);     

    problem.AddParameterBlock(para_q, 4, q_parameterization);
    problem.AddParameterBlock(para_t, 3);

    pcl::PointCloud<pcl::PointXYZ> co_cloud;

    for (int iter = 0; iter < 3; ++iter)
    {
        int search_num = 0;
        int corrs_num = 0,
            far_num = 0,
            notplane_num = 0;
        for (size_t i = 0; i < cloud2.points.size(); i += incre_number)
        {
            ++search_num;

            pcl::PointXYZ searchPoint = cloud2.points[i];
            Eigen::Vector3d searchPoint_eigen(searchPoint.x, searchPoint.y, searchPoint.z);
            searchPoint_eigen = R * searchPoint_eigen + t;
            pcl::PointXYZ searchPointWorld;
            searchPointWorld.x = searchPoint_eigen[0];
            searchPointWorld.y = searchPoint_eigen[1];
            searchPointWorld.z = searchPoint_eigen[2]; 

            if (kdtree->nearestKSearch(searchPointWorld, nn_number, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
            {
                // If nearest neighbours are too far
                if (pointNKNSquaredDistance[nn_number - 1] > squareDistThre)
                {
                    ++far_num;
                    continue;
                }

                // Whether nearest neighbours form a plane?
                float ratio_thre = 200.0; 
                std::vector<Eigen::Vector3d> points;
                for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
                {
                    int idx = pointIdxNKNSearch[j];
                    pcl::PointXYZ & p = cloud_query->points[idx]; 
                    Eigen::Vector3d point(p.x, p.y, p.z);
                    points.push_back(point);
                }

                Eigen::Vector3d center, normal;
                double d;
                if (calc_plane(points, center, normal, d, ratio_thre) == 1)
                {
                    ++corrs_num;
                    co_cloud.points.push_back(searchPoint);

                    // Add residual block
                    Eigen::Vector3d point(searchPoint.x, searchPoint.y, searchPoint.z);
                    ceres::CostFunction *cost_function;
                    cost_function = LidarPlaneFactor::Create(point, center, normal);
                    problem.AddResidualBlock(cost_function, loss_function, para_q, para_t);
                }
                else
                {
                    ++notplane_num;
                }
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.max_num_iterations = 4;
        options.minimizer_progress_to_stdout = false;
        options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);

        R = m_q.toRotationMatrix();
        t[0] = m_t(0);
        t[1] = m_t(1);
        t[2] = m_t(2);

        q = R;
        para_q[0] = q.x();
        para_q[1] = q.y();
        para_q[2] = q.z();
        para_q[3] = q.w();
        para_t[0] = t[0];
        para_t[1] = t[1];
        para_t[2] = t[2]; 

        squareDistThre /= 4.0;
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "registration");
    ros::NodeHandle nh;

    std::string file_folder = "";
    int data_num = 0;
    float dist_thre = 0.0;
    float leaf_size = 0.0;

    int nn_number = 1;
    int incre_number = 1;
    float squareDistThre = 1.0;

    nh.param<std::string>("file_folder", file_folder, "");
    nh.param<int>("data_num", data_num, 0);
    nh.param<float>("dist_thre", dist_thre, 2.5);
    nh.param<float>("leaf_size", leaf_size, 0.2);
    nh.param<int>("nn_number", nn_number, 20);
    nh.param<int>("incre_number", incre_number, 5);
    nh.param<float>("squareDistThre", squareDistThre, 16.0);

    std::cout << "Incremental point-to-plane registration. " << std::to_string(data_num) << " scans to process." << std::endl;

    // load PCL point cloud
    std::vector<pcl::PointCloud<pcl::PointXYZ>> raw_pointclouds;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> downsampled_pointclouds;
    for (int i = 0; i < data_num; ++i)
    {
        std::string file_path = file_folder + "clouds/" + std::to_string(i) + ".pcd";
        pcl::PointCloud<pcl::PointXYZ> raw_pointcloud;
        pcl::io::loadPCDFile(file_path, raw_pointcloud);
        removeClosePoints(raw_pointcloud, dist_thre);
        raw_pointclouds.push_back(raw_pointcloud);
    
        pcl::PointCloud<pcl::PointXYZ> downsampled_pointcloud;
        downSamplePointCloud(raw_pointclouds[i], downsampled_pointcloud, leaf_size);
        downsampled_pointclouds.push_back(downsampled_pointcloud);
    }

    std::vector<std::pair<Eigen::Matrix3d, Eigen::Vector3d>> poses;
    Eigen::Matrix3d R;
    R << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    Eigen::Vector3d t;
    t << 0.0, 0.0, 0.0;
    poses.push_back(std::pair<Eigen::Matrix3d, Eigen::Vector3d>(R, t));

    pcl::PointCloud<pcl::PointXYZ> downsampled_map;
    pcl::PointCloud<pcl::PointXYZ> world_map;
    downsampled_map = downsampled_pointclouds[0];
    world_map = raw_pointclouds[0];
    for (int i = 1; i < data_num; ++i)
    {
        registerPointCloud(downsampled_map, downsampled_pointclouds[i], 
                           R, t, nn_number, incre_number, squareDistThre);
        poses.push_back(std::pair<Eigen::Matrix3d, Eigen::Vector3d>(R, t));
        Eigen::Matrix4f T;
        T << float(R(0,0)), float(R(0,1)), float(R(0,2)), float(t[0]),
             float(R(1,0)), float(R(1,1)), float(R(1,2)), float(t[1]),
             float(R(2,0)), float(R(2,1)), float(R(2,2)), float(t[2]),
             0.0,     0.0,     0.0,     1.0;
        pcl::PointCloud<pcl::PointXYZ> currentCloudInWorld;
        pcl::transformPointCloud(raw_pointclouds[i], currentCloudInWorld, T);
        world_map += currentCloudInWorld;
        downSamplePointCloud(world_map, downsampled_map, leaf_size);
    }
    std::cout << "Registration completed." << std::endl;

    std::string pose_path = file_folder + "LiDAR_pose/lidar_poses_regis.txt";
    std::cout << "Writing poses to " << pose_path << std::endl;

    std::ofstream pose_file;
    pose_file.open(pose_path);
    for (size_t i = 0; i < poses.size(); ++i)
    {
        R = poses[i].first;
        t = poses[i].second;
        Eigen::Quaterniond q(R);

        if (i < (poses.size() - 1))
        {
            pose_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
                      << t[0] << " " << t[1] << " " << t[2] << "\n";
        }
        else
        {
            pose_file << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
                      << t[0] << " " << t[1] << " " << t[2];
        }
    }

    pose_file.close();

    return 0;
}