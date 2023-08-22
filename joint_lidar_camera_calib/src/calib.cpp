#include "include/calib.hpp"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "calib");
    ros::NodeHandle nh;
    std::string data_folder_ = "";
    nh.param<std::string>("data_folder", data_folder_, "");

    Calib calib(data_folder_); // Load raw data and configuration parameters

    calib.save_SfM_cloud("init", false, "init"); // Visualize SfM point cloud;
    calib.save_LiDAR_BA_cloud(); // Visualize LiDAR point clouds after BA;

    std::cout << std::endl;
    calib.recover_visual_scale(); // Solve monocular visual scale through hand-eye-calibration
    calib.refine_scale_ext(); // Iteratively refine visual scale and extrinsic parameters
    std::cout << std::endl;

    calib.save_SfM_cloud("before_opt", true, "init"); // Visualize SfM point cloud in the LiDAR frame before optimization;
    std::cout << std::endl;
    calib.joint_calib(); // Jointly calibrate intrinsic and extrinsic parameters
    calib.print_optimized_calibration();
    calib.save_opt_calib();
    calib.save_SfM_cloud("after_opt", true, "opt"); // Visualize SfM point cloud in the LiDAR frame after optimization;
    calib.colorize(); // Colorize point clouds given the calibration

    return 0;
}