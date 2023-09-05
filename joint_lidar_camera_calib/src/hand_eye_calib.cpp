#include "include/calib.hpp"

void hand_eye_calib(std::vector<Eigen::Matrix4d> & lidar_poses,
                    std::vector<Eigen::Matrix4d> & camera_poses,
                    Eigen::Matrix4d & T,
                    double & scale)
{

    if (lidar_poses.size() != camera_poses.size())
    {
        std::string msg = "LiDAR and camera pose size mismatch.";
        ROS_ERROR_STREAM(msg.c_str());
        return;
    }

    // Recover rotation first 
    const int N = 9 * lidar_poses.size();

    cv::Mat A = cv::Mat::zeros(N, 9, CV_32F);
    
    for (size_t i = 0; i < lidar_poses.size(); ++i)
    {
        Eigen::Matrix4d & l_p = lidar_poses[i];
        Eigen::Matrix4d & c_p = camera_poses[i];

        double a1 = l_p(0,0),
               a2 = l_p(0,1),
               a3 = l_p(0,2),
               a4 = l_p(1,0),
               a5 = l_p(1,1),
               a6 = l_p(1,2),
               a7 = l_p(2,0),
               a8 = l_p(2,1),
               a9 = l_p(2,2);

        double b1 = c_p(0,0),
               b2 = c_p(0,1),
               b3 = c_p(0,2),
               b4 = c_p(1,0),
               b5 = c_p(1,1),
               b6 = c_p(1,2),
               b7 = c_p(2,0),
               b8 = c_p(2,1),
               b9 = c_p(2,2);               

        // 0th row
        A.at<float>(i*9 + 0, 0) = a1-b1;
        A.at<float>(i*9 + 0, 1) = -b4;
        A.at<float>(i*9 + 0, 2) = -b7;
        A.at<float>(i*9 + 0, 3) = a2;
        A.at<float>(i*9 + 0, 6) = a3;

        // 1st row
        A.at<float>(i*9 + 1, 0) = -b2;
        A.at<float>(i*9 + 1, 1) = a1-b5;
        A.at<float>(i*9 + 1, 2) = -b8;
        A.at<float>(i*9 + 1, 4) = a2;
        A.at<float>(i*9 + 1, 7) = a3;

        // 2nd row
        A.at<float>(i*9 + 2, 0) = -b3;
        A.at<float>(i*9 + 2, 1) = -b6;
        A.at<float>(i*9 + 2, 2) = a1-b9;
        A.at<float>(i*9 + 2, 5) = a2;
        A.at<float>(i*9 + 2, 8) = a3;   

        // 3rd row
        A.at<float>(i*9 + 3, 0) = a4;
        A.at<float>(i*9 + 3, 3) = a5-b1;
        A.at<float>(i*9 + 3, 4) = -b4;
        A.at<float>(i*9 + 3, 5) = -b7;
        A.at<float>(i*9 + 3, 6) = a6;  

        // 4th row
        A.at<float>(i*9 + 4, 1) = a4;
        A.at<float>(i*9 + 4, 3) = -b2;
        A.at<float>(i*9 + 4, 4) = a5-b5;
        A.at<float>(i*9 + 4, 5) = -b8;
        A.at<float>(i*9 + 4, 7) = a6;  

        // 5th row
        A.at<float>(i*9 + 5, 2) = a4;
        A.at<float>(i*9 + 5, 3) = -b3;
        A.at<float>(i*9 + 5, 4) = -b6;
        A.at<float>(i*9 + 5, 5) = a5-b9;
        A.at<float>(i*9 + 5, 8) = a6;   

        // 6th row
        A.at<float>(i*9 + 6, 0) = a7;
        A.at<float>(i*9 + 6, 3) = a8;
        A.at<float>(i*9 + 6, 6) = a9-b1;
        A.at<float>(i*9 + 6, 7) = -b4;
        A.at<float>(i*9 + 6, 8) = -b7;   

        // 7th row
        A.at<float>(i*9 + 7, 1) = a7;
        A.at<float>(i*9 + 7, 4) = a8;
        A.at<float>(i*9 + 7, 6) = -b2;
        A.at<float>(i*9 + 7, 7) = a9-b5;
        A.at<float>(i*9 + 7, 8) = -b8;    

        // 8th row
        A.at<float>(i*9 + 8, 2) = a7;
        A.at<float>(i*9 + 8, 5) = a8;
        A.at<float>(i*9 + 8, 6) = -b3;
        A.at<float>(i*9 + 8, 7) = -b6;
        A.at<float>(i*9 + 8, 8) = a9-b9;                              
    }

    cv::Mat u, w, vt, R;
    cv::SVDecomp(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    cv::Mat Rpre = vt.row(8).reshape(0, 3);

    cv::SVDecomp(Rpre, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
    R = u * vt;

    // Recover t
    Eigen::Matrix3d rot;
    rot << R.at<float>(0,0), R.at<float>(0,1), R.at<float>(0,2),
           R.at<float>(1,0), R.at<float>(1,1), R.at<float>(1,2),
           R.at<float>(2,0), R.at<float>(2,1), R.at<float>(2,2); 

    Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3,3);

    const int N2 = 3 * lidar_poses.size();
    Eigen::MatrixXd B(N2, 4);
    Eigen::MatrixXd C(N2, 1);

    for (size_t i = 0; i < lidar_poses.size(); ++i)
    {
        Eigen::Matrix4d & l_p = lidar_poses[i];
        Eigen::Matrix4d & c_p = camera_poses[i];        
        B.block(i*3, 0, 3, 3) = I - l_p.block(0, 0, 3, 3);
        B.block(i*3, 3, 3, 1) = rot * c_p.block(0, 3, 3, 1);
        C.block(i*3, 0, 3, 1) = l_p.block(0, 3, 3, 1);
    }

    Eigen::MatrixXd D = B.transpose() * B;
    Eigen::MatrixXd V = D.inverse() * B.transpose() * C;

    T.block(0, 0, 3, 3) = rot;
    T.block(0, 3, 3, 1) = V.block(0, 0, 3, 1);
    T(3, 3) = 1.0;

    scale = V(3, 0);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "hand_eye_calib");
    ros::NodeHandle nh;
    std::string lidar_pose_path = "",
                camera_pose_path = "";
    nh.param<std::string>("lidar_pose_path", lidar_pose_path, "");
    nh.param<std::string>("camera_pose_path", camera_pose_path, "");

    std::ifstream lidar_pose_file(lidar_pose_path);
    std::ifstream camera_pose_file(camera_pose_path);

    std::vector<Eigen::Matrix4d> lidar_poses, camera_poses;

    while (!lidar_pose_file.eof())
    {   
        Eigen::Matrix4d lidar_T = Eigen::Matrix4d::Zero();
        Eigen::Matrix3d lidar_R = Eigen::Matrix3d::Zero();
        Eigen::Quaterniond lidar_q;

        double qw, qx, qy, qz, tx, ty, tz; 
        lidar_pose_file >> qw >> qx >> qy >> qz >> tx >> ty >> tz;

        lidar_q.w() = qw;
        lidar_q.x() = qx;
        lidar_q.y() = qy;
        lidar_q.z() = qz;

        lidar_R = lidar_q.toRotationMatrix();
        lidar_T.block(0, 0, 3, 3) = lidar_R;
        lidar_T(0, 3) = tx;
        lidar_T(1, 3) = ty;
        lidar_T(2, 3) = tz;
        lidar_T(3, 3) = 1.0;

        lidar_poses.push_back(lidar_T);
    }
    lidar_pose_file.close();

    while (!camera_pose_file.eof())
    {
        Eigen::Matrix4d camera_T = Eigen::Matrix4d::Zero();
        Eigen::Matrix3d camera_R = Eigen::Matrix3d::Zero();
        Eigen::Quaterniond camera_q;

        double qw, qx, qy, qz, tx, ty, tz; 
        camera_pose_file >> qw >> qx >> qy >> qz >> tx >> ty >> tz;

        camera_q.w() = qw;
        camera_q.x() = qx;
        camera_q.y() = qy;
        camera_q.z() = qz;

        camera_R = camera_q.toRotationMatrix();
        camera_T.block(0, 0, 3, 3) = camera_R;
        camera_T(0, 3) = tx;
        camera_T(1, 3) = ty;
        camera_T(2, 3) = tz;
        camera_T(3, 3) = 1.0;

        camera_poses.push_back(camera_T);
    }
    camera_pose_file.close();

    double scale;
    Eigen::Matrix4d T_c_l;

    hand_eye_calib(lidar_poses, camera_poses, T_c_l, scale);

    std::cout << "Extrinsic parameters: (project a point from camera frame to LiDAR frame)" << std::endl;
    std::cout << T_c_l << std::endl;
    std::cout << "(project a point from LiDAR frame to camera frame)" << std::endl;
    std::cout << T_c_l.inverse() << std::endl;
    std::cout << "Visual scale: " << scale << std::endl;    
}