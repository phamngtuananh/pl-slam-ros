#include <iostream>
#include <string>

#include <ros/package.h>

#include "node.h"

int main(int argc, char **argv)
{
    // Usage: ./slam  <params_file>  <config_file>
    // <params_file> - file with the camera configuration
    // <config_file> - file with the VO/SLAM configuration (optional)
    ros::init(argc, argv, "plslam");

    std::string params_file, config_file;
    std::string package_path = ros::package::getPath("pl_slam_ros");
    if (argc == 1 && !package_path.empty())
    {
        params_file = package_path + "/dependencies/pl-slam/config/dataset_params/zed_params_raw.yaml";
        config_file = package_path + "/dependencies/pl-slam/config/config/config_euroc.yaml";
    }
    else
    {
        std::cout << "Cannot find path to package" << std::endl;
        params_file = argv[1];
        if (argc == 3)
            config_file = argv[2];
    }

    std::cout << "Using config: " << params_file << std::endl;
    PLSLAM::Node plslam(params_file, config_file);
    ros::spin();
    return 0;
}