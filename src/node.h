#pragma once

#include <iostream>
#include <chrono>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include <mapHandler.h>
#include <stereoFrame.h>
#include <stereoFrameHandler.h>

#include "publisher.h"

using namespace std::chrono;

namespace PLSLAM
{
class Node
{
  private:
    ros::NodeHandle nh_;

    // The synchronization policy used by the interface to sync stereo images
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;

    // Subscribers
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> img_sub_l_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> img_sub_r_;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync_;
    image_transport::ImageTransport it_;
    ros::Subscriber imu_sub;

    PinholeStereoCamera *cam_pin;
    std::string img_encoding;
    bool rectify;

    StereoFrameHandler *StVO;
    PLSLAM::MapHandler *map;
    PLSLAM::Publisher *pb;
    int frame_counter;

    std::thread *pbt;

  public:
    Node(const std::string &params_file, const std::string &config_file)
        : it_(nh_), rectify(false)
    {

        std::cout << "Initializing PL-SLAM ... " << std::endl;

        YAML::Node dset_config = YAML::LoadFile(params_file);
        YAML::Node cam_config = dset_config["cam0"];
        string camera_model = cam_config["cam_model"].as<string>();
        if (camera_model == "Pinhole" && cam_config["Kl"].IsDefined())
        {
            std::cout << "Reading camera configuration ... " << std::flush;
            rectify = !cam_config["is_rectified"].as<bool>();
            img_encoding = cam_config["encoding"].as<string>();
            Mat Kl, Kr, Dl, Dr, R, t;
            vector<double> Kl_ = cam_config["Kl"].as<vector<double>>();
            vector<double> Kr_ = cam_config["Kr"].as<vector<double>>();
            vector<double> Dl_ = cam_config["Dl"].as<vector<double>>();
            vector<double> Dr_ = cam_config["Dr"].as<vector<double>>();
            Kl = (Mat_<float>(3, 3) << Kl_[0], 0.0, Kl_[2], 0.0, Kl_[1], Kl_[3], 0.0, 0.0, 1.0);
            Kr = (Mat_<float>(3, 3) << Kr_[0], 0.0, Kr_[2], 0.0, Kr_[1], Kr_[3], 0.0, 0.0, 1.0);
            vector<double> R_ = cam_config["R"].as<vector<double>>();
            vector<double> t_ = cam_config["t"].as<vector<double>>();
            R = Mat::eye(3, 3, CV_64F);
            t = Mat::eye(3, 1, CV_64F);
            int k = 0;
            for (int i = 0; i < 3; i++)
            {
                t.at<double>(i, 0) = t_[i];
                for (int j = 0; j < 3; j++, k++)
                    R.at<double>(i, j) = R_[k];
            }
            // load distortion parameters
            int Nd = Dl_.size();
            Dl = Mat::eye(1, Nd, CV_64F);
            Dr = Mat::eye(1, Nd, CV_64F);
            for (int i = 0; i < Nd; i++)
            {
                Dl.at<double>(0, i) = Dl_[i];
                Dr.at<double>(0, i) = Dr_[i];
            }

            // create camera object for EuRoC
            cam_pin = new PinholeStereoCamera(
                cam_config["cam_width"].as<double>(),
                cam_config["cam_height"].as<double>(),
                cam_config["cam_bl"].as<double>(),
                Kl, Kr, R, t, Dl, Dr, false);
            std::cout << "Done" << endl;
        }
        else
        {
            std::cout << "Unrecognized configuration" << std::endl;
            exit(EXIT_FAILURE);
        }

        std::cout << "Initialize VO / SLAM ... " << std::flush;
        // Initialize VO object
        //------------------------------------------------------
        frame_counter = 0;
        StVO = new StereoFrameHandler(cam_pin);
        if (!config_file.empty())
            Config::loadFromFile(config_file);

        // Initialize SLAM object
        //------------------------------------------------------
        map = new PLSLAM::MapHandler(cam_pin);
        if (!config_file.empty())
            SlamConfig::loadFromFile(config_file);
        std::cout << "Done" << std::endl;

        // Initializa ROS image subscriber
        img_sub_l_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
            new message_filters::Subscriber<sensor_msgs::Image>(nh_, cam_config["left_topic"].as<string>(), 1));
        img_sub_r_ = std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>(
            new message_filters::Subscriber<sensor_msgs::Image>(nh_, cam_config["right_topic"].as<string>(), 1));
        sync_ = std::shared_ptr<message_filters::Synchronizer<sync_pol>>(
            new message_filters::Synchronizer<sync_pol>(sync_pol(10), *img_sub_l_, *img_sub_r_));
        sync_->registerCallback(boost::bind(&Node::imageCallback, this, _1, _2));


        // imu_sub = nh_.subscribe("/imu/data", 1000, &Node::imuCallback, this);

        // Initialize output publisher
        pb = new PLSLAM::Publisher(nh_);
        pb->setMapHandler(map);
        pbt = new std::thread(&PLSLAM::Publisher::run, pb);

        std::cout << "Initialization finished" << std::endl;
    }

    ~Node()
    {
        delete pbt;
        delete pb;
        delete map;
        delete StVO;
        delete cam_pin;
    }

    void imuCallback(const sensor_msgs::ImuConstPtr &imu)
    {
        // Wait until camera is tracked and the coordinates are init first
        if (!frame_counter)
            return;

        // Convert quaternion to matrix pose format
        ros::Time imu_time(imu->header.stamp);
        static Eigen::Quaterniond imu_quat_init(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        Eigen::Quaterniond imu_quat(imu->orientation.w, imu->orientation.x, imu->orientation.y, imu->orientation.z);
        imu_quat = imu_quat_init.inverse() * imu_quat;

        sensor_msgs::Imu::Ptr imu_out(new sensor_msgs::Imu(*imu));
        static int sim_imu_seq = 1;
        imu_out->header.seq = sim_imu_seq++;
        imu_out->orientation.x = imu_quat.x();
        imu_out->orientation.y = imu_quat.y();
        imu_out->orientation.z = imu_quat.z();
        imu_out->orientation.w = imu_quat.w();
        publishImu(imu_out, imu->header.stamp);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg_l, const sensor_msgs::ImageConstPtr &msg_r)
    {
        // start time benchmark
        const auto time_start = high_resolution_clock::now();
        // grab images
        cv_bridge::CvImageConstPtr cv_ptr_l, cv_ptr_r;
        try
        {
            cv_ptr_l = cv_bridge::toCvShare(msg_l, img_encoding);
            cv_ptr_r = cv_bridge::toCvShare(msg_r, img_encoding);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // rectify images (if distorted)
        cv::Mat img_l_rec, img_r_rec;
        if (rectify)
        {
            cam_pin->rectifyImagesLR(cv_ptr_l->image, img_l_rec, cv_ptr_r->image, img_r_rec);
        }
        else
        {
            img_l_rec = cv_ptr_l->image;
            img_r_rec = cv_ptr_r->image;
        }

        // initialize VO if frame #0
        if (frame_counter == 0)
        {
            StVO->initialize(img_l_rec, img_r_rec, frame_counter);
            PLSLAM::KeyFrame *kf = new PLSLAM::KeyFrame(StVO->prev_frame, 0);
            map->initialize(kf);
            frame_counter++;
            pb->updateKeyFrames(ros::Time(msg_l->header.stamp));
        }
        // run VO otherwise
        else
        {
            StVO->insertStereoPair(img_l_rec, img_r_rec, frame_counter);
            StVO->optimizePose();

            if (StVO->needNewKF())
            {
                PLSLAM::KeyFrame *curr_kf = new PLSLAM::KeyFrame(StVO->curr_frame);
                StVO->currFrameIsKF();
                map->addKeyFrame(curr_kf);
                pb->updateKeyFrames(ros::Time(msg_l->header.stamp));
            }
            else
            {
                pb->updateFrame(ros::Time(msg_l->header.stamp), StVO->curr_frame->DT);
            }

            StVO->updateFrame();
            frame_counter++;
        }

        std::cout << "Loop finished. Took " 
                  << duration_cast<chrono::milliseconds>(high_resolution_clock::now() - time_start).count() 
                  << "ms" << std::endl;
    }

    void publishImu(const sensor_msgs::Imu::ConstPtr &imu, const ros::Time &stamp)
    {
        // imu_pub.publish(imu);

        // br->sendTransform(
        //     tf::StampedTransform(
        //         tf::Transform(tf::Quaternion(imu->orientation.x,
        //                                      imu->orientation.y,
        //                                      imu->orientation.z,
        //                                      imu->orientation.w),
        //                       tf::Vector3(0.0, 0.0, 0.0))
        //             .inverse(),
        //         stamp,
        //         "imu",
        //         "odom"));
    }
}; // class Node
} // namespace PLSLAM