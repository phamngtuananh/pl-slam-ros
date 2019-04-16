#pragma once

#include <iostream>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <mapHandler.h>

using namespace std;

namespace PLSLAM
{
class Publisher
{
  public:
    Publisher(const ros::NodeHandle &nh, const std::string &frame_id = "camera", const std::string &map_frame_id = "odom")
        : nh_(nh), traj_msg_ptr_(new nav_msgs::Path), frame_id_(frame_id), map_frame_id_(map_frame_id),
          last_optical_pose_(Eigen::Matrix4d::Identity()), is_updated_(false)
    {
        traj_pub_ = nh_.advertise<nav_msgs::Path>("trajectory", 1, true);
        pose_pub_ = nh_.advertise<nav_msgs::Odometry>("slam/odom", 1, true);
        traj_msg_ptr_->header.frame_id = map_frame_id_;
        offset_ << 0, -1, 0, 0, 0, 0, -1, 0, 1, 0, 0, 0, 0, 0, 0, 1;
        offset_inv_ = offset_.inverse();
    }

    ~Publisher() {}

    void setMapHandler(MapHandler *map)
    {
        map_ = map;
    }

    void run()
    {
        while (ros::ok())
        {
            if (is_updated_)
            {
                std::lock_guard<std::mutex> lck(traj_mtx_);
                traj_pub_.publish(traj_msg_ptr_);
                publishCameraPose();
                is_updated_ = false;
            }
        }
    }

    void updateKeyFrames(const ros::Time &t)
    {
        std::lock_guard<std::mutex> lck(traj_mtx_);
        traj_msg_ptr_->poses.clear();
        for (std::vector<KeyFrame *>::const_iterator it = map_->map_keyframes.begin();
             it < map_->map_keyframes.end(); ++it)
        {
            if (*it == NULL)
            {
                continue;
            }

            Eigen::Matrix4d T = opticalOffset((*it)->T_kf_w);

            Eigen::Quaterniond q(T.block<3, 3>(0, 0));
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = T(0, 3);
            pose.pose.position.y = T(1, 3);
            pose.pose.position.z = T(2, 3);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            traj_msg_ptr_->poses.push_back(pose);
        }

        traj_msg_ptr_->header.stamp = t;
        last_optical_pose_ = map_->map_keyframes.back()->T_kf_w;
        last_pose_ = opticalOffset(last_optical_pose_);
        is_updated_ = true;
    }

    void updateFrame(const ros::Time &t, const Eigen::Matrix4d &dT)
    {
        std::lock_guard<std::mutex> lck(traj_mtx_);
        if (traj_msg_ptr_->poses.empty())
        {
            ROS_WARN("Call to updateFrame() without initialization");
            return;
        }

        last_optical_pose_ = last_optical_pose_ * dT;
        last_pose_ = opticalOffset(last_optical_pose_);

        Eigen::Quaterniond q(last_pose_.block<3, 3>(0, 0));
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = last_pose_(0, 3);
        pose.pose.position.y = last_pose_(1, 3);
        pose.pose.position.z = last_pose_(2, 3);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        traj_msg_ptr_->poses.push_back(pose);
        traj_msg_ptr_->header.stamp = t;
        is_updated_ = true;
    }

  private:
    void publishCameraPose()
    {
        nav_msgs::Odometry::Ptr out(new nav_msgs::Odometry);
        out->header.frame_id = map_frame_id_;
        out->header.stamp = traj_msg_ptr_->header.stamp;
        out->child_frame_id = frame_id_;

        Eigen::Quaterniond q(last_pose_.block<3, 3>(0, 0));
        out->pose.pose.position.x = last_pose_(0, 3);
        out->pose.pose.position.y = last_pose_(1, 3);
        out->pose.pose.position.z = last_pose_(2, 3);
        out->pose.pose.orientation.x = q.x();
        out->pose.pose.orientation.y = q.y();
        out->pose.pose.orientation.z = q.z();
        out->pose.pose.orientation.w = q.w();
        out->pose.covariance[0] = 0.01;
        out->pose.covariance[7] = 0.01;
        out->pose.covariance[14] = 0.01;
        out->pose.covariance[21] = 0.1;
        out->pose.covariance[28] = 0.1;
        out->pose.covariance[35] = 0.1;
        pose_pub_.publish(out);

        br_.sendTransform(
            tf::StampedTransform(
                tf::Transform(
                    tf::Quaternion(q.x(), q.y(), q.z(), q.w()),
                    tf::Vector3(last_pose_(0, 3), last_pose_(1, 3), last_pose_(2, 3)))
                    .inverse(),
                traj_msg_ptr_->header.stamp,
                frame_id_,
                map_frame_id_));
    }

    inline Eigen::Matrix4d opticalOffset(const Eigen::Matrix4d &in)
    {
        return offset_inv_ * (in * offset_);
    }

    ros::NodeHandle nh_;
    tf::TransformBroadcaster br_;
    ros::Publisher traj_pub_, pose_pub_;
    std::string frame_id_, map_frame_id_;

    MapHandler *map_;
    Eigen::Matrix4d last_pose_, last_optical_pose_;
    nav_msgs::Path::Ptr traj_msg_ptr_;
    std::atomic<bool> is_updated_;
    std::mutex traj_mtx_;

    Eigen::Matrix4d offset_, offset_inv_;
};
} // namespace PLSLAM