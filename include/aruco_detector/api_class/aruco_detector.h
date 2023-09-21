#ifndef __MORIN_ARUCO_DETECTOR_H__
#define __MORIN_ARUCO_DETECTOR_H__

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
#include <unordered_map>
#include <visualization_msgs/MarkerArray.h>
using namespace std;

class ArucoDetector{
    private:
        ros::NodeHandle nh_;
        ros::Subscriber sub_image_;
        ros::CallbackQueue queue_;
        ros::AsyncSpinner spinner_;
        unordered_map<int, pair<double, Eigen::Vector4d>> marker_info_map_; //id, size, handle position
        void imageCallback(const sensor_msgs::ImageConstPtr& image);
        Eigen::Matrix4d optic_in_camera_; // cam.inv * optic
        Eigen::Matrix4d camera_in_robot_;
        cv::Mat camera_matrix_; 
        cv::Mat distortion_;
        tf2_ros::TransformBroadcaster broadcaster_;
        tf2_ros::Buffer buffer_;
        tf2_ros::TransformListener listener_;

        //==========For visualization=======
        ros::Publisher pub_handle_pose_;
        //==================================
    public:
        ArucoDetector();

        ~ArucoDetector();
};

#endif