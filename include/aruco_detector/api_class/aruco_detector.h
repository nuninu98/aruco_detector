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
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <Eigen/Dense>
using namespace std;

class ArucoDetector{
    private:
        int search_id_;
        ros::NodeHandle nh_;
        ros::Subscriber sub_image_;
        ros::CallbackQueue queue_;
        ros::AsyncSpinner spinner_;
        void imageCallback(const sensor_msgs::ImageConstPtr& image);

        cv::Mat camera_matrix_; 
        cv::Mat distortion_;
        tf2_ros::TransformBroadcaster broadcaster_;
    public:
        ArucoDetector();

        ~ArucoDetector();
};

#endif