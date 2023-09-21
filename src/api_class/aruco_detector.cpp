#include <aruco_detector/api_class/aruco_detector.h>

using namespace std;

ArucoDetector::ArucoDetector(): queue_(), spinner_(0, &queue_), listener_(buffer_),
optic_in_camera_(Eigen::Matrix4d::Identity()),camera_in_robot_(Eigen::Matrix4d::Identity()){
    nh_.setCallbackQueue(&queue_);
    sub_image_ = nh_.subscribe("camera/color/image_raw", 1, &ArucoDetector::imageCallback, this);
    spinner_.start();
    
    camera_matrix_ = cv::Mat::eye(3, 3, CV_32FC1);
    camera_matrix_.at<float>(0, 0) = 932.24106;
    camera_matrix_.at<float>(0, 2) = 662.775813;
    camera_matrix_.at<float>(1, 1) = 932.79107;
    camera_matrix_.at<float>(1, 2) = 348.316303;

    distortion_ = cv::Mat::zeros(1, 5, CV_32FC1);
    distortion_.at<float>(0, 0) = 0.123765;
    distortion_.at<float>(0, 1) = -0.264373;
    distortion_.at<float>(0, 2) = -0.005782;
    distortion_.at<float>(0, 3) = 0.002051;

    marker_info_map_.insert(make_pair(0, make_pair(0.1, Eigen::Vector4d::Identity())));
    marker_info_map_.insert(make_pair(1, make_pair(0.05, Eigen::Vector4d::Identity())));
    marker_info_map_.insert(make_pair(2, make_pair(0.05, Eigen::Vector4d::Identity())));

    // optic_in_camera_(0, 2) = 1.0;
    // optic_in_camera_(1, 0) = -1.0;
    // optic_in_camera_(2, 1) = -1.0;
    // optic_in_camera_(3, 3) = 1.0;
    ros::Time tic = ros::Time::now();
    geometry_msgs::TransformStamped optic_tf;
    optic_tf.header.stamp = ros::Time(0.0);
    while((ros::Time::now() - tic).toSec() < 5.0){
        try
        {
            optic_tf = buffer_.lookupTransform("camera_link", "camera_color_optical_frame", ros::Time::now());
        }
        catch(const std::exception& e)
        {
            optic_tf.header.stamp = ros::Time(0.0);
        }
        if(optic_tf.header.stamp != ros::Time(0.0)){
            break;
        }
    }
    if(optic_tf.header.stamp != ros::Time(0.0)){
        ROS_INFO_STREAM("Got TF camera_link to camera_color_optical_frame");
        optic_in_camera_(0, 3) = optic_tf.transform.translation.x;
        optic_in_camera_(1, 3) = optic_tf.transform.translation.y;
        optic_in_camera_(2, 3) = optic_tf.transform.translation.z;

        Eigen::Quaterniond q(optic_tf.transform.rotation.w, optic_tf.transform.rotation.x, 
        optic_tf.transform.rotation.y, optic_tf.transform.rotation.z);
        optic_in_camera_.block<3, 3>(0, 0) = q.toRotationMatrix();
    }
    else{
        ROS_WARN_STREAM("Failed to get transform camera_link to camera_color_optical_frame. Use Default");
        optic_in_camera_(0, 0) = 0.0;
        optic_in_camera_(0, 1) = 0.0;
        optic_in_camera_(0, 2) = 1.0;

        optic_in_camera_(1, 0) = -1.0;
        optic_in_camera_(1, 1) = 0.0;
        optic_in_camera_(1, 2) = 0.0;

        optic_in_camera_(2, 0) = 0.0;
        optic_in_camera_(2, 1) = -1.0;
        optic_in_camera_(2, 2) = 0.0;
    }
    
    cout<<"CAMERA MAT: "<<camera_matrix_<<endl;
    cout<<"DISTORTION: "<<distortion_<<endl;
}

ArucoDetector::~ArucoDetector(){
    spinner_.stop();
}

void ArucoDetector::imageCallback(const sensor_msgs::ImageConstPtr& image){
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);
    cv::Mat image_mat = cv_ptr->image;
    vector<int> marker_ids, target_id;
    vector<vector<cv::Point2f>> marker_corners, rejected_candidates, target_corner;
    cv::Ptr<cv::aruco::DetectorParameters> param = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dict = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
    cv::aruco::detectMarkers(image_mat, dict, marker_corners, marker_ids, param, rejected_candidates);
    cv::Mat detected_show = image_mat.clone();
    for(size_t i = 0; i < marker_ids.size(); i++){
        if(marker_info_map_.find(marker_ids[i]) != marker_info_map_.end()){
            target_id.push_back(marker_ids[i]);
            target_corner.push_back(marker_corners[i]);
        }
    }
    if(target_corner.empty()){
        cv::imshow("marker_detection", detected_show);
        cv::waitKey(3);
        return;
    }
    cv::aruco::drawDetectedMarkers(detected_show, target_corner, target_id);
    vector<geometry_msgs::TransformStamped> tfs;
    //====================For Test=======================
    geometry_msgs::TransformStamped tf_optic;
    tf_optic.header.frame_id = "rs_camera";
    tf_optic.header.stamp = ros::Time::now();
    tf_optic.child_frame_id = "camera_optic";
    tf_optic.transform.translation.x = 0.0;
    tf_optic.transform.translation.y = 0.0;
    tf_optic.transform.translation.z = 0.0;

    Eigen::Quaterniond optic_quat(optic_in_camera_.block<3, 3>(0, 0));
    tf_optic.transform.rotation.w = optic_quat.w();
    tf_optic.transform.rotation.x = optic_quat.x();
    tf_optic.transform.rotation.y = optic_quat.y();
    tf_optic.transform.rotation.z = optic_quat.z();
    tfs.push_back(tf_optic);
    //====================================================

    for(int i = 0; i < target_id.size(); i++){
        double marker_length = marker_info_map_[target_id[i]].first;
        vector<vector<cv::Point2f>> corner;
        corner.push_back(target_corner[i]);

        vector<cv::Vec3d> v_rotation, v_translation;
        cv::aruco::estimatePoseSingleMarkers(corner, marker_length, camera_matrix_, distortion_, v_rotation, v_translation);
        cv::Mat rotation_mat;
        Eigen::Matrix4d pose_se3 = Eigen::Matrix4d::Identity();
        cv::Rodrigues(v_rotation[0], rotation_mat);
        
        cv::drawFrameAxes(detected_show, camera_matrix_, distortion_, v_rotation, v_translation, 0.1);
        for(size_t r= 0; r < 3; r++){
            for(size_t c = 0; c < 3; c++){
                pose_se3(r, c) = rotation_mat.at<double>(r, c);
            }
        }
        pose_se3(0, 3) = v_translation[0](0);
        pose_se3(1, 3) = v_translation[0](1);
        pose_se3(2, 3) = v_translation[0](2);

        //==================Testing tf================
        Eigen::Quaterniond q(pose_se3.block<3, 3>(0, 0));
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "camera_optic";
        tf_msg.child_frame_id = "marker" + to_string(target_id[i]);
        tf_msg.transform.translation.x = pose_se3(0, 3);
        tf_msg.transform.translation.y = pose_se3(1, 3);
        tf_msg.transform.translation.z = pose_se3(2, 3);
        tf_msg.transform.rotation.w = q.w();
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tfs.push_back(tf_msg);
        //====================================
    }
    
    broadcaster_.sendTransform(tfs);
    cv::imshow("marker_detection", detected_show);
    cv::waitKey(3);
}