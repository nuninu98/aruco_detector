#include <aruco_detector/api_class/aruco_detector.h>

using namespace std;

ArucoDetector::ArucoDetector(): queue_(), spinner_(0, &queue_), listener_(buffer_),
optic_in_camera_(Eigen::Matrix4d::Identity()),camera_in_robot_(Eigen::Matrix4d::Identity()), got_camera_optic_tf_(false), got_camera_extrinsic_tf_(false){
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

    Eigen::Vector4d handle_in_marker(0.0, 0.0, 0.0, 1.0);
    marker_info_map_.insert(make_pair(0, make_pair(0.1, handle_in_marker)));
    
    handle_in_marker(0) = -0.185;
    handle_in_marker(2) = 0.1;
    marker_info_map_.insert(make_pair(1, make_pair(0.05, handle_in_marker)));

    handle_in_marker(0) = 0.205;
    handle_in_marker(2) = 0.1;
    marker_info_map_.insert(make_pair(2, make_pair(0.05, handle_in_marker)));

    // optic_in_camera_(0, 2) = 1.0;
    // optic_in_camera_(1, 0) = -1.0;
    // optic_in_camera_(2, 1) = -1.0;
    // optic_in_camera_(3, 3) = 1.0;
    ros::Time tic = ros::Time::now();
    geometry_msgs::TransformStamped optic_tf, cam_extrinsic;
    optic_tf.header.stamp = ros::Time(0.0);
    cam_extrinsic.header.stamp = ros::Time(0.0);
    
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
    tic = ros::Time::now();
    while((ros::Time::now() - tic).toSec() < 5.0){
        try
        {
            cam_extrinsic = buffer_.lookupTransform("base_link", "camera_link", ros::Time::now());
        }
        catch(const std::exception& e)
        {
            cam_extrinsic.header.stamp = ros::Time(0.0);
        }
        if(cam_extrinsic.header.stamp != ros::Time(0.0)){
            break;
        }
    }
    if(optic_tf.header.stamp != ros::Time(0.0)){
        got_camera_optic_tf_ = true;
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

    if(cam_extrinsic.header.stamp != ros::Time(0.0)){
        got_camera_extrinsic_tf_ = true;
        ROS_INFO_STREAM("Got TF base_link to camera_link");
        camera_in_robot_(0, 3) = cam_extrinsic.transform.translation.x;
        camera_in_robot_(1, 3) = cam_extrinsic.transform.translation.y;
        camera_in_robot_(2, 3) = cam_extrinsic.transform.translation.z;

        Eigen::Quaterniond q(cam_extrinsic.transform.rotation.w, cam_extrinsic.transform.rotation.x, 
        cam_extrinsic.transform.rotation.y, cam_extrinsic.transform.rotation.z);
        camera_in_robot_.block<3, 3>(0, 0) = q.toRotationMatrix();
    }
    else{
        ROS_WARN_STREAM("Failed to get transform base_link to camera_link. Use Default: Identity()");
        camera_in_robot_(0, 3) = 0.221;
        camera_in_robot_(1, 3) = 0.025;
        camera_in_robot_(2, 3) = 0.0;

        double yaw =-135.0 * (M_PI/180.0);
        camera_in_robot_(0, 0) = cos(yaw);
        camera_in_robot_(0, 1) = -sin(yaw);
        camera_in_robot_(1, 0) = sin(yaw);
        camera_in_robot_(1, 1) = cos(yaw);
    }

    pub_handle_vis_ = nh_.advertise<visualization_msgs::Marker>("handle_marker", 1);
    pub_handle_pose_ = nh_.advertise<geometry_msgs::Point>("handle_pose", 1);
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
    if(!got_camera_optic_tf_){
        geometry_msgs::TransformStamped tf_optic;
        tf_optic.header.frame_id = "camera_link";
        tf_optic.header.stamp = ros::Time::now();
        tf_optic.child_frame_id = "camera_color_optical_frame";
        tf_optic.transform.translation.x = optic_in_camera_(0, 3);
        tf_optic.transform.translation.y = optic_in_camera_(1, 3);
        tf_optic.transform.translation.z = optic_in_camera_(2, 3);

        Eigen::Quaterniond optic_quat(optic_in_camera_.block<3, 3>(0, 0));
        tf_optic.transform.rotation.w = optic_quat.w();
        tf_optic.transform.rotation.x = optic_quat.x();
        tf_optic.transform.rotation.y = optic_quat.y();
        tf_optic.transform.rotation.z = optic_quat.z();
        tfs.push_back(tf_optic);
    }
    if(!got_camera_extrinsic_tf_){
        geometry_msgs::TransformStamped tf_extrinsic;
        tf_extrinsic.header.frame_id = "base_link";
        tf_extrinsic.child_frame_id = "camera_link";
        tf_extrinsic.header.stamp = ros::Time::now();
        tf_extrinsic.transform.translation.x = camera_in_robot_(0, 3);
        tf_extrinsic.transform.translation.y = camera_in_robot_(1, 3);
        tf_extrinsic.transform.translation.z = camera_in_robot_(2, 3);

        Eigen::Quaterniond extrinsic_quat(camera_in_robot_.block<3, 3>(0, 0));
        tf_extrinsic.transform.rotation.w = extrinsic_quat.w();
        tf_extrinsic.transform.rotation.x = extrinsic_quat.x();
        tf_extrinsic.transform.rotation.y = extrinsic_quat.y();
        tf_extrinsic.transform.rotation.z = extrinsic_quat.z();
        tfs.push_back(tf_extrinsic);
    }
    Eigen::Vector4d handle_pose_in_cam(0.0, 0.0, 0.0, 0.0); // cam.inv * handle
    for(int i = 0; i < target_id.size(); i++){
        double marker_length = marker_info_map_[target_id[i]].first;
        vector<vector<cv::Point2f>> corner;
        corner.push_back(target_corner[i]);

        vector<cv::Vec3d> v_rotation, v_translation;
        cv::aruco::estimatePoseSingleMarkers(corner, marker_length, camera_matrix_, distortion_, v_rotation, v_translation);
        cv::Mat rotation_mat;
        Eigen::Matrix4d pose_se3 = Eigen::Matrix4d::Identity(); // cam_optic.inv * marker
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
        Eigen::Vector4d handle_in_marker = marker_info_map_[target_id[i]].second; //marker.inv * handle
        Eigen::Vector4d handle_in_cam_optic = pose_se3 * handle_in_marker; // cam_optic.inv * marker
        Eigen::Vector4d handle_in_cam = optic_in_camera_ * handle_in_cam_optic; 
        handle_pose_in_cam += handle_in_cam;
        //==================Testing tf================
        Eigen::Quaterniond q(pose_se3.block<3, 3>(0, 0));
        geometry_msgs::TransformStamped tf_msg;
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "camera_color_optical_frame";
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
    handle_pose_in_cam /= target_id.size();

    Eigen::Vector4d handle_pose_in_robot = camera_in_robot_ * handle_pose_in_cam;
    //===========Testing handle position=========
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.lifetime = ros::Duration(0);
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.color.a = 1.0;
    marker.color.r = 255.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.pose.position.x = handle_pose_in_robot(0);
    marker.pose.position.y = handle_pose_in_robot(1);
    marker.pose.position.z = handle_pose_in_robot(2);
    marker.pose.orientation.w = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    //===========================================
    pub_handle_vis_.publish(marker);

    geometry_msgs::PointStamped marker_pose_msg;
    marker_pose_msg.header.frame_id = "base_link";
    marker_pose_msg.header.stamp = ros::Time::now();
    marker_pose_msg.point.x = handle_pose_in_robot(0);
    marker_pose_msg.point.y = handle_pose_in_robot(1);
    marker_pose_msg.point.z = handle_pose_in_robot(2);
    pub_handle_pose_.publish(marker_pose_msg);
    broadcaster_.sendTransform(tfs);
    cv::imshow("marker_detection", detected_show);
    cv::waitKey(3);
}