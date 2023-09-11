#include <aruco_detector/api_class/aruco_detector.h>

using namespace std;

ArucoDetector::ArucoDetector(): queue_(), spinner_(0, &queue_), search_id_(0){
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
        if(marker_ids[i] == search_id_){
            target_id.push_back(search_id_);
            target_corner.push_back(marker_corners[i]);
        }
    }
    if(target_corner.size() != 1){
        // if(target_corner.empty()){
        //     ROS_WARN_THROTTLE(1.0, "Tag ID "<< to_string(search_id_)<<" is not found");
        // }
        // else{
        //     ROS_WARN_THROTTLE(1.0, "Tag ID "+ to_string(search_id_)+" is not found");
        // }
        return;
    }
    cv::aruco::drawDetectedMarkers(detected_show, target_corner, target_id);
    
    vector<cv::Vec3d> v_rotation, v_translation;
    cv::aruco::estimatePoseSingleMarkers(target_corner, 0.1, camera_matrix_, distortion_, v_rotation, v_translation);
    cout<<"RVEC: \n"<<v_rotation[0]<<endl;
    cout<<"TVEC: \n"<<v_translation[0]<<endl;
    cv::Mat rotation_mat;
    Eigen::Matrix4d pose_se3 = Eigen::Matrix4d::Identity();
    cv::Rodrigues(v_rotation[0], rotation_mat);
    cout<<"ROTATION: \n"<<rotation_mat<<endl;
    cout<<endl;
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
    tf_msg.child_frame_id = "marker";
    tf_msg.transform.translation.x = pose_se3(0, 3);
    tf_msg.transform.translation.y = pose_se3(1, 3);
    tf_msg.transform.translation.z = pose_se3(2, 3);
    tf_msg.transform.rotation.w = q.w();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();

    broadcaster_.sendTransform(tf_msg);
    //============================================

    cv::imshow("marker_detection", detected_show);
    cv::waitKey(3);
}