#include <aruco_detector/api_class/aruco_detector.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "test_marker_detector");
    ArucoDetector detector;

    while(ros::ok()){
        ros::spinOnce();
    }
}