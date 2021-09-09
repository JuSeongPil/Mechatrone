#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <math.h>
#include "rohang2/GpsPoint.h"
#include "rohang2/CameraPoint.h"

cv::Mat cameraMatrix= cv::Mat::eye(3, 3, CV_64FC1);
cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64FC1);

rohang2::GpsPoint df;
rohang2::CameraPoint cf;
void dfCallback(const rohang2::GpsPoint::ConstPtr& msg){
    df = *msg;
    //std::cout << df.x << ", " << df.y << "-> " << df.state << std::endl;
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "tlqkf");
    ros::NodeHandle nh;

    ros::Publisher cam_loc_pub = nh.advertise<rohang2::CameraPoint>("cam_loc", 10);
    ros::Subscriber camera_sub = nh.subscribe<rohang2::GpsPoint>("cam_state", 10, dfCallback);
    ros::Rate rate(5.0); 
    rohang2::CameraPoint cf;

    ROS_INFO("START");
    cv::VideoCapture cam(0);

    double fps = 10.0;
    int fourcc = cv::VideoWriter::fourcc('M','J','P','G');
    cv::VideoWriter save;
    save.open("red_circle_detect.avi", fourcc, fps, cv::Size(640,480), true);

    cameraMatrix=(cv::Mat1d(3, 3) << 569.13851, 0., 327.37043, 0., 568.43361, 231.75003, 0., 0., 1. );
    distCoeffs=(cv::Mat1d(1, 5) << 0.03020, 0., -0.01369, 0.0224, 0);
    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size(640, 480), CV_32FC1, map1, map2);

    
    ROS_INFO("CAM SETTING");

    cv::Point img_center(static_cast<short>(320), static_cast<short>(240));
    cv::Mat image;

    if (!cam.isOpened()) {
    //    std::cout << "Cannot open camera" << std::endl;
        return -1;
    }
    ROS_INFO("CAM OPEN");

    while(ros::ok()) {
        cam.read(image);
        cv::resize(image, image, cv::Size(640, 480));
        remap(image, image, map1, map2, cv::INTER_LINEAR);
        ROS_INFO("WHILE");

        if (image.empty()) {
            break;
        }
        cv::flip(image,image,1);

        cv::Mat gray, mask;
        
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mask, 125, 255, cv::THRESH_BINARY);

        ros::spinOnce();
            if (df.state == 1) {
            cv::GaussianBlur(mask, mask, cv::Size(3, 3), 0);

            std::vector<cv::Vec3f> circles;
            cv::HoughCircles(mask, circles, cv::HOUGH_GRADIENT, 1, 100, 150, 30, 0, 0);

            if (circles.size() > 0) {	//sort -> show the biggest circle only
                std::sort(circles.begin(), circles.end(), [](cv::Vec3f& a, cv::Vec3f& b) {return a[2] >= b[2]; });

                cv::Vec3i c = circles[0];
                cv::Point circle_center(c[0], c[1]);
                int radius = c[2];

                cv::circle(image, circle_center, radius, cv::Scalar(255, 255, 255), 4);
                cv::circle(image, circle_center, 5, cv::Scalar(0, 0, 0), 5);

                

                double dis = sqrt(pow((circle_center - img_center).x, 2) + pow((circle_center - img_center).y, 2));
                if (dis >= 50) {	//circle_center is located outside
                    //cv::circle(image, circle_center, radius, cv::Scalar(0, 0, 255), 4);
                    //cv::circle(image, circle_center, 5, cv::Scalar(0, 0, 0), 5);
                    //cv::line(image, circle_center, img_center, cv::Scalar(0, 0, 255), 3, 8, 0);
                    cf.a = circle_center.x;
                    cf.b = circle_center.y;
                    cam_loc_pub.publish(cf);
                    rate.sleep();
                }
                else {	//circle_center is located inside
                    //cv::circle(image, circle_center, radius, cv::Scalar(0, 255, 0), 4);
                   //cv::circle(image, circle_center, 5, cv::Scalar(0, 0, 0), 5);
                    //cv::line(image, circle_center, img_center, cv::Scalar(255, 0, 0), 3, 8, 0);
                }
                
            }
        }
        

        cv::circle(image, img_center, 50, cv::Scalar(255, 255, 255), 2);

        //cv::imshow("cam",image);
        
        save << image;

        if (cv::waitKey(1000/fps)==27)
            break;
    }

    cv::destroyAllWindows();
    cam.release();

    return 0;
}