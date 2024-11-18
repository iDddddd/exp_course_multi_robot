#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <aruco_detection.h>

int main(int argc, char **argv)
{
    ros::init(argc,argv,"aruco_detection");
   // ros::NodeHandle nh;
    ArucoDetection obj;
    //Image node and subscriber
   // image_transport::ImageTransport it(nh);
   // image_transport::Subscriber img_sub = it.subscribe("/camera/color/image_raw", 10, &ArucoDetection::imageCallback, &obj);

    while(ros::ok())
    {
        ros::spinOnce();
    }

    return(EXIT_SUCCESS);
}
