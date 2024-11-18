//
// Created by dzin on 19-3-7.
//

#ifndef PROJECT_ARUCO_DETECTION_H
#define PROJECT_ARUCO_DETECTION_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_calibration_parsers/parse_ini.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


class ArucoDetection{

public:

	struct CarMarkerInfo
	{

		int marker_id;
		// Marker ID

		cv::Vec3d rvec;

		cv::Vec3d tvec;

		std::vector<cv::Point2d> corners;

		cv::Vec3d pose_world;

		CarMarkerInfo(): marker_id(-1),
		    rvec(cv::Vec3d(0.0,0.0,0.0)),
		    tvec(cv::Vec3d(0.0,0.0,0.0)),
		    corners(4, cv::Point2d(0.0,0.0))
		{

		}

		cv::Point2d getCenter()
		{
			cv::Point2d center;
			center.x = 1.0/4.0*(corners[0].x + corners[1].x+ corners[2].x+ corners[3].x);
			center.y = 1.0/4.0*(corners[0].y + corners[1].y+ corners[2].y+ corners[3].y);
			return center;
		}
	};

	ArucoDetection();
	~ArucoDetection();

	ros::NodeHandle nh;

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

	cv::Mat intrinsics;
	cv::Mat distortion;
	int image_width;
	int image_height;
	std::string calib_filename_;

	//
	int dictionary_type;
	cv::Ptr<cv::aruco::DetectorParameters> detector_params;
	cv::Ptr<cv::aruco::Dictionary> dictionary;

	//car markers
    double car_marker_length;
    std::vector<int> car_markers_id;
    std::vector<cv::Vec3d> car_markers_rvec;
    std::vector<cv::Vec3d> car_markers_tvec;
    std::vector<cv::Point2f> car_center_pixel;
    std::vector<cv::Point2f> car_center_world;
    std::vector<float> car_theta;
    std::vector<cv::Vec3d> car_pose_world;

	//base marker
	float base_marker_length;
	int base_marker_id;
    cv::Vec3d base_marker_rvec;
    cv::Vec3d base_marker_tvec;
    int recalibrate;
    std::string base_marker_info;


    std::vector<cv::Vec3d> base_marker_tvecs_;

	tf::TransformListener listener_;
	tf::TransformBroadcaster broadcaster_;

	void imageCallback(const sensor_msgs::ImageConstPtr &original_image);

	void imageProcess(cv::Mat &input_image, cv::Mat &output_image);

	bool parseCalibrationFile(std::string filename);

    void pixelToBaseCoord(cv::Vec3d &pose, cv::Vec3d &rvec_base,
                            cv::Vec3d &tvec_base,  cv::Vec3d &rvec_marker, cv::Vec3d &tvec_marker);

	tf::Transform arucoMarker2Tf(cv::Vec3d &, cv::Vec3d &);
};


#endif //PROJECT_ARUCO_DETECTION_H
