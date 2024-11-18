#include <math.h>
#include <aruco_detection.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sstream>

#define pi 3.141592653579

char global_key;

float my_atan(cv::Point2d vector)
{
    float ang;

    if (vector.x == 0)
    {
        if(vector.y > 0)
            ang = CV_PI/2;//90°
        else
            ang = 3*CV_PI/2;//270°
    }

    else
    {
        if (vector.y/vector.x >= 0)
        {
            if(vector.y >= 0)
                ang = atan(vector.y/vector.x);//0~90°
            else
                ang = atan(vector.y/vector.x) + CV_PI;//180°~270°
        }
        else
        {
            if(vector.y < 0)
                ang = atan(vector.y/vector.x) + 2*CV_PI;//270°~360
            else
                ang = atan(vector.y/vector.x) + CV_PI;//90°～180°
        }
    }

    return ang;
}

ArucoDetection::ArucoDetection():
        it_(nh),
        calib_filename_("empty"),
        base_marker_length(0.195),  // in m
        recalibrate(-1),
        car_marker_length(0.1),
        dictionary_type(cv::aruco::DICT_5X5_50),
        intrinsics(cv::Mat::zeros(cv::Size(3,3), CV_64F)),
        distortion(cv::Mat::zeros(cv::Size(5,1), CV_64F)),
        image_width(1280),image_height(720),
        base_marker_id(0)
{
    this->image_sub_ = this->it_.subscribe("/camera/color/image_raw",1 ,&ArucoDetection::imageCallback, this);

    ///
	this->nh.getParam("aruco_detection/calibration_file", calib_filename_);


	///
    this->nh.getParam("aruco_detection/base_marker_info", base_marker_info);
    {
        cv::Mat rvec, tvec;
        cv::FileStorage fs;
        fs.open(base_marker_info, cv::FileStorage::READ);
        fs["base_marker_rvec"] >> rvec;
        fs["base_marker_tvec"] >> tvec;
        for(int i=0 ; i<3; i++)
        {
            this->base_marker_rvec(i) = rvec.at<float>(i);
            this->base_marker_tvec(i) = tvec.at<float>(i);
        }
        fs.release();
    }

	this->parseCalibrationFile(calib_filename_);

    ///
	detector_params = cv::aruco::DetectorParameters::create();
	detector_params->markerBorderBits=1;
	// detector_params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
	dictionary = cv::aruco::getPredefinedDictionary(dictionary_type);
	//
    cv::namedWindow("detected", cv::WINDOW_NORMAL);
	cv::resizeWindow("detected",  1280, 720);

}

ArucoDetection::~ArucoDetection()
{

}

bool ArucoDetection::parseCalibrationFile(std::string filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);

    if(fs.isOpened())
    {

        fs["intrinsics"] >> this->intrinsics;
        fs["distortion"] >> this->distortion;

        std::cout << "intrinsics " << this->intrinsics <<  std::endl;
        std::cout << "distortion " << this->distortion << std::endl;
    }

    fs.release();


    //Simple check if calibration data meets expected values
    if (! ((intrinsics.at<double>(2,2) == 1) && (distortion.at<double>(0,4) == 0)))
    {
        ROS_INFO_STREAM("Calibration data loaded successfully");
        return false;
    }
}


void ArucoDetection::imageCallback(const sensor_msgs::ImageConstPtr &original_image)
{

	cv_bridge::CvImagePtr cv_ptr;

	try
	{
		cv_ptr=cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("Not able to convert sensor_msgs::Image to OpenCV::Mat format %s", e.what());
		return;
	}

	cv::Mat src_image = cv_ptr->image.clone();

	cv::Mat output_image;
	src_image.copyTo(output_image);

	imageProcess(src_image, output_image);
	cv::imshow("detected", output_image);

	global_key = cv::waitKey(20);

	if(global_key == 'c')
    {
	    recalibrate = 0;
    }
    if(global_key == 's')
    {
        recalibrate = 1;
    }

}

void ArucoDetection::imageProcess(cv::Mat &input_image, cv::Mat &output_image)
{


    this->car_markers_id.clear();
    this->car_markers_rvec.clear();
    this->car_markers_tvec.clear();
    this->car_pose_world.clear();

	//marker board
	std::vector<int> marker_ids;
	std::vector<std::vector<cv::Point2f>> marker_corners;

	//detect marker board on table
	cv::aruco::detectMarkers(input_image, this->dictionary, marker_corners, marker_ids, this->detector_params);
    if(marker_ids.size() == 0)
    {
        ROS_DEBUG("No board marker found!");
        return;
    }

	cv::aruco::drawDetectedMarkers(output_image, marker_corners, marker_ids, cv::Scalar(0,255,0));

    std::vector<cv::Vec3d> rvec;
    std::vector<cv::Vec3d> tvec;
    cv::aruco::estimatePoseSingleMarkers(marker_corners, this->car_marker_length, this->intrinsics, this->distortion, rvec, tvec);

    //每个车的marker_id不同

    tf::Transform trans;
    tf::Quaternion qt;
    tf::Matrix3x3 tfMat;
    std::cout << "marker num: " << marker_ids.size()<< std::endl;

    //-1: 还没有检测到marker

    for(int i=0; i<marker_ids.size(); i++)
    {

        if( marker_ids[i] == this->base_marker_id )
        {
            if(recalibrate == 0)
            {
                std::cout << "base marker id: " << marker_ids[i] << std::endl;
                std::vector<std::vector<cv::Point2f>> corner;
                corner.push_back(marker_corners[i]);

                std::vector<cv::Vec3d> rvec_;
                std::vector<cv::Vec3d> tvec_;
                cv::aruco::estimatePoseSingleMarkers(corner, this->base_marker_length,
                                                     this->intrinsics, this->distortion, rvec_, tvec_);
                this->base_marker_rvec = rvec_[0];
                this->base_marker_tvec = tvec_[0];
            }
            if(recalibrate == 1)
            {
                cv::Mat rvec = cv::Mat::zeros(3,1,CV_32F);
                cv::Mat tvec = cv::Mat::zeros(3,1,CV_32F);

                for(int i=0 ; i<3; i++)
                {
                    rvec.at<float>(i) = this->base_marker_rvec(i);
                    tvec.at<float>(i) = this->base_marker_tvec(i);
                }

                cv::FileStorage fs;
                fs.open(base_marker_info, cv::FileStorage::WRITE);
                fs << "base_marker_rvec" << rvec;
                fs << "base_marker_tvec" << tvec;
                fs.release();

                std::cout << "finish calibration " << std::endl;

                recalibrate = -1;

            }
            //////////////////////////////////////////////////////////////////
        }

        {
            cv::aruco::drawAxis(output_image, this->intrinsics, this->distortion, this->base_marker_rvec, this->base_marker_tvec, 0.5);

            //////////////////////////////////////////////////////////////////
//            trans.setOrigin(tf::Vector3(base_marker_tvec(0),base_marker_tvec(1), base_marker_tvec(2)));
//            cv::Mat rotMat;
//            cv::Rodrigues(this->base_marker_rvec, rotMat);
//            // tf::Matrix3x3[i][j]  i行j列
//            for(int i=0 ;i<3; i++){
//                for(int j=0; j<3; j++){
//                    tfMat[i][j] = rotMat.at<double>(i,j);
//                }
//            }
//
//        //    ROS_INFO_STREAM("tfMat " << tfMat[0][0] << " "<< tfMat[1][1] << " "<< tfMat[2][2] << " " );
//            trans.setBasis(tfMat);
//            this->broadcaster_.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "world", "base_marker"));

        }

        {
            trans.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
            qt.setRPY(0.0, 0.0, 0.0);

            trans.setRotation(qt);
            this->broadcaster_.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "map", "base_marker"));
        }


        this->car_markers_id.push_back(marker_ids[i]);
        this->car_markers_rvec.push_back(rvec[i]);
        this->car_markers_tvec.push_back(tvec[i]);

        cv::aruco::drawAxis(output_image, this->intrinsics, this->distortion, rvec[i], tvec[i], 0.2);

        cv::Vec3d pose_world;
        pixelToBaseCoord(pose_world, this->base_marker_rvec, this->base_marker_tvec, rvec[i], tvec[i]);
        std::cout <<"car marker id: " << marker_ids[i] <<" pose is "<<"x y:  "<<pose_world(0) << "   "
                <<pose_world(1) << " m  theta: " << pose_world(2)*180.0/M_PI << " degree" << std::endl;

        this->car_pose_world.push_back(pose_world);


		trans.setOrigin(tf::Vector3(pose_world(0),pose_world(1), 0.0));
		qt.setRPY(0, 0, pose_world(2));

		trans.setRotation(qt);
		std::stringstream ss;

		ss<< "robot_"<< marker_ids[i];
		this->broadcaster_.sendTransform(tf::StampedTransform(trans, ros::Time::now(), "base_marker", ss.str()));
    }

}


tf::Transform ArucoDetection::arucoMarker2Tf(cv::Vec3d &rvec, cv::Vec3d &tvec)
{
	//CV
	cv::Mat cv_rotation_matrix(cv::Mat::zeros(cv::Size(3,3),CV_64F));
	cv::Rodrigues(rvec, cv_rotation_matrix);
	std::cout << "\ncv_rotation_matrix:\n" << cv_rotation_matrix << std::endl;
	std::cout << "\ncv_rotation_matrix double (0,0):\n" << cv_rotation_matrix.at<double>(0,0) << std::endl;

	//tf
	tf::Matrix3x3 tf_rotation_matrix(
		cv_rotation_matrix.at<double>(0,0),cv_rotation_matrix.at<double>(0,1),cv_rotation_matrix.at<double>(0,2),
		cv_rotation_matrix.at<double>(1,0),cv_rotation_matrix.at<double>(1,1),cv_rotation_matrix.at<double>(1,2),
		cv_rotation_matrix.at<double>(2,0),cv_rotation_matrix.at<double>(2,1),cv_rotation_matrix.at<double>(2,2));
	double roll, pitch, yaw;
	tf_rotation_matrix.getRPY(roll,pitch,yaw);
	std::cout << "\n                                                                                                           tf RPY -roll(X) -pitch(Y) -yaw(Z) = "<< roll* 180 / pi <<" "<< pitch* 180 / pi <<" "<< yaw* 180 / pi << std::endl;
	tf::Quaternion q ;
	q.setRPY(roll,pitch,yaw);
	// q.setEuler(yaw,pitch,roll);
	// tf_rotation_matrix.getRotation(q);
	q.normalize();
	std::cout<< "\n                                                                                                            tf Quaternion -q = : "<<q.getX()<<" "<<q.getY()<<" "<<q.getZ()<<" "<<q.getW()<<std::endl;

	if (std::isnan(q.getX()) || std::isnan(q.getY()) || std::isnan(q.getZ()) || std::isnan(q.getW()))
	{
		std::cout << "[Quaternion of tf is NAN NAN NAN NAN!]"<< std::endl;
		q.setX(1);
		q.setY(1);
		q.setZ(1);
		q.setW(1);
		q.normalize();
		tf::Matrix3x3 marker_tf_rot_q(q);
		tf::Vector3 marker_tf_tran(tvec[0],	tvec[1], tvec[2]);
		return tf::Transform(marker_tf_rot_q, marker_tf_tran);
	}
	else
	{
		tf::Matrix3x3 marker_tf_rot_q(q);
		tf::Vector3 marker_tf_tran(tvec[0],	tvec[1], tvec[2]);
		return tf::Transform(marker_tf_rot_q, marker_tf_tran);
	}
}

void ArucoDetection::pixelToBaseCoord(cv::Vec3d &pose, cv::Vec3d &rvec_base,
                                        cv::Vec3d &tvec_base,  cv::Vec3d &rvec_marker, cv::Vec3d &tvec_marker)
{

    /////////////////////////////////////////////////
    //计算小车在 basemarker 下的坐标
    cv::Mat rotate_base(cv::Mat::zeros(cv::Size(3,3),CV_64F));
    cv::Rodrigues(rvec_base, rotate_base);

    cv::Mat trans_base(cv::Mat::zeros(cv::Size(1,3), CV_64F));
    trans_base.at<double>(0) = tvec_base(0);
    trans_base.at<double>(1) = tvec_base(1);
    trans_base.at<double>(2) = tvec_base(2);

    /////////////////////////////////////////
    //计算小车x轴在 basemarker 下的偏角
    cv::Mat rotate_marker(cv::Mat::zeros(cv::Size(3,3),CV_64F));
    cv::Rodrigues(rvec_marker, rotate_marker);

    cv::Mat trans_marker(cv::Mat::zeros(cv::Size(1,3), CV_64F));
    trans_marker.at<double>(0) = tvec_marker(0);
    trans_marker.at<double>(1) = tvec_marker(1);
    trans_marker.at<double>(2) = tvec_marker(2);

    cv::Mat car_xaxis(cv::Mat::zeros(cv::Size(1,3), CV_64F));
    car_xaxis.at<double>(0) = 1.0;
    car_xaxis.at<double>(1) = 0.0;
    car_xaxis.at<double>(2) = 0.0;
    cv::Mat car_origin(cv::Mat::zeros(cv::Size(1,3), CV_64F));


    cv::Mat xaxis_base = rotate_base.t()*(rotate_marker*car_xaxis + trans_marker - trans_base);
    cv::Mat origin_base = rotate_base.t()*(rotate_marker*car_origin + trans_marker - trans_base);

    cv::Mat dir = xaxis_base - origin_base;

    cv::Point2d vec;
    vec.x = dir.at<double>(0);
    vec.y = dir.at<double>(1);

    float theta = my_atan(vec);

    pose(0) = origin_base.at<double>(0);;
    pose(1) = origin_base.at<double>(1);;
    pose(2) = theta;

}
