#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>
#include <walls_detection/Walls.h>
#include <sensor_msgs/LaserScan.h>
#include <walls_detection/CheckWalls.h>

using namespace cv;
namespace enc = sensor_msgs::image_encodings;

//ros::Publisher walls;
image_geometry::PinholeCameraModel cam_model;
tf::TransformListener* tf_listener = NULL;
volatile int need_for_wall = 0;
ros::Publisher walls_all;
ros::Publisher walls_scan;
float actual_min = 100;


bool send_walls(walls_detection::CheckWalls::Request &req, walls_detection::CheckWalls::Response &res){
	std::cout << "\n\n\n\n\n\nChca ode mnie sciany\n\n\n\n\n\n";
	std_msgs::Float32 scanner_message;
	scanner_message.data = actual_min;
	res.walls = scanner_message;
	std::cout << "wyslalem im sciane \n\n\n";
	return true;


}


void needCallback(std_msgs::Bool need){
   // if(need.data==1){
	//need_for_wall = 1;
	//std::cout << "\n\n\n\n\n\nChca ode mnie sciany\n\n\n\n\n\n";
	//std_msgs::Float32 scanner_message;
	//scanner_message.data = actual_min;
	//walls_scan.publish(scanner_message);
	//std::cout << "wyslalem im sciane \n\n\n";
    //}

}

void laserCallback(sensor_msgs::LaserScan laser_image){

    //if(need_for_wall==1){
	//need_for_wall=0;
	//std::cout << "Angle_min: " << laser_image.angle_min << "\n";
	//std::cout << "Angle_max: " << laser_image.angle_max << "\n";
	//std::cout << "Angle_increment: " << laser_image.angle_increment << "\n";
	//std::cout << "Laser: " << laser_image.ranges.size() << "\n";
	float i_min_float = abs(laser_image.angle_min + 3.14/4)/laser_image.angle_increment;
	float i_max_float = abs(laser_image.angle_min - 3.14/4)/laser_image.angle_increment;
	int i_min = i_min_float;
	int i_max = i_max_float;
	float min = 100;
	for(int i = i_min; i <= i_max; i++){
		if(laser_image.ranges[i] < min){
			min = laser_image.ranges[i];
		}
	}
	actual_min = min;
	//std_msgs::Float32 scanner_message;
	//scanner_message.data = min;
	//walls_scan.publish(scanner_message);

    //}

}

void depthCallback(const sensor_msgs::ImageConstPtr& original_image, const sensor_msgs::CameraInfoConstPtr& info){

    if(need_for_wall==2){
        need_for_wall = 0;
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(original_image, enc::TYPE_16UC1);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
            return;
        }
        cam_model.fromCameraInfo(info);
        Mat depth_image = cv_ptr->image;

        float x = 0;
        float y = 360;
        float wall_height = 0;
        geometry_msgs::PointStamped xy_point_stamped, pipe_point_stamped;
        walls_detection::Walls walls_all_message;
        for(int i = 100; i<600; i+=110){
            wall_height = 0;
            y = 360;
            x = i;
            unsigned short wall_depth;
            while(wall_height < 0.05 && y > 0){
                    cv::Point2d uv_point(x,y);
                    unsigned short wall_depth;
                    wall_depth = depth_image.at<unsigned short>(y, x);
                    cv::Point3d xy_point;
                    xy_point = cam_model.projectPixelTo3dRay(uv_point);
                    xy_point = xy_point * wall_depth;

                    xy_point_stamped.header.frame_id = "/camera_rgb_optical_frame";
                    xy_point_stamped.header.stamp = ros::Time::now();

                    xy_point_stamped.point.x = 0.001*xy_point.x;
                    xy_point_stamped.point.y = 0.001*xy_point.y;
                    xy_point_stamped.point.z = 0.001*xy_point.z;

                    try {
                            tf_listener->waitForTransform("/pipe_link", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(10.0) );
                            tf_listener->transformPoint("/pipe_link", xy_point_stamped, pipe_point_stamped);
                    }
                    catch (tf::TransformException ex) {
                            ROS_ERROR("%s",ex.what());
                    }

                wall_height = pipe_point_stamped.point.z;
                y = y - 120;
            }
            std::cout<<"Sciana nr "<< i <<" - x: "<< pipe_point_stamped.point.x <<", y: "<< pipe_point_stamped.point.y <<", z: "<< pipe_point_stamped.point.z <<"\n";
            if(wall_height < 0.05){
                std::cout << "Nie ma sciany\n";
                pipe_point_stamped.point.x = 0;
                pipe_point_stamped.point.y = 0;
                pipe_point_stamped.point.z = 0;
            }
            switch(i){
                case 100:
                walls_all_message.wall1 = pipe_point_stamped;
                break;

                case 210:
                walls_all_message.wall2 = pipe_point_stamped;
                break;

                case 320:
                walls_all_message.wall3 = pipe_point_stamped;
                break;

                case 430:
                walls_all_message.wall4 = pipe_point_stamped;
                break;

                case 540:
                walls_all_message.wall5 = pipe_point_stamped;
                break;

            }

	}
    walls_all.publish(walls_all_message);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "walls_detector");

    tf_listener = new (tf::TransformListener);

    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    ros::Subscriber need_walls = nh.subscribe("/need_walls", 1, needCallback);
    ros::Subscriber laserSubscriber = nh.subscribe("/scan", 1, laserCallback);

    image_transport::CameraSubscriber sub_depth = it.subscribeCamera("/camera/depth_registered/image_raw", 1, depthCallback);

    walls_all = nh.advertise<walls_detection::Walls>("walls_all",1);
    walls_scan = nh.advertise<std_msgs::Float32>("walls_scan",1);

    ros::ServiceServer service = nh.advertiseService("need_walls",send_walls);

    ros::spin();

    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}
