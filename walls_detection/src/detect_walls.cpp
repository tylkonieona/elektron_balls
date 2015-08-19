#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>


using namespace cv;
namespace enc = sensor_msgs::image_encodings;


ros::Publisher walls;
image_geometry::PinholeCameraModel cam_model;
tf::TransformListener* tf_listener = NULL;
int need_for_wall = 0;


void needCallback(std_msgs::Bool need){
    if(need.data==1) need_for_wall = 1;
}


void depthCallback(const sensor_msgs::ImageConstPtr& original_image, const sensor_msgs::CameraInfoConstPtr& info){

    if(need_for_wall==1){
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

        float x = 320;
        float y = 180;
        cv::Point2d uv_point(x,y);
        unsigned short wall_depth;
        wall_depth = depth_image.at<unsigned short>(y, x);
        cv::Point3d xy_point;
        xy_point = cam_model.projectPixelTo3dRay(uv_point);
        xy_point = xy_point * wall_depth;

        geometry_msgs::PointStamped xy_point_stamped, pipe_point_stamped;

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

        std::cout << "Sciana jest w odleglosci " << pipe_point_stamped.point.x << " m. i jest na wysokosci: " << pipe_point_stamped.point.z << " m.\n";

        walls.publish(pipe_point_stamped);
        need_for_wall = 0;
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

    image_transport::CameraSubscriber sub_depth = it.subscribeCamera("/camera/depth_registered/image_raw", 1, depthCallback);

    walls = nh.advertise<geometry_msgs::PointStamped>("the_wall", 1);

    ros::spin();

    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");

}
