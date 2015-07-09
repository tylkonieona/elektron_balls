//Includes all the headers necessary to use the most common public pieces of the ROS system.
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

using namespace cv;


//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher balls;


int ilepilek = 0;
int licznik = 0;


//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image)
{
    //Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }

    Mat orig_image;
    orig_image = cv_ptr->image;

    Mat src_gray, dst, dst2, dst3, dst4, abs_dst3, abs_dst4;

    Mat grad;

    //src = imread( cv_ptr->image, 1 );

    /// Convert it to gray
      cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );

      Sobel(src_gray, dst3, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
      convertScaleAbs( dst3, abs_dst3 );

      Sobel(src_gray, dst4, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
      convertScaleAbs( dst4, abs_dst4 );

      addWeighted( abs_dst3, 0.5, abs_dst4, 0.5, 0, dst );

      /// Reduce the noise so we avoid false circle detection
      //GaussianBlur( src_gray, src_gray, Size(5, 5), 2, 2 );

      vector<Vec3f> circles;

      /// Apply the Hough Transform to find the circles
     HoughCircles( dst, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 60, 30, 3, 20);

      /// Draw the circles detected
      int closest = 100;
      int closest_radius = 0;
      for( size_t i = 0; i < circles.size(); i++ )
      {
          Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
          int radius = cvRound(circles[i][2]);
          if(radius>closest_radius){
		closest_radius = radius;
		closest = i;
	  }
          // circle center
          circle( dst, center, 3, Scalar(255,255,0), -1, 8, 0 );
          // circle outline
          circle( dst, center, radius, Scalar(255,0,255), 3, 8, 0 );
          ilepilek++;
       }
      if(licznik==10)
      {
        std::cout << "Pilek jest:" << ilepilek << "\n";
        licznik=0;
      }
      else licznik++;

      if(ilepilek>0)
      {
	geometry_msgs::Point wybrana;
	wybrana.x = circles[closest][0];
	wybrana.y = circles[closest][1];
	wybrana.z = circles[closest][2];
      
        std::cout << "Promien:" << wybrana.z << "\n";
	balls.publish(wybrana);
      }

      else{
	geometry_msgs::Point wybrana;
	wybrana.x=0;
	wybrana.y=0;
	wybrana.z=0;

	balls.publish(wybrana);
      }
      ilepilek=0;


    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
}
 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    /**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
    * The name used here must be a base name, ie. it cannot have a / in it.
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
        ros::init(argc, argv, "image_processor");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
        ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
        image_transport::ImageTransport it(nh);

//	ros::Publisher vel_pub;
//	vel_pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    //OpenCV HighGUI call to create a display window on start-up.
//    cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
        image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);

        balls = nh.advertise<geometry_msgs::Point>("the_ball", 100);
    //OpenCV HighGUI call to destroy a display window on shut-down.
//    cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
        pub = it.advertise("/camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
