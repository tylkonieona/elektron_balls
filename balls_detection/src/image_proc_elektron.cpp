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
#include <std_msgs/Float32.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf/transform_listener.h>

using namespace cv;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;
 
//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;
ros::Publisher balls;
ros::Publisher selected_ball;
image_geometry::PinholeCameraModel cam_model;
image_transport::Publisher circle_pub;
//tf::TransformListener tf_listener;
tf::TransformListener* tf_listener = NULL;

int ilepilek = 0;
int licznik = 0;
int licznik_depth=0;
//int wybrane_10[10][3];
int licznik_wybrane = 0;
int no_balls_counter = 0;

vector<Vec3f> circles_all;

Vec3f last_circle(0,0,0);


geometry_msgs::Point wybrana;

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

    /// Convert it to gray
      cvtColor( cv_ptr->image, src_gray, CV_BGR2GRAY );
      
      Sobel(src_gray, dst3, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT);
      convertScaleAbs( dst3, abs_dst3 );

      Sobel(src_gray, dst4, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT);
      convertScaleAbs( dst4, abs_dst4 );

      addWeighted( abs_dst3, 0.5, abs_dst4, 0.5, 0, dst );

      vector<Vec3f> circles;

      /// Apply the Hough Transform to find the circles
      HoughCircles(dst, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 80, 40, 3, 20);
      circles_all.insert(circles_all.end(), circles.begin(), circles.end());

      if(circles.size()>0) licznik_wybrane++;
      if(circles.size()==0) no_balls_counter++;
	
      if(no_balls_counter==10){
            no_balls_counter = 0;
	    licznik_wybrane = 0;
	    geometry_msgs::Point no_balls;
	    no_balls.x = 0;
	    no_balls.y = 0;
	    no_balls.z = 0;
            balls.publish(no_balls);
            std::cout << "Nie bylo pilek juz " << no_balls_counter << " razy. \n";
	    circle_pub.publish(cv_ptr->toImageMsg());
        circles_all.clear();

      }
      
      if(licznik_wybrane==10) //mamy cala tablice pilek
      {
        licznik_wybrane = 0;
	no_balls_counter = 0;

	std::cout << "10 razy byly pilki! \n";
        
        vector <int> pilki;
        for( int k = 0; k < circles_all.size(); k++ ){
            pilki.push_back(1);
        }

        for( size_t i = 0; i < circles_all.size(); i++ ){
            if (pilki[i]!=0){
                for(int j=i+1; j<circles_all.size(); j++){
                    if(pilki[j]!=0 && abs(circles_all[j][0]-circles_all[i][0])<=6 && abs(circles_all[j][1]-circles_all[i][1])<=10 &&
                            abs(circles_all[j][2]-circles_all[i][2])<=10){
                        pilki[j]=0;
                        pilki[i]++;// ta pilka to tak naprawde ta sama pilka
                    }
                }
            }
            i++;
		}

        // w tej chwili w pilki[] sa informacje o liczbie wystapien kazdej roznej pileczki
        // jesli jest 1, to za malo - niewiarygodne
        // trzeba wybrac najblizsza, z ponad jednym powtorzeniem

	// TODO
	// jak juz zbierzemy te prawdopodobne pilki
	// to trzeba sprawdzic, czy to faktycznie pilki
	// jak nie, to wstawic 0 w odpowiednie miejsce

        int closest = 0;
        int distance = 0;
        for(size_t i=0; i<circles_all.size(); i++){
            if(pilki[i]>1){
				if(abs(circles_all[i][0]-last_circle[0])<=6 && abs(circles_all[i][1]-last_circle[1])<=10 && abs(circles_all[i][2]-last_circle[2])<=10){
					distance=circles_all[i][2];
                    closest=i;
                    break;
				}
                if(circles_all[i][2]>=distance){
                    distance=circles_all[i][2];
                    closest=i;
                }
            }
        }

        int wybrana_x = circles_all[closest][0];
        int wybrana_y = circles_all[closest][1];
        int wybrana_z = circles_all[closest][2];
        
        last_circle[0] = wybrana_x;
        last_circle[1] = wybrana_y;
        last_circle[2] = wybrana_z;

        Point center_closest(cvRound(wybrana_x), cvRound(wybrana_y));
        int radius_closest = cvRound(wybrana_z);

        //draw
        // circle center
        circle( cv_ptr->image, center_closest, 3, Scalar(255,255,0), -1, 8, 0 );
        // circle outline
        circle( cv_ptr->image, center_closest, radius_closest, Scalar(255,0,255), 3, 8, 0 );

	circle_pub.publish(cv_ptr->toImageMsg());
        //TODO: tylko tutaj wysylac info o pileczce dalej
        //inaczej wysylaja sie smieci
        //geometry_msgs::Point wybrana;
        wybrana.x = wybrana_x;
        wybrana.y = wybrana_y;
        wybrana.z = wybrana_z;
     
       // balls.publish(wybrana);

        circles_all.clear();
        pilki.clear();
        
      }

    cv::waitKey(3);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
}


void depthCallback(const sensor_msgs::ImageConstPtr& original_image, const sensor_msgs::CameraInfoConstPtr& info)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Point2d uv_point(wybrana.x, wybrana.y);
    cam_model.fromCameraInfo(info);
    Mat depth_image = cv_ptr->image;
    unsigned short ball_depth;
//    std::cout << "u: " << wybrana.x << "  v: " << wybrana.y << "\n"; 

    ball_depth = depth_image.at<unsigned short>(wybrana.y,wybrana.x)+20;

//    std::cout << "Odleglosc: " << ball_depth << "\n";

    geometry_msgs::Point message_selected;
    cv::Point3d xy_point;
    xy_point = cam_model.projectPixelTo3dRay(uv_point);
    xy_point = xy_point * ball_depth;

    geometry_msgs::PointStamped xy_point_stamped, odom_point_stamped;

    xy_point_stamped.header.frame_id = "/camera_rgb_optical_frame";
    xy_point_stamped.header.stamp = ros::Time::now();

    xy_point_stamped.point.x = 0.001*xy_point.x;
    xy_point_stamped.point.y = 0.001*xy_point.y;
    xy_point_stamped.point.z = 0.001*xy_point.z;

    //tf::StampedTransform transform;

    try {
        tf_listener->waitForTransform("/base_link", "/camera_rgb_optical_frame", ros::Time::now(), ros::Duration(10.0) );
       // tf_listener.lookupTransform("/odom", "/camera_rgb_optical_frame", ros::Time(0), transform);
        tf_listener->transformPoint("/base_link", xy_point_stamped, odom_point_stamped);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }


    message_selected.x = odom_point_stamped.point.x;
    message_selected.y = odom_point_stamped.point.y;
//    std::cout << "x: " << message_selected.x << "   y: " << message_selected.y << "\n";
    message_selected.z = odom_point_stamped.point.z;
   // message_selected.x = xy_point_stamped.point.x;
   // message_selected.y = xy_point_stamped.point.y;
   // message_selected.z = xy_point_stamped.point.z;

    if(message_selected.z > 0.05){
	std::cout << "To nie pilka, jest za wysoko! \n";
    }
    	balls.publish(wybrana);
    	selected_ball.publish(message_selected);
   licznik_depth = 0;


}
 
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_processor");

    tf_listener = new (tf::TransformListener);

    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);

    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
	
	image_transport::CameraSubscriber sub_depth = it.subscribeCamera("/camera/depth_registered/image_raw", 1, depthCallback);

    balls = nh.advertise<geometry_msgs::Point>("the_ball", 1);
    selected_ball = nh.advertise<geometry_msgs::Point>("ball_depth", 1);

    circle_pub  = it.advertise("/camera/image_processed", 1);
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
