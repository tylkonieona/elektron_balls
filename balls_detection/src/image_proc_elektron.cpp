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


int ilepilek = 0;
int licznik = 0;
int licznik_depth=0;
int wybrane_10[10][3];
int licznik_wybrane = 0;

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
      HoughCircles(dst, circles, CV_HOUGH_GRADIENT, 1, dst.rows/8, 60, 30, 3, 20);
      circles_all.insert(circles_all.end(), circles.begin(), circles.end());

      if(circles.size()>0) licznik_wybrane++;
      
      if(licznik_wybrane==10) //mamy cala tablice pilek
      {
        licznik_wybrane=0;
        
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
     
        balls.publish(wybrana);

        circles_all.clear();
        //std::cout << "Pilek jest:" << ilepilek << "--------------------------------------\n";
        //for(size_t i=0; i++; i<circles_all.size()){
        //    std::cout << "\n\n\n------------------\n "<< circles_all[i];
        //}
        pilki.clear();

        
      }

	//namedWindow( "Pilki", CV_WINDOW_AUTOSIZE );
    //imshow( "Pilki", dst );
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window created and the window is active. If there are several HighGUI windows, any of them can be active.
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
   //int depth = original_image->data[wybrana.x][wybrana.y];

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        //Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (cv_bridge::Exception& e)
    {
        //if there is an error during conversion, display it
        ROS_ERROR("tutorialROSOpenCV::main.cpp::cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Point2d uv_point(wybrana.x, wybrana.y);
    cam_model.fromCameraInfo(info);
    Mat depth_image = cv_ptr->image;
    unsigned short ball_depth;
    std::cout << "u: " << wybrana.x << "  v: " << wybrana.y << "\n"; 
   // if(wybrana.x >= 2 && wybrana.y >= 2){
   //     for(int i=-2; i<=2; i++){
   //        for(int j=-2; j<=2; j++){
   //             if(ball_depth > depth_image.at<unsigned short>(wybrana.x+i, wybrana.y+j) && depth_image.at<unsigned short>(wybrana.x+i, wybrana.y+j) != 0){
                	ball_depth = depth_image.at<unsigned short>(wybrana.y,wybrana.x)+20;
	//	}
	        //std::cout << "Ball_depth: "<< ball_depth << "\n";
        //    }
       // }
    //}
   //else ball_depth = 0;
   //ball_depth = ball_depth/25;
   // ball_depth = 0.1236 * tan(ball_depth / 2842.5 + 1.1863);
   //ball_depth = depth_image.at<unsigned short>(wybrana.x, wybrana.y);
   std::cout << "Odleglosc: " << ball_depth << "\n";
    //ball_depth=1/(-0.00307 * ball_depth + 3.33);
    //int x_beg, y_beg, x_end, y_end;
    

    //if(wybrana.x <= 40) x_beg = 0;
    //else x_beg = wybrana.x-40;

    //if(wybrana.y <=40) y_beg = 0;
    //else y_beg = wybrana.y-40;

    //if(wybrana.x >= 600) x_end = 640;
    //else x_end = wybrana.x+40;

    //if(wybrana.y >= 440) y_end = 480;
    //else y_end = wybrana.y+40;
   
    //cv::Rect region_of_interest = Rect(x_beg, y_beg, 81, 81);

    //Mat ball_surr = depth_image(region_of_interest);
    
    //double depth_double, furth;

    //cv::minMaxLoc(ball_surr, &depth_double, &furth);
    //ball_depth = depth_double;
    //std::cout << "Ball_depth: " << ball_depth << "\n-------------------\n"; 
    geometry_msgs::Point message_selected;
    cv::Point3d xy_point;
    xy_point = cam_model.projectPixelTo3dRay(uv_point);
    xy_point = xy_point * ball_depth;
    message_selected.x = xy_point.x;
    message_selected.y = xy_point.y;
    //message_selected.x = 0;
    //message_selected.y = 0;
    std::cout << "x: " << message_selected.x << "   y: " << message_selected.y << "\n";
    message_selected.z = xy_point.z;
    selected_ball.publish(message_selected); 
   // pub.publish(cv_ptr->toImageMsg());
   licznik_depth = 0;


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

	//ros::Publisher vel_pub;
	//vel_pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
    //OpenCV HighGUI call to create a display window on start-up.
    //cv::namedWindow("Pilki", CV_WINDOW_AUTOSIZE);
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
        image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_color", 1, imageCallback);
	
	image_transport::CameraSubscriber sub_depth = it.subscribeCamera("/camera/depth_registered/image_raw", 1, depthCallback);

        balls = nh.advertise<geometry_msgs::Point>("the_ball", 100);
       selected_ball = nh.advertise<geometry_msgs::Point>("ball_depth", 100);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    //cv::destroyWindow("Pilki");
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
        circle_pub  = it.advertise("/camera/image_processed", 1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
        ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("tutorialROSOpenCV::main.cpp::No error.");
 
}
