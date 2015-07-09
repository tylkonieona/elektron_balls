//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>

//using namespace cv;

ros::Publisher cmd_vel_publisher;

void ballCallback(geometry_msgs::Point ball)
{
    float x_position = ball.x;
    float y_position = ball.y;
    float radius = ball.z;


    geometry_msgs::Twist command;
    
    command.linear.x = 0;
	command.linear.y = 0;
	command.linear.z = 0;
	command.angular.x = 0;
	command.angular.y = 0;
	command.angular.z = 0;

    if(y_position >= 420 && x_position >= 210 && x_position <= 420){	// STOP
   	 //command.linear.x = 10;
   	 //command.angular.z = y_position/100;
    }
    
    if(y_position < 420 && x_position >= 210 && x_position <= 420){		// do przodu
		command.linear.x = 1;
    }
    
    if(x_position >= 420){												// w prawo
		command.angular.z = -1;
    }
    
    if(x_position <= 210){												// w lewo
		command.angular.z = 1;
    }
    

    cmd_vel_publisher.publish(command);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n;

    ros::Subscriber ball_s = n.subscribe("the_ball", 1, ballCallback);
    cmd_vel_publisher = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    ros::spin();
}
