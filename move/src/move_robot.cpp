//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>

ros::Publisher cmd_vel_publisher;
ros::Publisher hoover_state_pub;

void ballCallback(geometry_msgs::Point ball)
{
    float x_position = ball.x;
    float y_position = ball.y;
   // float radius = ball.z;
    float z_position = ball.z;


    geometry_msgs::Twist command;
    std_msgs::Int16 state;
    
    command.linear.x = 0;
	command.linear.y = 0;
	command.linear.z = 0;
	command.angular.x = 0;
	command.angular.y = 0;
	command.angular.z = 0;
    	state.data=1;

    if(x_position == 0){						// nie ma pilek, krecimy sie
	command.angular.z = -0.7;
	state.data = 1;
        ros::Duration(0.5).sleep();
    }

    if(y_position >= 400 && x_position >= 300 && x_position <= 340){	// STOP
        command.linear.x = 0.5;
        state.data=0;
        hoover_state_pub.publish(state);
        cmd_vel_publisher.publish(command);

        ros::Duration(3).sleep();

        command.linear.x = 0;
        cmd_vel_publisher.publish(command);

        ros::Duration(10).sleep();

    }
    
    if(y_position < 400 && x_position >= 210 && x_position <= 430){		// do przodu
		command.linear.x = 0.5;
    		state.data=1;
    }
    
    if(x_position >= 430 || (y_position >= 400 && x_position > 340)){												// w prawo
		command.angular.z = -0.7;
    		state.data=1;

    }
    
    if(x_position <= 210 || (y_position >= 400 & x_position < 300)){												// w lewo
		command.angular.z = 0.7;
    		state.data=1;
    }
    
    cmd_vel_publisher.publish(command);
    	hoover_state_pub.publish(state);
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n;

    ros::Subscriber ball_s = n.subscribe("the_ball", 1, ballCallback);
    cmd_vel_publisher = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    hoover_state_pub = n.advertise<std_msgs::Int16> ("/hoover_state",1);

    std_msgs::Int16 state;
    state.data=0;
    hoover_state_pub.publish(state);
    ros::spin();
}
