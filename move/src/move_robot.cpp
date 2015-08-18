//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>

ros::Publisher cmd_vel_publisher;
ros::Publisher hoover_state_pub;
ros::Publisher robot_state_publisher;
tf::TransformListener* tf_listener = NULL;

void searchCallback(std_msgs::Bool no_balls){

    geometry_msgs::Twist command; // predkosc
    std_msgs::Int16 state;        // odkurzacz

    std_msgs::Bool robot_state;
    robot_state.data = 1;
    robot_state_publisher.publish(robot_state);

    command.linear.x = 0;
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = 0.7;
    state.data=1;
    // publikacja zadanej predkosci i stanu odkurzacza
    cmd_vel_publisher.publish(command);
    hoover_state_pub.publish(state);
    ros::Duration(1).sleep();
    command.angular.z=0;
    cmd_vel_publisher.publish(command);

    robot_state.data = 0;
    robot_state_publisher.publish(robot_state);


}

void ballCallback(geometry_msgs::PointStamped ball)
{
    float x_position = ball.point.x;
    float y_position = ball.point.y;
    float z_position = ball.point.z;

    geometry_msgs::Twist command; // predkosc
    geometry_msgs::PointStamped pipe_link_ball;
    std_msgs::Int16 state;        // odkurzacz
    
    command.linear.x = 0;
    command.linear.y = 0;
    command.linear.z = 0;
    command.angular.x = 0;
    command.angular.y = 0;
    command.angular.z = 0;
    state.data=1;


    std_msgs::Bool robot_state;
    robot_state.data = 1;
    robot_state_publisher.publish(robot_state);

    ball.header.stamp = ros::Time::now();

    try {
        tf_listener->waitForTransform("/pipe_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
        tf_listener->transformPoint("/pipe_link", ball, pipe_link_ball);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }


    while(pipe_link_ball.point.y > 0.02){ // pileczka po lewej stronie
        command.angular.z = 0.7;

        // publikacja zadanej predkosci i stanu odkurzacza
        cmd_vel_publisher.publish(command);
        hoover_state_pub.publish(state);

        ball.header.stamp = ros::Time::now();
        // krecimy sie, az pileczka w ukladzie base_link bedzie miala prawie 0 na y
        try {
            tf_listener->waitForTransform("/pipe_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
            tf_listener->transformPoint("/pipe_link", ball, pipe_link_ball);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
    }
    command.angular.z = 0;

    while(pipe_link_ball.point.y < -0.02){ // pileczka po prawej stronie
        command.angular.z = -0.7;

        // publikacja zadanej predkosci i stanu odkurzacza
        cmd_vel_publisher.publish(command);
        hoover_state_pub.publish(state);

        ball.header.stamp = ros::Time::now();
        try {
            tf_listener->waitForTransform("/pipe_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
            tf_listener->transformPoint("/pipe_link", ball, pipe_link_ball);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
    }
    command.angular.z = 0;

    while(pipe_link_ball.point.x >= 0.05){ // pileczka z przodu
        command.linear.x = 0.5;

        // publikacja zadanej predkosci i stanu odkurzacza
        cmd_vel_publisher.publish(command);
        hoover_state_pub.publish(state);

        ball.header.stamp = ros::Time::now();
        try {
            tf_listener->waitForTransform("/pipe_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
            tf_listener->transformPoint("/pipe_link", ball, pipe_link_ball);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }
    }
    command.linear.x = 0;

    if(pipe_link_ball.point.x < 0.05 && pipe_link_ball.point.y < 0.02 && pipe_link_ball.point.y > -0.02){
	command.linear.x = 0.3;
	state.data = 0;
        cmd_vel_publisher.publish(command);
        hoover_state_pub.publish(state);
    	ros::Duration(3).sleep();
    }

    command.linear.x = 0;
    robot_state.data = 0;
    robot_state_publisher.publish(robot_state);


}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n;

    //ros::Subscriber ball_s = n.subscribe("the_ball", 1, ballCallback);
    ros::Subscriber ball_position = n.subscribe("ball_depth", 1, ballCallback);
    ros::Subscriber search_for_balls = n.subscribe("no_balls", 1, searchCallback);

    cmd_vel_publisher = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    hoover_state_pub = n.advertise<std_msgs::Int16> ("/hoover_state",1);

    tf_listener = new (tf::TransformListener);

    robot_state_publisher = n.advertise<std_msgs::Bool>("got_there", 1);

    std_msgs::Int16 state;
    state.data=0;
    hoover_state_pub.publish(state);
    ros::spin();
}
