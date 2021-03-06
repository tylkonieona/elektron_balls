#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <tf/transform_listener.h>
#include <walls_detection/Walls.h>
#include <walls_detection/CheckWalls.h>

#define sqr(x) ((x)*(x))

ros::Publisher cmd_vel_publisher;
ros::Publisher hoover_state_pub;
ros::Publisher robot_state_publisher;
ros::Publisher wall_need_publisher;
ros::ServiceClient client;
tf::TransformListener* tf_listener = NULL;
bool wall_ready = 0;
geometry_msgs::PointStamped base_point;
geometry_msgs::PointStamped base_point_odom;

//geometry_msgs::PointStamped wall_place;
void scanCallback(std_msgs::Float32 wall_scanner){
	std::cout << "Jestem w callbacku\n";

}

void wallsCallback(walls_detection::Walls walls_message){
    //wall_place = wall_message;
	std::cout << "Jestem w callbacku\n";
}

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
    command.angular.z = 1;
    state.data=1;

    // publikacja zadanej predkosci i stanu odkurzacza
    cmd_vel_publisher.publish(command);
    hoover_state_pub.publish(state);
    ros::Duration(2).sleep();
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
    cmd_vel_publisher.publish(command);

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
    cmd_vel_publisher.publish(command);

    if(pipe_link_ball.point.x >= 0.05){ // pileczka z przodu

        // zanim pojedziemy, zapisac w ukladzie /odom nasza pozycje w /base_link (czyli 0,0,0?)
        base_point.header.stamp = ros::Time::now();
        base_point.header.frame_id = "/base_link";
        base_point.point.x = 0;
        base_point.point.y = 0;
        base_point.point.z = 0;

        try {
            tf_listener->waitForTransform("/odom", "/base_link", ros::Time::now(), ros::Duration(10.0) );
            tf_listener->transformPoint("/odom", base_point, base_point_odom);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }

        //zapytac o sciany
        //std_msgs::Bool message;
        //message.data = 1;
        //wall_need_publisher.publish(message);
	//std::cout << "Wyslalem zapytanie o sciany\n";

        //boost::shared_ptr<std_msgs::Float32 const> sharedPtr;
        std_msgs::Float32 scanner_wall;

        //sharedPtr  = ros::topic::waitForMessage<std_msgs::Float32>("walls_scan", ros::Duration(20));
        //if (sharedPtr == NULL)
        //    std::cout << "No messages received";
        //else{
        //    scanner_wall = *sharedPtr;
        //    std::cout << "Dostalem wiadomosc!";
        //}

	walls_detection::CheckWalls srv;
	srv.request.need_walls.data = 1;
	if(client.call(srv)){
		scanner_wall = srv.response.walls;
		std::cout << "Dostalem odpowiedz\n";
	}
	else{
		std::cout < "Nie udalo sie polaczyc z serwisem\n";
	}

	geometry_msgs::PointStamped laser_link_ball;	
        ball.header.stamp = ros::Time::now();
        try {
            tf_listener->waitForTransform("/base_laser_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
            tf_listener->transformPoint("/base_laser_link", ball, laser_link_ball);
        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s",ex.what());
        }


        if(laser_link_ball.point.x < scanner_wall.data){ // nie ma scian
            std::cout << "Zadnych scian! Whee!\n";
            std::cout << "Pileczka w odleglosci: " << pipe_link_ball.point.x << ".\n";
        }
        else{                                   // sa jakies sciany
            // obliczyc odleglosc pileczki od punktow na scianach

                std::cout << "Sciana, nigdzie nie jade :( \n";
                command.angular.z = 0.9;
                state.data=1;

		for(int i=0; i<14; i++){

                	cmd_vel_publisher.publish(command);
                	hoover_state_pub.publish(state);
                	ros::Duration(1).sleep();
		}

                command.angular.z = 0;
                robot_state.data = 0;
                cmd_vel_publisher.publish(command);
                robot_state_publisher.publish(robot_state);

                return;
        }
    }

    while(pipe_link_ball.point.x >= 0.05){ // pileczka z przodu

        command.linear.x = 0.5;
        state.data=1;

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
    command.angular.z = 0;
    state.data=1;


	command.linear.x = 0.3;
	state.data = 0;
    cmd_vel_publisher.publish(command);
    hoover_state_pub.publish(state);
    ros::Duration(3).sleep();

    // w tym momencie pilka jest juz wciagnieta, wiec trzeba sie cofnac do pozycji bazowej

    base_point_odom.header.stamp = ros::Time::now();
    try {
        tf_listener->waitForTransform("/base_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
        tf_listener->transformPoint("/base_link", base_point_odom, base_point);
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    while(base_point.point.x <= 0){
          command.linear.x = -0.3;
	  state.data=1;
          base_point_odom.header.stamp = ros::Time::now();
    	  cmd_vel_publisher.publish(command);
    	  hoover_state_pub.publish(state);

          try {
              tf_listener->waitForTransform("/base_link", "/odom", ros::Time::now(), ros::Duration(10.0) );
              tf_listener->transformPoint("/base_link", base_point_odom, base_point);
          }
          catch (tf::TransformException ex) {
              ROS_ERROR("%s",ex.what());
          }
    }

    // cofnelismy sie


    command.linear.x = 0;
    robot_state.data = 0;
    cmd_vel_publisher.publish(command);
    robot_state_publisher.publish(robot_state);

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle n;

    ros::Subscriber walls_subscriber = n.subscribe("walls_all", 1, wallsCallback);
    ros::Subscriber walls_laser_subscriber = n.subscribe("walls_scan", 1, scanCallback);
    ros::Subscriber ball_position = n.subscribe("ball_depth", 1, ballCallback);
    ros::Subscriber search_for_balls = n.subscribe("no_balls", 1, searchCallback);

    cmd_vel_publisher = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
    hoover_state_pub = n.advertise<std_msgs::Int16> ("/hoover_state",1);
    wall_need_publisher = n.advertise<std_msgs::Bool>("need_walls",1);

    tf_listener = new (tf::TransformListener);

    robot_state_publisher = n.advertise<std_msgs::Bool>("got_there", 1);
    client = n.serviceClient<walls_detection::CheckWalls>("need_walls");

    std_msgs::Int16 state;
    state.data=0;
    hoover_state_pub.publish(state);
    ros::spin();
}
