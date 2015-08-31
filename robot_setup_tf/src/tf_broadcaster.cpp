#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster broadcaster;
  tf::TransformBroadcaster broadcaster2;

  while(n.ok()){
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0.26, 0, 1), tf::Vector3(0.05, 0.0, 0.55)), ros::Time::now(),"base_link", "camera_link"));
    broadcaster2.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.35, 0, 0)), ros::Time::now(),"base_link", "pipe_link"));
    r.sleep();
  }
}
