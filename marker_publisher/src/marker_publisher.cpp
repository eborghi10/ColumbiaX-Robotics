#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "marker_publisher");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  while (ros::ok())
  {
    visualization_msgs::Marker object;
    object.header.frame_id = "/object_frame";
    object.header.stamp = ros::Time::now();
    object.ns = "project_2";
    object.id = 0;
    object.type = visualization_msgs::Marker::CYLINDER;
    object.action = visualization_msgs::Marker::ADD;
    object.pose.position.x = 0;
    object.pose.position.y = 0;
    object.pose.position.z = 0;
    object.pose.orientation.x = 0.0;
    object.pose.orientation.y = 0.0;
    object.pose.orientation.z = 0.0;
    object.pose.orientation.w = 1.0;
    object.scale.x = 0.3;
    object.scale.y = 0.3;
    object.scale.z = 0.3;
    object.color.r = 0.0f;
    object.color.g = 1.0f;
    object.color.b = 0.0f;
    object.color.a = 1.0;
    object.lifetime = ros::Duration();
    marker_pub.publish(object);

    ros::Duration(1).sleep();

    visualization_msgs::Marker robot;
    robot.header.frame_id = "/robot_frame";
    robot.header.stamp = ros::Time::now();
    robot.ns = "project_2";
    robot.id = 1;
    robot.type = visualization_msgs::Marker::CUBE;
    robot.action = visualization_msgs::Marker::ADD;
    robot.pose.position.x = 0;
    robot.pose.position.y = 0;
    robot.pose.position.z = 0;
    robot.pose.orientation.x = 0.0;
    robot.pose.orientation.y = 0.0;
    robot.pose.orientation.z = 0.0;
    robot.pose.orientation.w = 1.0;
    robot.scale.x = 0.3;
    robot.scale.y = 0.3;
    robot.scale.z = 0.3;
    robot.color.r = 0.0f;
    robot.color.g = 1.0f;
    robot.color.b = 0.0f;
    robot.color.a = 1.0;
    robot.lifetime = ros::Duration();
    marker_pub.publish(robot);

    ros::Duration(1).sleep();

    visualization_msgs::Marker camera;
    camera.header.frame_id = "/camera_frame";
    camera.header.stamp = ros::Time::now();
    camera.ns = "project_2";
    camera.id = 2;
    camera.type = visualization_msgs::Marker::ARROW;
    camera.action = visualization_msgs::Marker::ADD;
    camera.pose.position.x = 0;
    camera.pose.position.y = 0;
    camera.pose.position.z = 0;
    camera.pose.orientation.x = 0.0;
    camera.pose.orientation.y = 0.0;
    camera.pose.orientation.z = 0.0;
    camera.pose.orientation.w = 1.0;
    camera.scale.x = 0.3;
    camera.scale.y = 0.3;
    camera.scale.z = 0.3;
    camera.color.r = 0.0f;
    camera.color.g = 1.0f;
    camera.color.b = 0.0f;
    camera.color.a = 1.0;
    camera.lifetime = ros::Duration();
    marker_pub.publish(camera);

    ros::Duration(1).sleep();
  }
}
