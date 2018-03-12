#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/String.h>
#include <Eigen/Core>

struct Position
{
  void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    //  ROS_INFO("I heard (%f, %f, %f)", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) ;
//      std::cout<<"timestamp:"<<msg->header.stamp<<std::endl;
      pos(0) = msg->pose.position.x;
      pos(1) = msg->pose.position.y;
      pos(2) = msg->pose.position.z;
      q = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll,pitch,yaw);
      //std::cout << "Roll: " << roll << ", Pitch: " << pitch << ", Yaw: " << yaw << std::endl;
  }
  Eigen::Vector3d pos;
  tf::Quaternion q;
};


// namespace tracker
// {
//   Eigen::Vector3d GetPosition()
//   {
//     ros::NodeHandle n;
//     Position pos;
//     ros::Subscriber sub = n.subscribe("Voltaire/pose", 1000, &Position::callback, &pos);
//     ros::spinOnce();
//     Eigen::Vector3d v = pos.pos;
//     return v;
//   }
// };
