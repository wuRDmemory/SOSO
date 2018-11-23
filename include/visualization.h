#pragma once

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <robot.h>

void register_pub(ros::NodeHandle &n);
void pub_odometry(Robot& real, Robot& curr, const std_msgs::Header &header);
void pub_keyposes(Robot& real, Robot& curr, const std_msgs::Header &header);
void pub_TF(Robot &curr, const std_msgs::Header &header);
