#include "visualization.h"

using namespace ros;
using namespace Eigen;
ros::Publisher pubOdometry;
ros::Publisher pubPath, pubRealPath;
ros::Publisher pubRealKeyPose, pubKeyPose;

nav_msgs::Path path, real_path;
visualization_msgs::Marker real_key_poses, key_poses;

void register_pub(ros::NodeHandle &n) {
    pubPath = n.advertise<nav_msgs::Path>("path", 1000);
    pubRealPath = n.advertise<nav_msgs::Path>("read_path", 1000);
    pubOdometry = n.advertise<nav_msgs::Odometry>("odometry", 1000);
    pubRealKeyPose = n.advertise<visualization_msgs::Marker>("real_key_poses", 1000);
    pubKeyPose = n.advertise<visualization_msgs::Marker>("key_poses", 1000);

}

void pub_odometry(Robot& real, Robot& curr, const std_msgs::Header &header) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header = header;
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.pose.position.x = curr.mX;
    pose_stamped.pose.position.y = curr.mY;
    pose_stamped.pose.position.z = 0;
    Eigen::Quaterniond curr_Q = Quaterniond(AngleAxisd((double)real.mTheta, Vector3d::UnitZ()));
    pose_stamped.pose.orientation.x = curr_Q.x();
    pose_stamped.pose.orientation.y = curr_Q.y();
    pose_stamped.pose.orientation.z = curr_Q.z();
    pose_stamped.pose.orientation.w = curr_Q.w();
    path.header = header;
    path.header.frame_id = "base_link";
    path.poses.push_back(pose_stamped);
    pubPath.publish(path);

    pose_stamped.header = header;
    pose_stamped.header.frame_id = "base_link";
    pose_stamped.pose.position.x = real.mX;
    pose_stamped.pose.position.y = real.mY;
    pose_stamped.pose.position.z = 0;
    Eigen::Quaterniond real_Q = Quaterniond(AngleAxisd((double)real.mTheta, Vector3d::UnitZ()));
    pose_stamped.pose.orientation.x = real_Q.x();
    pose_stamped.pose.orientation.y = real_Q.y();
    pose_stamped.pose.orientation.z = real_Q.z();
    pose_stamped.pose.orientation.w = real_Q.w();
    real_path.header = header;
    real_path.header.frame_id = "base_link";
    real_path.poses.push_back(pose_stamped);
    pubRealPath.publish(real_path);
}

void pub_keyposes(Robot& real, Robot& curr, const std_msgs::Header &header)
{
    // real key pose
    real_key_poses.header = header;
    real_key_poses.header.frame_id = "base_link";
    real_key_poses.ns = "real_key_poses";
    real_key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    real_key_poses.action = visualization_msgs::Marker::ADD;
    real_key_poses.pose.orientation.w = 1.0;
    real_key_poses.lifetime = ros::Duration();
    //static int key_poses_id = 0;
    real_key_poses.id = 0; //key_poses_id++;
    real_key_poses.scale.x = 0.03;
    real_key_poses.scale.y = 0.03;
    real_key_poses.scale.z = 0.03;
    real_key_poses.color.g = 1.0;
    real_key_poses.color.a = 0.1;

    geometry_msgs::Point real_pose_marker;
    real_pose_marker.x = real.mX;
    real_pose_marker.y = real.mY;
    real_pose_marker.z = 0;
    real_key_poses.points.push_back(real_pose_marker);

    pubRealKeyPose.publish(real_key_poses);

    // key pose
    key_poses.header = header;
    key_poses.header.frame_id = "base_link";
    key_poses.ns = "key_poses";
    key_poses.type = visualization_msgs::Marker::SPHERE_LIST;
    key_poses.action = visualization_msgs::Marker::ADD;
    key_poses.pose.orientation.w = 1.0;
    key_poses.lifetime = ros::Duration();

    //static int key_poses_id = 0;
    key_poses.id = 0; //key_poses_id++;
    key_poses.scale.x = 0.02;
    key_poses.scale.y = 0.02;
    key_poses.scale.z = 0.02;
    key_poses.color.r = 1.0;
    key_poses.color.a = 1.0;

    geometry_msgs::Point pose_marker;
    pose_marker.x = curr.mX;
    pose_marker.y = curr.mY;
    pose_marker.z = 0;
    key_poses.points.push_back(pose_marker);

    pubKeyPose.publish(key_poses);
}

void pub_TF(Robot &curr, const std_msgs::Header &header)
{
    static double last_mx = 0, last_my=0;
    static tf::TransformBroadcaster br;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(curr.mTheta);
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = header;
    odom_trans.header.frame_id = "car_link";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = curr.mX;
    odom_trans.transform.translation.y = curr.mY;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    br.sendTransform(odom_trans);

    nav_msgs::Odometry odometry;
    odometry.header = header;
    odometry.header.frame_id = "car_link";
    odometry.child_frame_id = "base_link";

//    geometry_msgs::Quaternion tmp_Q = tf::createQuaternionFromYaw(real.mTheta);
    odometry.pose.pose.position.x = curr.mX;
    odometry.pose.pose.position.y = curr.mY;
    odometry.pose.pose.position.z = 0;
    odometry.pose.pose.orientation.x = odom_quat.x;
    odometry.pose.pose.orientation.y = odom_quat.y;
    odometry.pose.pose.orientation.z = odom_quat.z;
    odometry.pose.pose.orientation.w = odom_quat.w;
    odometry.twist.twist.linear.x = curr.mX - last_mx;
    odometry.twist.twist.linear.y = curr.mY - last_my;
    odometry.twist.twist.linear.z = 0;
    pubOdometry.publish(odometry);

    last_mx = curr.mX;
    last_my = curr.mY;
}
