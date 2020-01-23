#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void odomCb(const nav_msgs::Odometry::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_tf_pub");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("odom", 100, odomCb);
    ros::spin();
}

void odomCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    // ROS_INFO("Seq: [%d]", msg->header.seq);
	// ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
	// ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
	// ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);

    //first, we'll publish the transform over tf
    tf::TransformBroadcaster odom_broadcaster;
    geometry_msgs::TransformStamped odom_trans;

    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = msg->header.frame_id;
    odom_trans.child_frame_id = msg->child_frame_id;

    odom_trans.transform.translation.x = msg->pose.pose.position.x;
    odom_trans.transform.translation.y = msg->pose.pose.position.y;
    odom_trans.transform.translation.z = msg->pose.pose.position.z;
    odom_trans.transform.rotation.x = msg->pose.pose.orientation.x;
    odom_trans.transform.rotation.y = msg->pose.pose.orientation.y;
    odom_trans.transform.rotation.z = msg->pose.pose.orientation.z;
    odom_trans.transform.rotation.w = msg->pose.pose.orientation.w;

    // Send the transform
    odom_broadcaster.sendTransform(odom_trans);
    // ros::spinOnce();
    // ROS_INFO("TF ----");
}