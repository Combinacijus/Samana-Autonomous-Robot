#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include <iostream>
#include <stdio.h>

using namespace std;

class FloatDebugger
{
  public:
    void FloatDebuger()
    {
      printf("sasdads"); 
      cout << "asas" << endl;
    }

    void loop()
    {
      cout << "loopaa" << endl;
    }
};

int main(int argc, char **argv)
{
  cout << "Start" << endl;
  ros::init(argc, argv, "mapviz_pub_floats_cpp");
  ros::NodeHandle n;
  ROS_DEBUG("Hello %s", "World");
  ROS_INFO("Hello %s", "World");
  ros::Publisher pub_yaw_imu = n.advertise<std_msgs::Float32>("debug/yaw_imu", 1);

  FloatDebugger fd;

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    fd.loop(); 
    std_msgs::Float32 msg_imu_yaw;
    msg_imu_yaw.data = 10;
    
    // std_msgs::String msg;

    // std::stringstream ss;
    // ss << "hello world " << count;
    // msg.data = ss.str();

    ROS_DEBUG("Hello %s", "World");
    ROS_INFO("Hello %s", "World");
    ROS_INFO("Num %f", msg_imu_yaw.data);

    // chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}