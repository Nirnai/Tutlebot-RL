/* keyboard_node.cpp
 *
 * authors: Christian Gajek
 *
 */


#include "ros/ros.h"
#include "std_msgs/Char.h"
#include <iostream>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_node");
  ros::NodeHandle nh;

  ros::Publisher pub;
  pub = nh.advertise<std_msgs::Char>("key", 10);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    std_msgs::Char msg;
    char ch = getchar();

    if(ch != '\n')    // clear buffer '\n'
      getchar();

    msg.data = ch;
    pub.publish(msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}