#ifndef USER_INTERFACE_H
#define USER_INTERFACE_H

#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <std_msgs/Empty.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


#define ESC 27

using namespace std;

class UserInterface
{
public:
  UserInterface(ros::NodeHandle &nh);

  void process_action();

private:

  ros::NodeHandle &nh;

  // States of  user_interface
  enum State {RESET, RUN, INTERRUPT};
  State state;
  
  // goal variables
  geometry_msgs::Pose2D prev_goal;
  geometry_msgs::PoseStamped prev_goal_pos;
  bool is_goal_set;
  bool interrupt;
  
  void reset_goal();

  void process_state(char ch);

  void broadcast_goal_tf();
  void force_stop();
  void publish_goal();
  void reset_odometry();

  // TFs
  tf::TransformListener listener;
  std::string fixed_frame;                  // fixed reference frame
  std::string robot_frame;                  // frame attached to robot

  // TF Broadcaster
  tf::TransformBroadcaster br;

  // SUBSCRIBER
  ros::Subscriber key_sub;                  // keyboard input
  ros::Subscriber goal_sub;                 // navigation goal

  // PUBLISHER
  ros::Publisher goal_pub;                  // navigation goal
  ros::Publisher reset_odom_pub;            // reset odometry frame

  // CALLBACKS
  void keyboardCallback(const std_msgs::Char msg);
  void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

};

#endif // USER_INTERFACE_H