#include "ros/ros.h"
#include "user_interface/user_interface_node.h"
#include <string>

using namespace std;

UserInterface::UserInterface(ros::NodeHandle &nh) : nh(nh), state(RESET), is_goal_set(false), interrupt(false)
{
    // Subscriber
    key_sub = nh.subscribe("key", 10, &UserInterface::keyboardCallback, this); 
    goal_sub = nh.subscribe("/move_base_simple/goal", 10, &UserInterface::goal_callback, this);

    // Publisher
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
    reset_odom_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 10);

    // init prev_goal
    reset_goal();

    // init TFs
    fixed_frame = "/odom";
    robot_frame = "/base_link";

}


void UserInterface::reset_goal() {
    prev_goal.x = 0;
    prev_goal.y = 0;
    prev_goal.theta = 0;

    prev_goal_pos.pose.position.x = 0;
    prev_goal_pos.pose.position.y = 0;
    prev_goal_pos.pose.orientation.z = 0.0;

    is_goal_set = false;
}

/*
 * process_state()
 *
 * processes state transitions depending on the keyboard input topic /key
 *
 */

void UserInterface::process_state(char ch)
{
  // ESC Command
  // if ESC is pressed, always goes to the state RESET
  if(ch==ESC) {
    state = RESET;
    ROS_INFO_STREAM("RESET ");
    interrupt = false;
    reset_goal();
    reset_odometry();
    return;
  }

  // other state transitions for arbitary /key
  switch(state) {

    case RESET: {
        state = RUN;
        ROS_INFO_STREAM("RUN ");
      break;
    }

    case RUN: {
        state = INTERRUPT;
        // force_stop here!
        interrupt = true;
        force_stop();

        ROS_INFO_STREAM("INTERRUPT ");
      break;
    }

    case INTERRUPT:
    {
        state = RUN;
        // publish_goal
        publish_goal();
        ROS_INFO_STREAM("RUN ");
      break;
    }

    default:
      break;
  }

}

// All actions run here!
void UserInterface::process_action() 
{
  switch(state) {

    case RUN: {
        interrupt = false;
        broadcast_goal_tf();
      break;
    }

    default:
      break;
  }
}

// force_stop()
//
// send navigation_goal to base_link => force a stop
//
void UserInterface::force_stop() {

    if(is_goal_set) {
        geometry_msgs::PoseStamped posestamped;
        bool success = false;
        // get roboto pose in odom
        tf::StampedTransform transform;
        listener.waitForTransform(fixed_frame, robot_frame, ros::Time(0), ros::Duration(3.0));
        try{
        listener.lookupTransform(fixed_frame, robot_frame, ros::Time(0), transform);      
        success = true;
        }
        catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        }

        // assemble posestamped msg
        if(success) {
            posestamped.pose.position.x = transform.getOrigin().x();
            posestamped.pose.position.y = transform.getOrigin().y();
            posestamped.pose.orientation.z = 0.0;
            goal_pub.publish(posestamped);
        }
    }
}

// publish_goal()
//
// publishes previous goal (re-run)
//
 void UserInterface::publish_goal() {
    goal_pub.publish(prev_goal_pos);
 }


void UserInterface::broadcast_goal_tf()
{
    tf::Transform transform;
    tf::Quaternion qtf;
    // broadcast goal tf
    if(is_goal_set) {   
        qtf.setRPY(0.0, 0.0, 0.0);
        transform.setOrigin(tf::Vector3(prev_goal.x, prev_goal.y, 0.0));
        transform.setRotation(qtf);
        br.sendTransform(tf::StampedTransform(transform ,ros::Time::now(), fixed_frame, "/goal"));
    }
}


void UserInterface::reset_odometry() {
    ROS_INFO_STREAM("Reset odometry!");
    std_msgs::Empty msg;
    reset_odom_pub.publish(msg);
}


// ---------
// CALLBACKS
// ---------

void UserInterface::keyboardCallback(const std_msgs::Char msg) {

  process_state(msg.data);
}

// set_navigation_goal callback
void UserInterface::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& posestamped)
{
    if(!interrupt) {
        prev_goal_pos = *posestamped;
        prev_goal.x = posestamped->pose.position.x;
        prev_goal.y =posestamped->pose.position.y;
        ROS_INFO_STREAM("New Goal set to: x=" <<  prev_goal.x << " y=" << prev_goal.y);
        is_goal_set = true;
    }
}



// ----
// Main
// ----

int main(int argc, char** argv)
{
  ros::init(argc, argv, "user_interface_node");

  ROS_INFO_STREAM("Starting user_interface_node... \n");

  ros::NodeHandle nh;
  ros::Rate loop_rate(10);
  // create UserInterface
  UserInterface interface(nh);
  
  while(ros::ok())
  {
    interface.process_action();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}