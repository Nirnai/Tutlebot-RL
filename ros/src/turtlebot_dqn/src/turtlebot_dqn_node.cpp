#include "turtlebot_dqn/turtlebot_lidar_env.h"
#include "turtlebot_dqn/dqnlearner.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "turtlebot_dqn_node");

  ROS_INFO_STREAM("Starting turtlebot_dqn_node... \n");
  TurtlebotLidarDQNEnv env;
  ros::Rate loop_rate(50);

  DQNLearner learner(&env, "./log/");   // log directory
  learner.start(500, 10000);
  ROS_INFO_STREAM("DQN initialized!");

  while(ros::ok())
  {
    if(learner.update() > 0) {
      ROS_INFO_STREAM("main: Finished");
      break; // finished
    }

    // update action in environment
    env.process_action();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}