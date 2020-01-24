#ifndef TURTLEBOTLIDARENV_H_
#define TURTLEBOTLIDARENV_H_

#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h> 

#include "turtlebot_dqn/agent_environment.h"

/*
 * Base classe for Turtlebot environment
 */

class TurtlebotLidarEnv : public AgentEnvironment
{
public:
    TurtlebotLidarEnv();
    virtual ~TurtlebotLidarEnv();

    /* perform one step (action) -> learner */
    virtual void execute(action_type action);

    /* hw velocity control of turtlebot */
    virtual void process_action();

    /* returns the new observations */
    virtual bool observe(EnvState& env_state);

    /* reset environment and return to inital observation */
    virtual void reset();

    /* update current pose form tf */
    bool update_pose();

protected:
    bool hit_obstacle, reached_goal;       // controll flags
    float distance;                        // robot world position
    float theta;                           // robot world angle
    std::vector<float> ranges;             // reduced laser range data
    bool is_goal_set;                       // true if goal recived
    
    float min_goal_dist;                    // min distance to goal     (stop simulation)
    float min_obstical_dist;                // min distance to obsticle (stop simulation)

    action_type prev_action;                

private:
    ros::NodeHandle nh;

    // Publisher
    ros::Publisher vel_pub;
    ros::Publisher marker_pub;
    ros::Publisher reset_odom_pub;

    // Subscriber
    ros::Subscriber scan_sub;
    ros::Subscriber goal_sub;

    // TFs
    tf::TransformListener listener;
    std::string fixed_frame;                // fixed reference frame
    std::string robot_frame;                // frame attached to roboto

    tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion qtf;

    // lidar ray visualization
    visualization_msgs::Marker lidar_rays;  // ray visualization
    visualization_msgs::Marker create_visualization_marker(int type, int id, double r, double g, double b, double scale);

    bool is_scan_updated;                   // true if laser scan updated
    bool is_pose_updated;                   // true if pose updated

    geometry_msgs::Twist des_cmd_vel;       // desired action
    geometry_msgs::Twist hw_cmd_vel;        // cmd_vel on hardware
    bool move_forward_flag;                 // flag for driving forward

    geometry_msgs::Pose2D goal;             // robot goal
    
    int ray_num;                            // number of rays for state
    float visual_angle;
    float max_range;
    std::vector<int> indicies; 

    std::vector<float> raw_ranges;          // raw laser range data  
    std::vector<float> filtered_ranges;     // lowpass filtered ranges      

    /* setup variables */
    void setup();

    bool reduce_ranges(const std::vector<float>& raw_ranges, std::vector<float>& ranges, float max_range);

    /* signed rotation angle form src to dst */
    static float rotation_angle(float src, float dst);

    /* lidar callbacks */
    void scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan);
    /* new gaol set */
    void goal_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
};

#endif