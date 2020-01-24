#include <tf/tf.h>
#include <std_srvs/Empty.h> 
#include <std_msgs/Empty.h> 
#include <cmath>
#include <stdio.h>

#include "turtlebot_dqn/turtlebot_lidar_env.h"
#include "turtlebot_dqn/utilities.h"

const float ALPHA = 0.001;                    // exp() moving average for LIDAR
const float RAD2DEG = 57.2957795131;
const float VELOCITY_INC = 0.02;             // velocity increment


TurtlebotLidarEnv::TurtlebotLidarEnv()
    : AgentEnvironment(3, 22), is_scan_updated(false), is_pose_updated(false), is_goal_set(false),
      prev_action(0.0), distance(-1.0), move_forward_flag(false)
{
    // Publisher
    vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    reset_odom_pub = nh.advertise<std_msgs::Empty>("/mobile_base/commands/reset_odometry", 10);

    // Subscriber
    scan_sub = nh.subscribe("/virtualscan", 10, &TurtlebotLidarEnv::scan_callback, this);
    goal_sub = nh.subscribe("/move_base_simple/goal", 10, &TurtlebotLidarEnv::goal_callback, this);

    // TF
    fixed_frame = "/odom";
    robot_frame = "/base_link";

    // setup hyper parameters
    min_goal_dist = 0.25;
    min_obstical_dist = 0.17;
    
    // lidar parameters
    ray_num = 18;                       // 18 rays
    visual_angle = 3 * M_PI / 4.0f;     
    max_range = 2.0f;                   
   
    // lidar ray visualization
    lidar_rays = create_visualization_marker(visualization_msgs::Marker::LINE_LIST, 0, 1.0, 0.0, 0.0, 0.01);
    lidar_rays.points.resize(2*ray_num);
    filtered_ranges.resize(ray_num, 0);

    goal.x = 0;
    goal.y = 0;
    goal.theta = 0;
    hit_obstacle = false; reached_goal = false;

    hw_cmd_vel.linear.x = des_cmd_vel.linear.x = 0;
    hw_cmd_vel.linear.y = des_cmd_vel.linear.y = 0;
    hw_cmd_vel.angular.z = des_cmd_vel.angular.z = 0;

    // environment setup
    setup();
}

TurtlebotLidarEnv::~TurtlebotLidarEnv() {

}

void TurtlebotLidarEnv::setup() {
    // number of laser rays

    float delta_phi = M_PI / 180.0f;
    int idx = visual_angle / delta_phi;

    float delta_phi_discret = 2 * visual_angle / (ray_num - 1);
    int inc = delta_phi_discret / delta_phi;

    raw_ranges.resize(360, 0.0f);
    ranges.resize(ray_num, 0.0f);

    for(int i = -idx + inc/2.0f; i < idx; i += inc) {
        if(i < 0)
            indicies.push_back(360 + i);
        else
            indicies.push_back(i);
        //ROS_INFO_STREAM("[" << i << "]= " << indicies.back());
    }


}

/* create a visualization marker */
visualization_msgs::Marker TurtlebotLidarEnv::create_visualization_marker(int type, int id, double r, double g, double b, double scale)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = robot_frame;
    marker.header.stamp = ros::Time();
    marker.ns = "turtlebot_qlearn_node";
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;

    return marker;
}

/* set desired action command */
void TurtlebotLidarEnv::execute(action_type action)
{
    if(!is_goal_set)
        return;  // not available

    // 1. remember last distance
    prev_action = action;
    
    // 2. assemble action (set desired /des_cmd_vel)
    switch(action) {
    case 0:
        des_cmd_vel.linear.x = 0.25;
        des_cmd_vel.angular.z = 0.0;
        break;
    case 1:
        des_cmd_vel.linear.x = 0.09f;
        des_cmd_vel.angular.z = 0.8;
        break;
    case 2:
        des_cmd_vel.linear.x = 0.09f;
        des_cmd_vel.angular.z = -0.8;
        break;
    }
}


/* match desired action to hardware action (smoothing) */
void TurtlebotLidarEnv::process_action()
{
    if(!is_goal_set)
        return;  // not available

    // set angular velocity directly
    hw_cmd_vel.angular.z = des_cmd_vel.angular.z;

    /* assert smooth start and stop of robot*/
    double diff = des_cmd_vel.linear.x - hw_cmd_vel.linear.x;

    if(diff > 0) {      // we need to accelerate
        if(diff > VELOCITY_INC)
            hw_cmd_vel.linear.x += VELOCITY_INC;
        else
            hw_cmd_vel.linear.x = des_cmd_vel.linear.x;
    } else if (diff < 0) {
        if(diff < -VELOCITY_INC)
            hw_cmd_vel.linear.x -= VELOCITY_INC;
        else
            hw_cmd_vel.linear.x = des_cmd_vel.linear.x;
    }
    // 3. publish "/cmd_vel"
    vel_pub.publish(hw_cmd_vel);
}

/* do observation */
bool TurtlebotLidarEnv::observe(EnvState& env_state)
{   
    // 1. check if new observation available
    if(!is_scan_updated)
        return false;  // not available

    is_scan_updated = false;
    is_pose_updated = false;

    // 2. get position relative to goal
    reached_goal = update_pose();

    // 3. reduce laser ranges to ray_num
    hit_obstacle = reduce_ranges(raw_ranges, ranges, max_range);

    return true;
}

void TurtlebotLidarEnv::reset()
{
    // generate an inital observation ( wait for  topic as inital observation )
    const sensor_msgs::LaserScan::ConstPtr& scan = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/virtualscan", nh, ros::Duration(1.0));
    raw_ranges = scan->ranges;
    update_pose();

    is_scan_updated = true;
    is_pose_updated = true;
    is_goal_set = false; 
}

bool TurtlebotLidarEnv::reduce_ranges(const std::vector<float>& raw_ranges, std::vector<float>& ranges, float max_range)
{
    bool done = false;
    int inf_cnt;

    float min_ray;

    // points for ray visualization
    geometry_msgs::Point p0, p1;
    p0.x = p0.y = 0;
    p0.z = p1.z = 0.24;

    for(int k = 0; k < indicies.size(); ++k) {
        // mean range
        float r = 0.0f;
        min_ray = INFINITY;
        inf_cnt = 0;

        // search for minimum laser distance in a range of +- 2° to indicies[k]
        for(int j = -2; j <= 2; ++j ) {              
            int idx = (360 + indicies[k] + j - 4) % 360;
            // don't consider nan and inf laser rays
            if(!std::isinf(raw_ranges[idx] && !std::isnan(raw_ranges[idx])))
                if(raw_ranges[idx] < min_ray)
                    min_ray = raw_ranges[idx];
        }

        ranges[k] = std::min(min_ray, max_range);

        r = filtered_ranges[k] = ALPHA * filtered_ranges[k] + (1 - ALPHA) * ranges[k] ;    
        ranges[k] = filtered_ranges[k];

        // check if obsticle was hit
        if( ranges[k] < min_obstical_dist)
            done = true;

        // lidar ray visualization
        float ray_angle = (-visual_angle + 2 * k * visual_angle / (ray_num-1.0));
        p1.x = cos(ray_angle) * r;
        p1.y = sin(ray_angle) * r;
        lidar_rays.points[2*k] = p0;                // start point of ray
        lidar_rays.points[2*k+1] = p1;              // end point of ray
    }

    // publish rays 
    marker_pub.publish(lidar_rays);
    return done;
}

bool TurtlebotLidarEnv::update_pose()
{
    bool done = false;
    tf::StampedTransform transform;
    // 1. extract position and orientation of robot_frame wrt fixed_frame
    listener.waitForTransform(fixed_frame, robot_frame, ros::Time(0), ros::Duration(3.0));
    try{
      listener.lookupTransform(fixed_frame, robot_frame, ros::Time(0), transform);      
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }
    is_pose_updated = true;

    double roll, pitch, yaw;
    transform.getBasis().getRPY(roll, pitch, yaw);

    // 2. compute distance to goal
    geometry_msgs::Pose2D pos;
    pos.x = transform.getOrigin().x();
    pos.y = transform.getOrigin().y();
    distance = geom2d::distance_point(pos, goal);

    // 3. compute angle to goal
    theta = rotation_angle(atan2(goal.y - pos.y, goal.x - pos.x), yaw );

    // 4. goal reached
    if( distance <  min_goal_dist)
        done = true;

    return done;
}

float TurtlebotLidarEnv::rotation_angle(float src, float dst)
{
    float val = (src > dst) ? -M_PI + std::fmod(src - dst + M_PI, M_PI * 2) 
                             :  M_PI - std::fmod(dst - src + M_PI, M_PI * 2);
    return  val;
}



// ---------
// Callbacks
// ---------

void TurtlebotLidarEnv::scan_callback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    raw_ranges = scan->ranges;
    is_scan_updated = true;
}

void TurtlebotLidarEnv::goal_callback(const geometry_msgs::PoseStamped::ConstPtr& posestamped)
{
    goal.x = posestamped->pose.position.x;
    goal.y = posestamped->pose.position.y;
    ROS_INFO_STREAM("New Goal set to: x=" <<  goal.x << " y=" << goal.y);
    is_goal_set = true;
}