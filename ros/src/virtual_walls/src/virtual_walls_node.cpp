#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

#include "ros/ros.h"
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "geometry_msgs/Point.h"

#include <eigen3/Eigen/Dense>

using namespace std;
using namespace Eigen;



class VirtualWalls
{
public:
  VirtualWalls();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

  sensor_msgs::LaserScan virtualScan;

private:
  ros::NodeHandle n;
  ros::Publisher virtualLaserpub;
  ros::Subscriber laserSub;

  tf::TransformListener* listner;


};


VirtualWalls::VirtualWalls()
:listner(new tf::TransformListener)
{

  virtualLaserpub = n.advertise<sensor_msgs::LaserScan>("/virtualscan", 1,false);
  laserSub = n.subscribe("/lidarscan", 1, &VirtualWalls::scanCallback, this);
}



void VirtualWalls::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  tf::StampedTransform transform;
  bool canTransform = false;

  try
  {
      canTransform = listner->waitForTransform("/odom", scan->header.frame_id, scan->header.stamp, ros::Duration(10.0));
      if(canTransform)
        listner->lookupTransform("/odom", scan->header.frame_id,  scan->header.stamp, transform);
  }
  catch (tf::TransformException& ex)
  {
      ROS_ERROR("Received an exception trying to transform a point from \"rplidar\" to \"odom\": %s", ex.what());
  }

  virtualScan.header.frame_id = scan->header.frame_id;
  virtualScan.header.stamp = scan->header.stamp;
  virtualScan.angle_min = scan->angle_min;
  virtualScan.angle_max = scan->angle_max;
  virtualScan.angle_increment = scan->angle_increment;
  virtualScan.scan_time = scan->scan_time;
  virtualScan.range_min = scan->range_min;
  virtualScan.range_max = scan->range_max;
  virtualScan.ranges = scan->ranges;
  virtualScan.intensities = scan->intensities;

  geometry_msgs::PointStamped in, out;
  float angle;

  for(int i = 0; i < scan->ranges.size(); i++)
  {
    if(std::isnan( scan->ranges[i] ))
    {
      // virtualScan.ranges[i] = virtualScan.ranges[max(0,i-1)];
      continue;
    }
    else if(std::isinf( scan->ranges[i]))
    {
      virtualScan.ranges[i] = 10;
    }
    angle = virtualScan.angle_min + i * virtualScan.angle_increment;
    in.point.x = virtualScan.ranges[i] * cos( angle );
    in.point.y = virtualScan.ranges[i] * sin( angle );
    in.header.frame_id = virtualScan.header.frame_id;
    in.header.stamp = virtualScan.header.stamp;

    if(canTransform)
      listner->transformPoint("/odom", in, out);

    Vector2f p;
    p << out.point.x, out.point.y;
    Vector2f r;
    r << transform.getOrigin().x(), transform.getOrigin().y();

    float wallRight = -2.6;
    float wallLeft = 0.5;
    float wallFront = 3.3;
    float wallBack = -0.5;

    Vector2f m = p-r;
    m = m.normalized();


    if(out.point.x < wallBack && out.point.y < wallRight)
    {
      float range1 = abs((wallRight - r[1])/m[1]);
      float range2 = abs((wallBack - r[0])/m[0]);
      virtualScan.ranges[i] = min(range1, range2);
    }
    else if(out.point.x < wallBack && out.point.y > wallLeft)
    {
      float range1 = abs((wallLeft - r[1])/m[1]);
      float range2 = abs((wallBack - r[0])/m[0]);
      virtualScan.ranges[i] = min(range1, range2);
    }
    else if(out.point.x > wallFront && out.point.y < wallRight)
    {
      float range1 = abs((wallFront - r[0])/m[0]);
      float range2 = abs((wallRight - r[1])/m[1]);
      virtualScan.ranges[i] = min(range1, range2);
    }
    else if(out.point.x > wallFront && out.point.y > wallLeft)
    {
      float range1 = abs((wallFront - r[0])/m[0]);
      float range2 = abs((wallLeft - r[1])/m[1]);
      virtualScan.ranges[i] = min(range1, range2);
    }
    else if(out.point.x > wallFront /*&& out.point.y > wallRight && out.point.y < wallLeft*/)
    {
      float new_range = abs((wallFront - r[0])/m[0]);
      virtualScan.ranges[i] = new_range;
    }
    else if(out.point.x < wallBack /*&& out.point.y > wallRight && out.point.y < wallLeft*/)
    {
      float new_range = abs((wallBack - r[0])/m[0]);
      virtualScan.ranges[i] = new_range;
    }

    else if(out.point.y <= wallRight /*&& out.point.x > wallBack && out.point.x < wallFront*/)
    {
      float new_range = abs((wallRight - r[1])/m[1]);
      virtualScan.ranges[i] = new_range;
    }
    else if(out.point.y >= wallLeft /*&& out.point.x > wallBack && out.point.x < wallFront*/)
    {
      float new_range = abs((wallLeft - r[1])/m[1]);
      virtualScan.ranges[i] = new_range;
    }
    //
    // if(i == 5)
    // {
    //   ROS_INFO_STREAM("angle = " << angle);
    //   ROS_INFO_STREAM("Range: " << scan->ranges[i] << "  New Range: " << virtualScan.ranges[i]);
    //   // ROS_INFO_STREAM("x: " << in.point.x << "  y: "<< in.point.y );
    //   // ROS_INFO_STREAM("x: " << out.point.x << "  y: "<< out.point.y );
    // }

  }
  virtualLaserpub.publish(virtualScan);
}



 int main(int argc, char **argv)
 {
   ros::init(argc, argv, "virtualWalls");
   VirtualWalls virtualWalls;

   ROS_INFO_STREAM("Starting Virtual Walls Generation...");

   ros::Rate rate(10);
    while (ros::ok())
    {
      ros::spinOnce();
      rate.sleep();
    }

   return 0;
 }
