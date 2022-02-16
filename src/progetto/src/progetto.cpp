
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "geometry_utils_fd.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <sstream>
ros::Publisher vel_pub;
bool vel_mod=false;
float target_x=0;
using namespace Eigen;

void LaserCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in){
 //devo estrarre i punti dal Laser_scan
   laser_geometry::LaserProjection projector_;
   sensor_msgs::PointCloud cloud;
   tf::TransformListener listener_;
   tf::StampedTransform transform;
try
     {
         projector_.transformLaserScanToPointCloud(
            "base_laser_link",*scan_in, cloud,listener_);
      }
     catch (tf::TransformException& e)
      {
          ROS_ERROR("%s",e.what());
          return;
     }
       Eigen::Isometry2f transform_laser = convertPose2D(transform);
  // Extract points from raw laser scan and paint them on canvas
       Eigen::Vector2f p;
       for(auto& point :cloud.points){
         p(0)=point.x;
         p(1)=point.y;
         
         p=transform_laser*p; //ottengo il punto trasformato
       }
      
        geometry_msgs::Twist send;

 
     
     
      
          
      vel_pub.publish(send);
}
int main(int argc, char **argv){ //subscriber
   if(argc<2){
       ROS_INFO("Errore:  numero di parametri errato");
       return -1;
   }  
     ROS_INFO("Insert the parameters: 1. Topic for scan 2. Topic per cmd_vel");
   ros::init(argc,argv,"progetto");

   ros::NodeHandle n; //initialize a node
    ros::Rate loop_rate(10);
  
   ros::Subscriber base_sub=n.subscribe(argv[1],1000,LaserCallBack);// subscriber to receive the laser scan commands
   ROS_INFO("Subscriber started to %s",argv[1]);
   
   ROS_INFO("Inviare al topic /cmd_vel_mod il comando per far muovere il robot");
   ros::spin();
   
  return 0;
  ROS_INFO("aho");

}
