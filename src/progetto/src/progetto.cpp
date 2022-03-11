
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <sstream>
#include <vector>
#include <iostream>
#include <fstream>
#include <memory>
#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "Eigen/Cholesky"
#include "rotations.h"
#include "tf/tf.h"
#include "tf2_msgs/TFMessage.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2_ros/transform_broadcaster.h>
using namespace std;
using namespace Eigen;
using ContainerType=ICP::ContainerType;

ros::Publisher pose_pub;
tf2_ros::Buffer tfBuffer;
int count_msg=0;
std::unique_ptr<ICP> laser_matcher;
int draw; //parameter if you want to draw points for gnuplot, use as debugger
int tf_send; //parameter if you want to send tf computed and not only 2dpose
float sample_num=2;//sample interval
//Calculates the transform
const Eigen::Isometry2f getTransform(const std::string& from, const std::string& to) {
  Eigen::Isometry2f MTB = Eigen::Isometry2f::Identity();
  if(tfBuffer.canTransform(from, to, ros::Time(0))){
    geometry_msgs::TransformStamped transformStamped=tfBuffer.lookupTransform(from, to, ros::Time(0));
    tf2::Quaternion q;
    tf2::convert(transformStamped.transform.rotation , q);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // cerr <<"rpy "<< roll <<" "<< pitch <<" "<< yaw << endl;
    auto tr = transformStamped.transform.translation;
    //take the translation and rotation 
    MTB.linear()=Rtheta(yaw); 
    MTB.translation()=Vector2f(tr.x, tr.y);
  }
  else{
    std::cerr << "cannot transform correctly" << endl;
  }
  cerr << from << "->" << to << endl;
  cerr << MTB.matrix() << endl;
  return MTB;
  }
void LaserCallBack(const sensor_msgs::LaserScan& scan_in){
  
 //extract the points from the laser scan
  //laser_geometry::LaserProjection projector_;
  // sensor_msgs::PointCloud cloud;
  /*tf::TransformListener listener_;
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
       cerr << iso.matrix() << endl;
  */
   float angle_min = scan_in.angle_min;       
  float angle_max = scan_in.angle_max; 
  float angle_increment = scan_in.angle_increment;
  int size = std::ceil((angle_max-angle_min)/angle_increment/sample_num); //size of the scan
  
  cerr << "count_msg" << count_msg << endl;
  
  if(count_msg==0){ // initial scan 
    Eigen::Isometry2f MTB=getTransform("map","base_link");
    Eigen::Isometry2f BTL=getTransform("base_link","base_laser_link");

    laser_matcher=std::unique_ptr<ICP>(new ICP(BTL,MTB*BTL,20,size,draw));//compute ICP to find the isometry

  }

   if(count_msg>1) {
      laser_matcher->updateOld();
   }
   int ok=count_msg!=0; //tells if it is more than the first scan
       
  float line;
  float angle=angle_min;
  int idx=0;
  for(int i=0; i<size; i+=1){
    line = scan_in.ranges[i*sample_num];
    angle += angle_increment*sample_num;
    idx++;
    float a = line*cos(angle);
    float b = line*sin(angle);
    laser_matcher->setVal(ok,idx, Eigen::Vector2f(a,b));
  }
  count_msg++;
  if(count_msg==1) return;
   cerr << "check seg" << endl;
   laser_matcher->run(10);
   cerr << "done check" << endl;
 
   laser_matcher->updateMTL(); //update the isometry

   auto mtb=laser_matcher->MTB(); //returns the pose of the base_link frame wrt map frame
   
   cerr << "Matrix MTB computed" << endl;
  cerr << mtb.matrix() << endl;
 
  //get traslation and rotation of the isometry
  geometry_msgs::Pose2D::Ptr pose_msg;
   pose_msg = boost::make_shared<geometry_msgs::Pose2D>();
  pose_msg->x = mtb.translation()(0);
  pose_msg->y = mtb.translation()(1);
  pose_msg->theta = Eigen::Rotation2Df(mtb.rotation()).angle();
  pose_pub.publish(pose_msg);
  ros::Rate r(10); // 10 hz
  if(tf_send){
    auto bto= getTransform("odom", "base_link").inverse();//from odom to base link
    //mto*otb=tmb => mto=mtb*otb^-1
    auto mto = mtb*bto;
    // publish odom->base_link
     static tf2_ros::TransformBroadcaster br;
     geometry_msgs::TransformStamped tf_msg;
       tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "/map";
        tf_msg.child_frame_id = "/odom";
        tf_msg.transform.translation.x = mto.translation()(0);
        tf_msg.transform.translation.y = mto.translation()(1);
        tf_msg.transform.translation.z = 0.0;
        tf2::Quaternion q;
        q.setRPY(0, 0,Eigen::Rotation2Df(mto.rotation()).angle());
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        br.sendTransform(tf_msg);
}

}
int main(int argc, char **argv){ 
     ROS_INFO("Laser matcher activation...");
   ros::init(argc,argv,"progetto");

   ros::NodeHandle n; //initialize a node
    ros::Rate loop_rate(10);
    n.getParam("draw",draw);
    n.getParam("tfsend",tf_send);
    pose_pub=n.advertise<geometry_msgs::Pose2D>("/pose2D", 1000);//publish the pose
    tf2_ros::TransformListener tfListener(tfBuffer);
   ros::Subscriber base_sub=n.subscribe("/base_scan",1000,LaserCallBack);// subscriber to receive the laser scan commands
   
   ROS_INFO("Subscriber started to %s",argv[1]);
   
   ROS_INFO("Make the robot move in order to receive laser scans");
   ros::spin();
   ros::shutdown();
  return 0;
}
