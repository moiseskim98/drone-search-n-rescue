#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandCode.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/PositionTarget.h>
#include <time.h>
#include <cmath>
#include <math.h>
#include <ros/duration.h>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PointStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Int8.h>
using namespace std;
using namespace mavros_msgs;

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
geometry_msgs::PoseStamped pose_snapshot;
geometry_msgs::Point current_point;
geometry_msgs::Point current_line;
double confidence_circle=0;
double confidence_line=0;
bool point_avalible=false;
bool line_avalible=false;

float GYM_OFFSET;
tf2_ros::Buffer tfBuffer;
ros::Publisher set_gp_origin_pub;
ros::Publisher set_raw_pub;
ros::Publisher local_pos_pub;
ros::Publisher mode_pub;
/***************************************call backs********************************************/
//get state
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
  bool connected = current_state.connected;
  bool armed = current_state.armed;
}

//get current position of drone
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = *msg;
//  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  tf2::Quaternion q(
  current_pose.pose.orientation.x,
  current_pose.pose.orientation.y,
  current_pose.pose.orientation.z,
  current_pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_heading.data=yaw;
}

void point_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  current_point = *msg;
}

void line_cb(const geometry_msgs::Point::ConstPtr& msg)
{
  current_line = *msg;
}

void point_confi_cb(const std_msgs::Float64::ConstPtr& msg)
{
  confidence_circle=msg->data;
  point_avalible=true;
}

void line_confi_cb(const std_msgs::Float64::ConstPtr& msg)
{
  confidence_line=msg->data;
  line_avalible=true;
}
/***************************************call backs end****************************************/

//set orientation of the drone (drone should always be level)

/*
Heading sets the yaw of base_link in snapshot_takeoff in rad
base_link and snapshot_takeoff are both FLU frames.
Thus,the x axis of snapshot_takeoff is 0 rad.
must call send_tf_snapshot_takeoff before takeoff to take a snapshot.
*/
void set_pose_local(float x, float y, float z, float heading)//x y z is in reference to the body (base_link) frame captured when taking off
{
  heading = heading + GYM_OFFSET;//Sent pose gets transformed from ENU to NED later in order to be sent to FCU.
  float yaw = heading;//YAW: from X(E) to Y(N) in ENU frame with X(E) axis being 0 deg
  float pitch = 0;//level
  float roll = 0;//level

  float cy = cos(yaw * 0.5);
  float sy = sin(yaw * 0.5);
  float cr = cos(roll * 0.5);
  float sr = sin(roll * 0.5);
  float cp = cos(pitch * 0.5);
  float sp = sin(pitch * 0.5);

  float qw = cy * cr * cp + sy * sr * sp;
  float qx = cy * sr * cp - sy * cr * sp;
  float qy = cy * cr * sp + sy * sr * cp;
  float qz = sy * cr * cp - cy * sr * sp;

  pose.pose.orientation.w = qw;
  pose.pose.orientation.x = qx;
  pose.pose.orientation.y = qy;
  pose.pose.orientation.z = qz;

 geometry_msgs::TransformStamped transformStamped;
 geometry_msgs::PointStamped  initial_pt,transformed_pt;
  try{
    transformStamped = tfBuffer.lookupTransform("map", "snapshot_takeoff", ros::Time(0));

    initial_pt.point.x=x;
    initial_pt.point.y=y;
    initial_pt.point.z=z;
    tf2::doTransform(initial_pt, transformed_pt, transformStamped);
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s",ex.what());
    ros::Duration(1.0).sleep();
    return;
  }
  float X = transformed_pt.point.x;
  float Y = transformed_pt.point.y;
  float Z = transformed_pt.point.z;
  pose.pose.position.x = X;
  pose.pose.position.y = Y;
  pose.pose.position.z = Z; 
  local_pos_pub.publish(pose);
  ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

void send_tf_snapshot_takeoff(void)
{
  static tf2_ros::StaticTransformBroadcaster  br;
  geometry_msgs::TransformStamped transformStamped;
  pose_snapshot=current_pose;;
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "snapshot_takeoff";
  transformStamped.transform.translation.x = pose_snapshot.pose.position.x;
  transformStamped.transform.translation.y = pose_snapshot.pose.position.y;
  transformStamped.transform.translation.z = pose_snapshot.pose.position.z;
  
  transformStamped.transform.rotation = pose_snapshot.pose.orientation;
  GYM_OFFSET=current_heading.data;
  br.sendTransform(transformStamped);
}

void send_gp_origin(ros::NodeHandle & nh)
{
  geographic_msgs::GeoPointStamped gp_origin;
  gp_origin.position.latitude=23.4657663284572;
  gp_origin.position.longitude=20.6268196552992;
  gp_origin.position.altitude=0;
  ros::Time ros_time = ros::Time::now();
  gp_origin.header.stamp.sec = ros_time.sec;
  gp_origin.header.stamp.nsec = ros_time.nsec;
  set_gp_origin_pub.publish(gp_origin);
  ROS_INFO("setting gp origin...");
}

int arm_drone(ros::NodeHandle & nh)
{
  // arming
  ros::ServiceClient arming_client_i = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  mavros_msgs::CommandBool srv_arm_i;
  srv_arm_i.request.value = true;
  if (arming_client_i.call(srv_arm_i) && srv_arm_i.response.success)
    ROS_INFO("ARM sent %d", srv_arm_i.response.success);
  else
  {
    ROS_ERROR("Failed arming");
    return -1;
  }
return 0;
}

int set_servo(ros::NodeHandle & nh,bool state)
{
  // arming
  ros::ServiceClient servo_client_i = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");
  mavros_msgs::CommandLong msg;
  msg.request.command=mavros_msgs::CommandCode::DO_SET_SERVO;
  msg.request.param1=9;
  msg.request.param2=state?1900:1500;

  if (servo_client_i.call(msg) && msg.response.success)
    ROS_INFO("servo msg sent");
  else
  {
    ROS_ERROR("Failed servo");
    return -1;
  }
return 0;
}

int takeoff(ros::NodeHandle & nh,double height)//meters
{
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = height;
  if(takeoff_cl.call(srv_takeoff)){
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  }else{
    ROS_ERROR("Failed Takeoff");
    return -1;
  }
return 0;
}

int land(ros::NodeHandle & nh)
{
  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
  mavros_msgs::CommandTOL srv_land;
  if (land_client.call(srv_land) && srv_land.response.success)
    ROS_INFO("land sent %d", srv_land.response.success);
  else
  {
    ROS_ERROR("Landing failed");
    ros::shutdown();
    return -1;
  }
return 0;
}

int set_speed_body(double x,double y,double z,double yaw_rate) //flu meter/s rad/s, must set continously, or the vehicle stops after a few seconds(failsafe feature). yaw_rate = 0 when not used.
{
  mavros_msgs::PositionTarget raw_target;
  raw_target.coordinate_frame=PositionTarget::FRAME_BODY_OFFSET_NED;
  raw_target.type_mask=PositionTarget::IGNORE_PX|PositionTarget::IGNORE_PY|PositionTarget::IGNORE_PZ|PositionTarget::IGNORE_AFX|PositionTarget::IGNORE_AFY|PositionTarget::IGNORE_AFZ|PositionTarget::IGNORE_YAW;
  if(fabs(yaw_rate)<1e-6)raw_target.type_mask|=PositionTarget::IGNORE_YAW_RATE;
  raw_target.velocity.x=x;
  raw_target.velocity.y=y;
  raw_target.velocity.z=z;
  raw_target.yaw_rate=yaw_rate;
  set_raw_pub.publish(raw_target);
  return 0;
}

int set_angular_rate(double yaw_rate) //FLU rad/s , must set continously, or the vehicle stops after a few seconds.(failsafe feature) used for adjusting yaw without setting others. 
{
  mavros_msgs::PositionTarget raw_target;
  raw_target.coordinate_frame=PositionTarget::FRAME_BODY_OFFSET_NED;
  raw_target.type_mask=PositionTarget::IGNORE_VX|PositionTarget::IGNORE_VY|PositionTarget::IGNORE_VZ|PositionTarget::IGNORE_AFX|PositionTarget::IGNORE_AFY|PositionTarget::IGNORE_AFZ|PositionTarget::IGNORE_YAW;//yaw_rate must be used with pose or vel.
  raw_target.position.x=0;
  raw_target.position.y=0;
  raw_target.position.z=0;
  raw_target.yaw_rate=yaw_rate;
  set_raw_pub.publish(raw_target);
  return 0;
}

int set_pose_body(double x,double y,double z,double yaw)//flu meters rad. yaw = 0 when not used. x=y=z=0 when not used.
{
  mavros_msgs::PositionTarget raw_target;
  raw_target.coordinate_frame=PositionTarget::FRAME_BODY_OFFSET_NED;
  raw_target.type_mask=PositionTarget::IGNORE_VX|PositionTarget::IGNORE_VY|PositionTarget::IGNORE_VZ|PositionTarget::IGNORE_AFX|PositionTarget::IGNORE_AFY|PositionTarget::IGNORE_AFZ|PositionTarget::IGNORE_YAW_RATE;
  if(fabs(yaw)<1e-6)raw_target.type_mask|=PositionTarget::IGNORE_YAW;
  raw_target.position.x=x;
  raw_target.position.y=y;
  raw_target.position.z=z;
  raw_target.yaw=yaw;
  set_raw_pub.publish(raw_target);
  return 0;
}

int set_break()
{
  mavros_msgs::PositionTarget raw_target;
  raw_target.coordinate_frame=PositionTarget::FRAME_BODY_OFFSET_NED;
  raw_target.type_mask=PositionTarget::IGNORE_VX|PositionTarget::IGNORE_VY|PositionTarget::IGNORE_VZ|PositionTarget::IGNORE_AFX|PositionTarget::IGNORE_AFY|PositionTarget::IGNORE_AFZ|PositionTarget::IGNORE_YAW_RATE;
  raw_target.position.x=0;
  raw_target.position.y=0;
  raw_target.position.z=0;
  raw_target.yaw=0;
  set_raw_pub.publish(raw_target);
  return 0;
}

void set_camera_down_mode(int mode)
{
  std_msgs::Int8 mode_pub_data;
  mode_pub_data.data=mode;
  mode_pub.publish(mode_pub_data);
}

template <typename T>
T  bound (T const& a,T const& limit) 
{ 
  if(a<-limit)return -limit;
  else if(a>limit)return limit;
  else return a;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_node");
  ros::NodeHandle nh;
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  set_raw_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  set_gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
  ros::Publisher loopback_pub = nh.advertise<geometry_msgs::Point>("/camera_down/loopback", 10);
/*****************************camera subs and pubs*****************************/
  ros::Subscriber point_sub = nh.subscribe<geometry_msgs::Point>("/camera_down/pose", 10, point_cb);
  ros::Subscriber line_sub = nh.subscribe<geometry_msgs::Point>("/camera_down/line", 10, line_cb);
  ros::Subscriber point_confi_sub = nh.subscribe<std_msgs::Float64>("/camera_down/pose_confi", 10, point_confi_cb);
  ros::Subscriber line_confi_sub = nh.subscribe<std_msgs::Float64>("/camera_down/line_confi", 10, line_confi_cb);
  mode_pub = nh.advertise<std_msgs::Int8>("/camera_down/mode", 10);
/*****************************camera subs and pubs end*************************/



  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
  tf2_ros::TransformListener tfListener(tfBuffer);



  // allow the subscribers to initialize
  ROS_INFO("INITILIZING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  // set_camera_down_mode(0x01);//point only
  
  // while(!point_avalible) //wait for camera
  // {
  //   ros::spinOnce();
  //   ros::Duration(0.01).sleep();
  // }

  // while(current_state.mode != "GUIDED") //wait for remote command
  // {
  //   ros::spinOnce();
  //   ros::Duration(0.01).sleep();
  // }

  // send_gp_origin(nh);

  // for(int i=0; i<200; i++)
  // {
  //   ros::spinOnce();
  //   ros::Duration(0.01).sleep();
  // }



  // // recheck for FCU connection
  // while (ros::ok() && !current_state.connected)
  // {
  //   ros::spinOnce();
  //   rate.sleep();
  // }
  // //arm
  // arm_drone(nh);
  
  // for(int i=0; i<500; i++)
  // {
  //   ros::spinOnce();
  //   ros::Duration(0.01).sleep();
  // }
  // send_tf_snapshot_takeoff();
  // //request takeoff
  // takeoff(nh,1.0);//meters

  // sleep(10);
  // ros::spinOnce();

for(;;)//(int i=1000;i>=0;i--) //20s fix
{
  ros::spinOnce();
  if(confidence_circle>=0.7)
  {
    loopback_pub.publish(current_point);
    double speed_x=(current_point.x+30)/350.0;//-30,0
    double speed_y=current_point.y/350.0;
    speed_x*=confidence_circle;
    speed_y*=confidence_circle;
    speed_x=bound<double>(speed_x,0.4);
    speed_y=bound<double>(speed_y,0.4);    
    set_speed_body(speed_x,speed_y,0,0);//flu
    ROS_INFO("fixing...");
  }

  ros::Duration(0.02).sleep();
}
//  set_break();
//   set_camera_down_mode(0x03);//line&circle


//   set_pose_body(0.1,0,0,0);

//   for(int i=0; i<300; i++)//3s
//   {
//     ros::spinOnce();
//     ros::Duration(0.01).sleep();
//   }

//   while(!line_avalible) //wait for camera
//   {
//     ros::spinOnce();
//     ros::Duration(0.01).sleep();
//   }

// for (int i=500;i>=0;i--) //5s line_follow
// {
//   ros::spinOnce();
//   if(confidence_line>=0.7)
//   {
//     double speed_x=0.10;
//     double speed_y=current_line.y/250.0;
//     double yaw_rate=current_line.x/2.0;
//     speed_x*=confidence_line;
//     speed_y*=confidence_line;
//     yaw_rate*=confidence_line;
//     speed_x=bound<double>(speed_x,0.4);
//     speed_y=bound<double>(speed_y,0.4);
//     yaw_rate=bound<double>(yaw_rate,30/180.0*M_PI); 
//     set_speed_body(speed_x,speed_y,0,yaw_rate);//flu
//     ROS_INFO("following1...");
//   }
//   else
//   {
//     set_speed_body(0.1,0,0,0);//flu
//   }
//   ros::Duration(0.02).sleep();
// }


// for (int i=500;i>=0;i--) //5s line_follow
// {
//   ros::spinOnce();
//   if(confidence_line>=0.7)
//   {
//     double speed_x=0.10;
//     double speed_y=current_line.y/250.0;
//     double yaw_rate=current_line.x/2.0;
//     speed_x*=confidence_line;
//     speed_y*=confidence_line;
//     yaw_rate*=confidence_line;
//     speed_x=bound<double>(speed_x,0.4);
//     speed_y=bound<double>(speed_y,0.4);
//     yaw_rate=bound<double>(yaw_rate,30/180.0*M_PI); 
//     set_speed_body(speed_x,speed_y,0,yaw_rate);//flu
//     ROS_INFO("following2...");
//   }
//   if(confidence_circle>=0.9)break;
//   ros::Duration(0.02).sleep();
// }

//   set_break();
//   set_camera_down_mode(0x01);//circle

//   for(int i=0; i<300; i++)//3s
//   {
//     ros::spinOnce();
//     ros::Duration(0.01).sleep();
//   }


// for (int i=500;i>=0;i--) //10s fix
// {
//   ros::spinOnce();
//   if(confidence_circle>=0.7)
//   {
//     double speed_x=(current_point.x+46)/250.0;
//     double speed_y=(current_point.y-15)/250.0;
//     speed_x*=confidence_circle;
//     speed_y*=confidence_circle;
//     speed_x=bound<double>(speed_x,0.3);
//     speed_y=bound<double>(speed_y,0.3);    
//     set_speed_body(speed_x,speed_y,0,0);//flu
//     ROS_INFO("fixing...");
//   }
//   ros::Duration(0.02).sleep();
// }

//   set_break();
//   set_servo(nh,true);
//   ros::spinOnce();
//   sleep(3);
  //land
// land(nh);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
