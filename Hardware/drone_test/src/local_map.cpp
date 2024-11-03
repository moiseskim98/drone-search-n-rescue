#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float64.h"
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <unistd.h>
#include <geometry_msgs/Pose2D.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <mavros_msgs/CommandTOL.h>
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

using namespace std;

//Set global variables
mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped pose;
std_msgs::Float64 current_heading;
float GYM_OFFSET;
tf2_ros::Buffer tfBuffer;

//get armed state
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
  ROS_INFO("x: %f y: %f z: %f", current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  tf2::Quaternion q(
  current_pose.pose.orientation.x,
  current_pose.pose.orientation.y,
  current_pose.pose.orientation.z,
  current_pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  current_heading.data=yaw*(180/M_PI);
// ROS_INFO("yaw: %f", yaw*(180/M_PI));
// ROS_INFO("y: %f", current_pose.pose.position.y);
// ROS_INFO("z: %f", current_pose.pose.position.z);
}


//set orientation of the drone (drone should always be level)

/*
setHeading sets the yaw of base_link in snapshot_takeoff in deg
base_link and snapshot_takeoff are both FLU frames.
Thus,the x axis of snapshot_takeoff is 0 deg.
*/

void setHeading(float heading)
{
  heading = heading + GYM_OFFSET;//Sent pose gets transformed from ENU to NED later in order to be sent to FCU.
  float yaw = heading*(M_PI/180);//YAW: from X(E) to Y(N) in ENU frame with X(E) axis being 0 deg
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

}

void setDestination(float x, float y, float z)//x y z is in reference to the body (base_link) frame captured when taking off
{
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
  ROS_INFO("Destination set to x: %f y: %f z %f", X, Y, Z);
}

void send_tf_snapshot_takeoff(void)
{
  static tf2_ros::StaticTransformBroadcaster  br;
  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseStamped pose_snapshot=current_pose;;
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
  ros::Publisher set_gp_origin_pub = nh.advertise<geographic_msgs::GeoPointStamped>("/mavros/global_position/set_gp_origin", 10);
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
int main(int argc, char** argv)
{
  ros::init(argc, argv, "drone_node");
  ros::NodeHandle nh;
  // the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(20.0);
  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
  ros::Publisher set_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
  ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
  ros::Subscriber currentPos = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, pose_cb);
  tf2_ros::TransformListener tfListener(tfBuffer);


  // allow the subscribers to initialize
  ROS_INFO("INITILIZING...");
  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }
  while(current_state.mode != "GUIDED")
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  send_gp_origin(nh);

  for(int i=0; i<100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  send_tf_snapshot_takeoff();

  ROS_INFO("the head is facing: %f", GYM_OFFSET);
  cout << GYM_OFFSET << "\n" << endl;


  // wait for FCU connection
  while (ros::ok() && !current_state.connected)
  {
    ros::spinOnce();
    rate.sleep();
  }
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


  //request takeoff
  ros::ServiceClient takeoff_cl = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/takeoff");
  mavros_msgs::CommandTOL srv_takeoff;
  srv_takeoff.request.altitude = 1.5;
  if(takeoff_cl.call(srv_takeoff)){
    ROS_INFO("takeoff sent %d", srv_takeoff.response.success);
  }else{
    ROS_ERROR("Failed Takeoff");
    return -1;
  }

  sleep(10);


  //move foreward
  setHeading(90);//in reference to snapshot_takeoff,FLU,direction:x to y
  setDestination(2, 0, 1.5);//in reference to snapshot_takeoff,FLU
  float tollorance = .35;
  if (local_pos_pub)
  {

    for (int i = 10000; ros::ok() && i > 0; --i)
    {

      local_pos_pub.publish(pose);

      float deltaX = abs(pose.pose.position.x - current_pose.pose.position.x);
      float deltaY = abs(pose.pose.position.y - current_pose.pose.position.y);
      float deltaZ = abs(pose.pose.position.z - current_pose.pose.position.z);
      //cout << " dx " << deltaX << " dy " << deltaY << " dz " << deltaZ << endl;
      float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) + pow(deltaZ, 2) );
      cout << dMag << endl;
      if( dMag < tollorance)
      {
        break;
      }
      ros::spinOnce();
      ros::Duration(0.5).sleep();
      if(i == 1)
      {
        ROS_INFO("Failed to reach destination. Stepping to next task.");
      }
    }
    ROS_INFO("Done moving foreward.");
  }

  //land
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

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
