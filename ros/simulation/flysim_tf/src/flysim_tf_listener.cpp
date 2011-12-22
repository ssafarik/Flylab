#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <flysim/Velocity.h>
#include <flysim/Spawn.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient add_flysim =
    node.serviceClient<flysim::Spawn>("spawn");
  flysim::Spawn srv;
  add_flysim.call(srv);

  ros::Publisher flysim_vel =
    node.advertise<flysim::Velocity>("robot/command_velocity", 10);

  tf::TransformListener listener;

  ros::Rate rate(10.0);
  while (node.ok()){
    tf::StampedTransform transform;
    try{
      listener.lookupTransform("/robot", "/fly",
                               ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    flysim::Velocity vel_msg;
    vel_msg.angular = 4.0 * atan2(transform.getOrigin().y(),
                                  transform.getOrigin().x());
    vel_msg.linear = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                                pow(transform.getOrigin().y(), 2));
    flysim_vel.publish(vel_msg);

    rate.sleep();
  }
  return 0;
};
