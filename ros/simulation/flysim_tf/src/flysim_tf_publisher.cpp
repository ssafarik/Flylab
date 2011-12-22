#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <flysim/Pose.h>

std::string flysim_name;



void poseCallback(const flysim::PoseConstPtr& msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->x, msg->y, 0.0) );
  transform.setRotation( tf::Quaternion(msg->theta, 0, 0) );
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", flysim_name));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf_publisher");
  if (argc != 2){ROS_ERROR("need flysim name as argument"); return -1;};
  flysim_name = argv[1];

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(flysim_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};

