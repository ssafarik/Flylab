#include <ros/ros.h>
#include <flysim/Velocity.h>
#include <joy/Joy.h>

class TeleopFlysim
{
public:
  TeleopFlysim();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  int linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;

};

TeleopFlysim::TeleopFlysim():
  linear_(1),
  angular_(2)
{

  nh_.param("axis_linear", linear_, linear_);
  nh_.param("axis_angular", angular_, angular_);
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);

  vel_pub_ = nh_.advertise<flysim::Velocity>("fly/command_velocity", 1);
  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopFlysim::joyCallback, this);
}

void TeleopFlysim::joyCallback(const joy::Joy::ConstPtr& joy)
{
  flysim::Velocity vel;
  vel.angular = a_scale_*joy->axes[angular_];
  vel.linear = l_scale_*joy->axes[linear_];
  vel_pub_.publish(vel);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_flysim");
  TeleopFlysim teleop_flysim;

  ros::spin();
}

