#include <ros/ros.h>
#include <joystick_ps3/JoystickValues.h>
#include <Joy.h>
#include <math.h>

class JoystickValuePublisher
{
public:
  JoystickValuePublisher();

private:
  void joyCallback(const joy::Joy::ConstPtr& joy);

  ros::NodeHandle nh_;

  ros::Publisher joy_values_pub_;
  ros::Subscriber joy_sub_;

};

JoystickValuePublisher::JoystickValuePublisher()
{
  joy_values_pub_ = nh_.advertise<joystick_ps3::JoystickValues>("Joystick/Values", 1);
  joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &JoystickValuePublisher::joyCallback, this);
}

void JoystickValuePublisher::joyCallback(const joy::Joy::ConstPtr& joy)
{
  joystick_ps3::JoystickValues joy_values;

  joy_values.x_left = -joy->axes[0];
  joy_values.y_left = joy->axes[1];
  joy_values.x_right = -joy->axes[2];
  joy_values.y_right = joy->axes[3];
  joy_values.up = joy->buttons[4];
  joy_values.down = joy->buttons[6];
  joy_values.left = joy->buttons[7];
  joy_values.right = joy->buttons[5];
  joy_values.triangle = joy->buttons[12];
  joy_values.x = joy->buttons[14];
  joy_values.square = joy->buttons[15];
  joy_values.circle = joy->buttons[13];
  joy_values.select = joy->buttons[0];
  joy_values.start = joy->buttons[3];
  joy_values.playstation = joy->buttons[16];
  joy_values.L1 = joy->buttons[10];
  joy_values.L2 = joy->buttons[8];
  joy_values.R1 = joy->buttons[11];
  joy_values.R2 = joy->buttons[9];

  joy_values_pub_.publish(joy_values);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "JoystickPublisher");
  JoystickValuePublisher joystick_value_publisher;

  ros::spin();
}

