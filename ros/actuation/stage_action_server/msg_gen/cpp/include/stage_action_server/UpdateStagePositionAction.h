/* Auto-generated by genmsg_cpp for file /home/ssafarik/git/Flyatar/ros/actuation/stage_action_server/msg/UpdateStagePositionAction.msg */
#ifndef STAGE_ACTION_SERVER_MESSAGE_UPDATESTAGEPOSITIONACTION_H
#define STAGE_ACTION_SERVER_MESSAGE_UPDATESTAGEPOSITIONACTION_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "stage_action_server/UpdateStagePositionActionGoal.h"
#include "stage_action_server/UpdateStagePositionActionResult.h"
#include "stage_action_server/UpdateStagePositionActionFeedback.h"

namespace stage_action_server
{
template <class ContainerAllocator>
struct UpdateStagePositionAction_ {
  typedef UpdateStagePositionAction_<ContainerAllocator> Type;

  UpdateStagePositionAction_()
  : action_goal()
  , action_result()
  , action_feedback()
  {
  }

  UpdateStagePositionAction_(const ContainerAllocator& _alloc)
  : action_goal(_alloc)
  , action_result(_alloc)
  , action_feedback(_alloc)
  {
  }

  typedef  ::stage_action_server::UpdateStagePositionActionGoal_<ContainerAllocator>  _action_goal_type;
   ::stage_action_server::UpdateStagePositionActionGoal_<ContainerAllocator>  action_goal;

  typedef  ::stage_action_server::UpdateStagePositionActionResult_<ContainerAllocator>  _action_result_type;
   ::stage_action_server::UpdateStagePositionActionResult_<ContainerAllocator>  action_result;

  typedef  ::stage_action_server::UpdateStagePositionActionFeedback_<ContainerAllocator>  _action_feedback_type;
   ::stage_action_server::UpdateStagePositionActionFeedback_<ContainerAllocator>  action_feedback;


private:
  static const char* __s_getDataType_() { return "stage_action_server/UpdateStagePositionAction"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "fc8b5fb1843eb6bdb98ca787eb9a04ec"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
UpdateStagePositionActionGoal action_goal\n\
UpdateStagePositionActionResult action_result\n\
UpdateStagePositionActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
UpdateStagePositionGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
UpdateStagePositionResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
UpdateStagePositionFeedback feedback\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, action_goal);
    ros::serialization::serialize(stream, action_result);
    ros::serialization::serialize(stream, action_feedback);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, action_goal);
    ros::serialization::deserialize(stream, action_result);
    ros::serialization::deserialize(stream, action_feedback);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(action_goal);
    size += ros::serialization::serializationLength(action_result);
    size += ros::serialization::serializationLength(action_feedback);
    return size;
  }

  typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct UpdateStagePositionAction
typedef  ::stage_action_server::UpdateStagePositionAction_<std::allocator<void> > UpdateStagePositionAction;

typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionAction> UpdateStagePositionActionPtr;
typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionAction const> UpdateStagePositionActionConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace stage_action_server

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fc8b5fb1843eb6bdb98ca787eb9a04ec";
  }

  static const char* value(const  ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfc8b5fb1843eb6bdULL;
  static const uint64_t static_value2 = 0xb98ca787eb9a04ecULL;
};

template<class ContainerAllocator>
struct DataType< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "stage_action_server/UpdateStagePositionAction";
  }

  static const char* value(const  ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
UpdateStagePositionActionGoal action_goal\n\
UpdateStagePositionActionResult action_result\n\
UpdateStagePositionActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
UpdateStagePositionGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.secs: seconds (stamp_secs) since epoch\n\
# * stamp.nsecs: nanoseconds since stamp_secs\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of postion and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Twist\n\
# This expresses velocity in free space broken into it's linear and angular parts. \n\
Vector3  linear\n\
Vector3  angular\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Vector3\n\
# This represents a vector in free space. \n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
UpdateStagePositionResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
UpdateStagePositionFeedback feedback\n\
\n\
================================================================================\n\
MSG: stage_action_server/UpdateStagePositionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
\n\
";
  }

  static const char* value(const  ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.action_goal);
    stream.next(m.action_result);
    stream.next(m.action_feedback);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct UpdateStagePositionAction_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::stage_action_server::UpdateStagePositionAction_<ContainerAllocator> & v) 
  {
    s << indent << "action_goal: ";
s << std::endl;
    Printer< ::stage_action_server::UpdateStagePositionActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
s << std::endl;
    Printer< ::stage_action_server::UpdateStagePositionActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
s << std::endl;
    Printer< ::stage_action_server::UpdateStagePositionActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};


} // namespace message_operations
} // namespace ros

#endif // STAGE_ACTION_SERVER_MESSAGE_UPDATESTAGEPOSITIONACTION_H

