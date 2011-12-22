/* Auto-generated by genmsg_cpp for file /home/ssafarik/git/Flyatar/ros/actuation/stage_action_server/msg/UpdateStagePositionResult.msg */
#ifndef STAGE_ACTION_SERVER_MESSAGE_UPDATESTAGEPOSITIONRESULT_H
#define STAGE_ACTION_SERVER_MESSAGE_UPDATESTAGEPOSITIONRESULT_H
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

#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"

namespace stage_action_server
{
template <class ContainerAllocator>
struct UpdateStagePositionResult_ {
  typedef UpdateStagePositionResult_<ContainerAllocator> Type;

  UpdateStagePositionResult_()
  : header()
  , pose()
  , velocity()
  {
  }

  UpdateStagePositionResult_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , pose(_alloc)
  , velocity(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef  ::geometry_msgs::Pose_<ContainerAllocator>  _pose_type;
   ::geometry_msgs::Pose_<ContainerAllocator>  pose;

  typedef  ::geometry_msgs::Twist_<ContainerAllocator>  _velocity_type;
   ::geometry_msgs::Twist_<ContainerAllocator>  velocity;


private:
  static const char* __s_getDataType_() { return "stage_action_server/UpdateStagePositionResult"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "b3dff8faff5245565795cc8a6e908f1f"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
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
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, header);
    ros::serialization::serialize(stream, pose);
    ros::serialization::serialize(stream, velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, header);
    ros::serialization::deserialize(stream, pose);
    ros::serialization::deserialize(stream, velocity);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(header);
    size += ros::serialization::serializationLength(pose);
    size += ros::serialization::serializationLength(velocity);
    return size;
  }

  typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct UpdateStagePositionResult
typedef  ::stage_action_server::UpdateStagePositionResult_<std::allocator<void> > UpdateStagePositionResult;

typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionResult> UpdateStagePositionResultPtr;
typedef boost::shared_ptr< ::stage_action_server::UpdateStagePositionResult const> UpdateStagePositionResultConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace stage_action_server

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "b3dff8faff5245565795cc8a6e908f1f";
  }

  static const char* value(const  ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xb3dff8faff524556ULL;
  static const uint64_t static_value2 = 0x5795cc8a6e908f1fULL;
};

template<class ContainerAllocator>
struct DataType< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "stage_action_server/UpdateStagePositionResult";
  }

  static const char* value(const  ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > {
  static const char* value() 
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
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
";
  }

  static const char* value(const  ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.pose);
    stream.next(m.velocity);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct UpdateStagePositionResult_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::stage_action_server::UpdateStagePositionResult_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "pose: ";
s << std::endl;
    Printer< ::geometry_msgs::Pose_<ContainerAllocator> >::stream(s, indent + "  ", v.pose);
    s << indent << "velocity: ";
s << std::endl;
    Printer< ::geometry_msgs::Twist_<ContainerAllocator> >::stream(s, indent + "  ", v.velocity);
  }
};


} // namespace message_operations
} // namespace ros

#endif // STAGE_ACTION_SERVER_MESSAGE_UPDATESTAGEPOSITIONRESULT_H

