/* Auto-generated by genmsg_cpp for file /home/ssafarik/git/Flyatar2/ros/sensing/processing/track_image_contours/msg/ArenaState.msg */
#ifndef TRACK_IMAGE_CONTOURS_MESSAGE_ARENASTATE_H
#define TRACK_IMAGE_CONTOURS_MESSAGE_ARENASTATE_H
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

#include "flystage/MsgFrameState.h"
#include "flystage/MsgFrameState.h"
#include "track_image_contours/Stopped.h"
#include "track_image_contours/Stopped.h"

namespace track_image_contours
{
template <class ContainerAllocator>
struct ArenaState_ {
  typedef ArenaState_<ContainerAllocator> Type;

  ArenaState_()
  : robot()
  , flies()
  , robot_stopped()
  , fly_stopped()
  {
  }

  ArenaState_(const ContainerAllocator& _alloc)
  : robot(_alloc)
  , flies(_alloc)
  , robot_stopped(_alloc)
  , fly_stopped(_alloc)
  {
  }

  typedef  ::flystage::MsgFrameState_<ContainerAllocator>  _robot_type;
   ::flystage::MsgFrameState_<ContainerAllocator>  robot;

  typedef std::vector< ::flystage::MsgFrameState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flystage::MsgFrameState_<ContainerAllocator> >::other >  _flies_type;
  std::vector< ::flystage::MsgFrameState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flystage::MsgFrameState_<ContainerAllocator> >::other >  flies;

  typedef  ::track_image_contours::Stopped_<ContainerAllocator>  _robot_stopped_type;
   ::track_image_contours::Stopped_<ContainerAllocator>  robot_stopped;

  typedef  ::track_image_contours::Stopped_<ContainerAllocator>  _fly_stopped_type;
   ::track_image_contours::Stopped_<ContainerAllocator>  fly_stopped;


  ROS_DEPRECATED uint32_t get_flies_size() const { return (uint32_t)flies.size(); }
  ROS_DEPRECATED void set_flies_size(uint32_t size) { flies.resize((size_t)size); }
  ROS_DEPRECATED void get_flies_vec(std::vector< ::flystage::MsgFrameState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flystage::MsgFrameState_<ContainerAllocator> >::other > & vec) const { vec = this->flies; }
  ROS_DEPRECATED void set_flies_vec(const std::vector< ::flystage::MsgFrameState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::flystage::MsgFrameState_<ContainerAllocator> >::other > & vec) { this->flies = vec; }
private:
  static const char* __s_getDataType_() { return "track_image_contours/ArenaState"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "bd846af7c93b07705a6eca8d496d7107"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "flystage/MsgFrameState robot\n\
flystage/MsgFrameState[] flies\n\
Stopped robot_stopped\n\
Stopped fly_stopped\n\
\n\
================================================================================\n\
MSG: flystage/MsgFrameState\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
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
================================================================================\n\
MSG: track_image_contours/Stopped\n\
bool stopped\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, robot);
    ros::serialization::serialize(stream, flies);
    ros::serialization::serialize(stream, robot_stopped);
    ros::serialization::serialize(stream, fly_stopped);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, robot);
    ros::serialization::deserialize(stream, flies);
    ros::serialization::deserialize(stream, robot_stopped);
    ros::serialization::deserialize(stream, fly_stopped);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(robot);
    size += ros::serialization::serializationLength(flies);
    size += ros::serialization::serializationLength(robot_stopped);
    size += ros::serialization::serializationLength(fly_stopped);
    return size;
  }

  typedef boost::shared_ptr< ::track_image_contours::ArenaState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::track_image_contours::ArenaState_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ArenaState
typedef  ::track_image_contours::ArenaState_<std::allocator<void> > ArenaState;

typedef boost::shared_ptr< ::track_image_contours::ArenaState> ArenaStatePtr;
typedef boost::shared_ptr< ::track_image_contours::ArenaState const> ArenaStateConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::track_image_contours::ArenaState_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::track_image_contours::ArenaState_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace track_image_contours

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::track_image_contours::ArenaState_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::track_image_contours::ArenaState_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::track_image_contours::ArenaState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bd846af7c93b07705a6eca8d496d7107";
  }

  static const char* value(const  ::track_image_contours::ArenaState_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xbd846af7c93b0770ULL;
  static const uint64_t static_value2 = 0x5a6eca8d496d7107ULL;
};

template<class ContainerAllocator>
struct DataType< ::track_image_contours::ArenaState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "track_image_contours/ArenaState";
  }

  static const char* value(const  ::track_image_contours::ArenaState_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::track_image_contours::ArenaState_<ContainerAllocator> > {
  static const char* value() 
  {
    return "flystage/MsgFrameState robot\n\
flystage/MsgFrameState[] flies\n\
Stopped robot_stopped\n\
Stopped fly_stopped\n\
\n\
================================================================================\n\
MSG: flystage/MsgFrameState\n\
Header header\n\
geometry_msgs/Pose pose\n\
geometry_msgs/Twist velocity\n\
\n\
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
================================================================================\n\
MSG: track_image_contours/Stopped\n\
bool stopped\n\
";
  }

  static const char* value(const  ::track_image_contours::ArenaState_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::track_image_contours::ArenaState_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.robot);
    stream.next(m.flies);
    stream.next(m.robot_stopped);
    stream.next(m.fly_stopped);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ArenaState_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::track_image_contours::ArenaState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::track_image_contours::ArenaState_<ContainerAllocator> & v) 
  {
    s << indent << "robot: ";
s << std::endl;
    Printer< ::flystage::MsgFrameState_<ContainerAllocator> >::stream(s, indent + "  ", v.robot);
    s << indent << "flies[]" << std::endl;
    for (size_t i = 0; i < v.flies.size(); ++i)
    {
      s << indent << "  flies[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::flystage::MsgFrameState_<ContainerAllocator> >::stream(s, indent + "    ", v.flies[i]);
    }
    s << indent << "robot_stopped: ";
s << std::endl;
    Printer< ::track_image_contours::Stopped_<ContainerAllocator> >::stream(s, indent + "  ", v.robot_stopped);
    s << indent << "fly_stopped: ";
s << std::endl;
    Printer< ::track_image_contours::Stopped_<ContainerAllocator> >::stream(s, indent + "  ", v.fly_stopped);
  }
};


} // namespace message_operations
} // namespace ros

#endif // TRACK_IMAGE_CONTOURS_MESSAGE_ARENASTATE_H

