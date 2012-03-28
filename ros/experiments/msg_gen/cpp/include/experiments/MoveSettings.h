/* Auto-generated by genmsg_cpp for file /home/flybowl/ros_workspace/git/Flyatar2/ros/experiments/msg/MoveSettings.msg */
#ifndef EXPERIMENTS_MESSAGE_MOVESETTINGS_H
#define EXPERIMENTS_MESSAGE_MOVESETTINGS_H
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

#include "experiments/MoveRelative.h"
#include "experiments/MovePattern.h"

namespace experiments
{
template <class ContainerAllocator>
struct MoveSettings_ {
  typedef MoveSettings_<ContainerAllocator> Type;

  MoveSettings_()
  : enabled(false)
  , mode()
  , relative()
  , pattern()
  , timeout(0.0)
  {
  }

  MoveSettings_(const ContainerAllocator& _alloc)
  : enabled(false)
  , mode(_alloc)
  , relative(_alloc)
  , pattern(_alloc)
  , timeout(0.0)
  {
  }

  typedef uint8_t _enabled_type;
  uint8_t enabled;

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _mode_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  mode;

  typedef  ::experiments::MoveRelative_<ContainerAllocator>  _relative_type;
   ::experiments::MoveRelative_<ContainerAllocator>  relative;

  typedef  ::experiments::MovePattern_<ContainerAllocator>  _pattern_type;
   ::experiments::MovePattern_<ContainerAllocator>  pattern;

  typedef double _timeout_type;
  double timeout;


private:
  static const char* __s_getDataType_() { return "experiments/MoveSettings"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "3b606570690c9cb62ec78b99c161556f"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool enabled\n\
string mode  # 'pattern' or 'relative'\n\
MoveRelative relative\n\
MovePattern pattern\n\
float64 timeout\n\
\n\
\n\
================================================================================\n\
MSG: experiments/MoveRelative\n\
bool tracking\n\
string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'\n\
string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'\n\
float64 distance\n\
float64 angle\n\
string angleType # 'random' or 'constant'\n\
float64 speed\n\
string speedType # 'random' or 'constant'\n\
float64 tolerance\n\
\n\
\n\
================================================================================\n\
MSG: experiments/MovePattern\n\
string shape  # 'constant' or 'ramp' or 'circle' or 'square' or 'flylogo' or 'spiral'\n\
float64 hzPattern\n\
float64 hzPoint\n\
int32 count  # -1 means forever\n\
float64 radius\n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, enabled);
    ros::serialization::serialize(stream, mode);
    ros::serialization::serialize(stream, relative);
    ros::serialization::serialize(stream, pattern);
    ros::serialization::serialize(stream, timeout);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, enabled);
    ros::serialization::deserialize(stream, mode);
    ros::serialization::deserialize(stream, relative);
    ros::serialization::deserialize(stream, pattern);
    ros::serialization::deserialize(stream, timeout);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(enabled);
    size += ros::serialization::serializationLength(mode);
    size += ros::serialization::serializationLength(relative);
    size += ros::serialization::serializationLength(pattern);
    size += ros::serialization::serializationLength(timeout);
    return size;
  }

  typedef boost::shared_ptr< ::experiments::MoveSettings_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::experiments::MoveSettings_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct MoveSettings
typedef  ::experiments::MoveSettings_<std::allocator<void> > MoveSettings;

typedef boost::shared_ptr< ::experiments::MoveSettings> MoveSettingsPtr;
typedef boost::shared_ptr< ::experiments::MoveSettings const> MoveSettingsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::experiments::MoveSettings_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::experiments::MoveSettings_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace experiments

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::experiments::MoveSettings_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::experiments::MoveSettings_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::experiments::MoveSettings_<ContainerAllocator> > {
  static const char* value() 
  {
    return "3b606570690c9cb62ec78b99c161556f";
  }

  static const char* value(const  ::experiments::MoveSettings_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x3b606570690c9cb6ULL;
  static const uint64_t static_value2 = 0x2ec78b99c161556fULL;
};

template<class ContainerAllocator>
struct DataType< ::experiments::MoveSettings_<ContainerAllocator> > {
  static const char* value() 
  {
    return "experiments/MoveSettings";
  }

  static const char* value(const  ::experiments::MoveSettings_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::experiments::MoveSettings_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool enabled\n\
string mode  # 'pattern' or 'relative'\n\
MoveRelative relative\n\
MovePattern pattern\n\
float64 timeout\n\
\n\
\n\
================================================================================\n\
MSG: experiments/MoveRelative\n\
bool tracking\n\
string frameidOriginPosition # 'Plate' or 'Robot' or 'Fly'\n\
string frameidOriginAngle # 'Plate' or 'Robot' or 'Fly'\n\
float64 distance\n\
float64 angle\n\
string angleType # 'random' or 'constant'\n\
float64 speed\n\
string speedType # 'random' or 'constant'\n\
float64 tolerance\n\
\n\
\n\
================================================================================\n\
MSG: experiments/MovePattern\n\
string shape  # 'constant' or 'ramp' or 'circle' or 'square' or 'flylogo' or 'spiral'\n\
float64 hzPattern\n\
float64 hzPoint\n\
int32 count  # -1 means forever\n\
float64 radius\n\
\n\
\n\
";
  }

  static const char* value(const  ::experiments::MoveSettings_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::experiments::MoveSettings_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.enabled);
    stream.next(m.mode);
    stream.next(m.relative);
    stream.next(m.pattern);
    stream.next(m.timeout);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct MoveSettings_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::experiments::MoveSettings_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::experiments::MoveSettings_<ContainerAllocator> & v) 
  {
    s << indent << "enabled: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.enabled);
    s << indent << "mode: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.mode);
    s << indent << "relative: ";
s << std::endl;
    Printer< ::experiments::MoveRelative_<ContainerAllocator> >::stream(s, indent + "  ", v.relative);
    s << indent << "pattern: ";
s << std::endl;
    Printer< ::experiments::MovePattern_<ContainerAllocator> >::stream(s, indent + "  ", v.pattern);
    s << indent << "timeout: ";
    Printer<double>::stream(s, indent + "  ", v.timeout);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EXPERIMENTS_MESSAGE_MOVESETTINGS_H

