/* Auto-generated by genmsg_cpp for file /home/ssafarik/git/Flyatar2/ros/experiments/msg/ExperimentSettings.msg */
#ifndef EXPERIMENTS_MESSAGE_EXPERIMENTSETTINGS_H
#define EXPERIMENTS_MESSAGE_EXPERIMENTSETTINGS_H
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


namespace experiments
{
template <class ContainerAllocator>
struct ExperimentSettings_ {
  typedef ExperimentSettings_<ContainerAllocator> Type;

  ExperimentSettings_()
  : description()
  , maxTrials(0)
  , trial(0)
  {
  }

  ExperimentSettings_(const ContainerAllocator& _alloc)
  : description(_alloc)
  , maxTrials(0)
  , trial(0)
  {
  }

  typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _description_type;
  std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  description;

  typedef int32_t _maxTrials_type;
  int32_t maxTrials;

  typedef int32_t _trial_type;
  int32_t trial;


private:
  static const char* __s_getDataType_() { return "experiments/ExperimentSettings"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "42e02677c518fd77f77d9cd10747bc20"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "string description\n\
int32 maxTrials\n\
int32 trial \n\
\n\
\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, description);
    ros::serialization::serialize(stream, maxTrials);
    ros::serialization::serialize(stream, trial);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, description);
    ros::serialization::deserialize(stream, maxTrials);
    ros::serialization::deserialize(stream, trial);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(description);
    size += ros::serialization::serializationLength(maxTrials);
    size += ros::serialization::serializationLength(trial);
    return size;
  }

  typedef boost::shared_ptr< ::experiments::ExperimentSettings_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::experiments::ExperimentSettings_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct ExperimentSettings
typedef  ::experiments::ExperimentSettings_<std::allocator<void> > ExperimentSettings;

typedef boost::shared_ptr< ::experiments::ExperimentSettings> ExperimentSettingsPtr;
typedef boost::shared_ptr< ::experiments::ExperimentSettings const> ExperimentSettingsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::experiments::ExperimentSettings_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::experiments::ExperimentSettings_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace experiments

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::experiments::ExperimentSettings_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::experiments::ExperimentSettings_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::experiments::ExperimentSettings_<ContainerAllocator> > {
  static const char* value() 
  {
    return "42e02677c518fd77f77d9cd10747bc20";
  }

  static const char* value(const  ::experiments::ExperimentSettings_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x42e02677c518fd77ULL;
  static const uint64_t static_value2 = 0xf77d9cd10747bc20ULL;
};

template<class ContainerAllocator>
struct DataType< ::experiments::ExperimentSettings_<ContainerAllocator> > {
  static const char* value() 
  {
    return "experiments/ExperimentSettings";
  }

  static const char* value(const  ::experiments::ExperimentSettings_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::experiments::ExperimentSettings_<ContainerAllocator> > {
  static const char* value() 
  {
    return "string description\n\
int32 maxTrials\n\
int32 trial \n\
\n\
\n\
";
  }

  static const char* value(const  ::experiments::ExperimentSettings_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::experiments::ExperimentSettings_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.description);
    stream.next(m.maxTrials);
    stream.next(m.trial);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct ExperimentSettings_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::experiments::ExperimentSettings_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::experiments::ExperimentSettings_<ContainerAllocator> & v) 
  {
    s << indent << "description: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.description);
    s << indent << "maxTrials: ";
    Printer<int32_t>::stream(s, indent + "  ", v.maxTrials);
    s << indent << "trial: ";
    Printer<int32_t>::stream(s, indent + "  ", v.trial);
  }
};


} // namespace message_operations
} // namespace ros

#endif // EXPERIMENTS_MESSAGE_EXPERIMENTSETTINGS_H

