/* Auto-generated by genmsg_cpp for file /home/ssafarik/git/Flyatar/ros/gui/image_gui/msg/DrawObjects.msg */
#ifndef IMAGE_GUI_MESSAGE_DRAWOBJECTS_H
#define IMAGE_GUI_MESSAGE_DRAWOBJECTS_H
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

#include "image_gui/DrawObject.h"

namespace image_gui
{
template <class ContainerAllocator>
struct DrawObjects_ {
  typedef DrawObjects_<ContainerAllocator> Type;

  DrawObjects_()
  : show_all(false)
  , hide_all(false)
  , draw_object_list()
  {
  }

  DrawObjects_(const ContainerAllocator& _alloc)
  : show_all(false)
  , hide_all(false)
  , draw_object_list(_alloc)
  {
  }

  typedef uint8_t _show_all_type;
  uint8_t show_all;

  typedef uint8_t _hide_all_type;
  uint8_t hide_all;

  typedef std::vector< ::image_gui::DrawObject_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::image_gui::DrawObject_<ContainerAllocator> >::other >  _draw_object_list_type;
  std::vector< ::image_gui::DrawObject_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::image_gui::DrawObject_<ContainerAllocator> >::other >  draw_object_list;


  ROS_DEPRECATED uint32_t get_draw_object_list_size() const { return (uint32_t)draw_object_list.size(); }
  ROS_DEPRECATED void set_draw_object_list_size(uint32_t size) { draw_object_list.resize((size_t)size); }
  ROS_DEPRECATED void get_draw_object_list_vec(std::vector< ::image_gui::DrawObject_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::image_gui::DrawObject_<ContainerAllocator> >::other > & vec) const { vec = this->draw_object_list; }
  ROS_DEPRECATED void set_draw_object_list_vec(const std::vector< ::image_gui::DrawObject_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::image_gui::DrawObject_<ContainerAllocator> >::other > & vec) { this->draw_object_list = vec; }
private:
  static const char* __s_getDataType_() { return "image_gui/DrawObjects"; }
public:
  ROS_DEPRECATED static const std::string __s_getDataType() { return __s_getDataType_(); }

  ROS_DEPRECATED const std::string __getDataType() const { return __s_getDataType_(); }

private:
  static const char* __s_getMD5Sum_() { return "09167aa6575b6803d68a4405eaa911e3"; }
public:
  ROS_DEPRECATED static const std::string __s_getMD5Sum() { return __s_getMD5Sum_(); }

  ROS_DEPRECATED const std::string __getMD5Sum() const { return __s_getMD5Sum_(); }

private:
  static const char* __s_getMessageDefinition_() { return "bool show_all\n\
bool hide_all\n\
DrawObject[] draw_object_list\n\
\n\
================================================================================\n\
MSG: image_gui/DrawObject\n\
bool show\n\
CvPoint object_center\n\
CvLine[] line_list\n\
CvCircle[] circle_list\n\
================================================================================\n\
MSG: image_gui/CvPoint\n\
int32 x\n\
int32 y\n\
\n\
================================================================================\n\
MSG: image_gui/CvLine\n\
CvPoint point1\n\
CvPoint point2\n\
CvColor color\n\
int32 thickness\n\
int32 lineType\n\
int32 shift\n\
================================================================================\n\
MSG: image_gui/CvColor\n\
float64 red\n\
float64 green\n\
float64 blue\n\
================================================================================\n\
MSG: image_gui/CvCircle\n\
CvPoint center\n\
int32 radius\n\
CvColor color\n\
int32 thickness\n\
int32 lineType\n\
int32 shift\n\
"; }
public:
  ROS_DEPRECATED static const std::string __s_getMessageDefinition() { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED const std::string __getMessageDefinition() const { return __s_getMessageDefinition_(); }

  ROS_DEPRECATED virtual uint8_t *serialize(uint8_t *write_ptr, uint32_t seq) const
  {
    ros::serialization::OStream stream(write_ptr, 1000000000);
    ros::serialization::serialize(stream, show_all);
    ros::serialization::serialize(stream, hide_all);
    ros::serialization::serialize(stream, draw_object_list);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint8_t *deserialize(uint8_t *read_ptr)
  {
    ros::serialization::IStream stream(read_ptr, 1000000000);
    ros::serialization::deserialize(stream, show_all);
    ros::serialization::deserialize(stream, hide_all);
    ros::serialization::deserialize(stream, draw_object_list);
    return stream.getData();
  }

  ROS_DEPRECATED virtual uint32_t serializationLength() const
  {
    uint32_t size = 0;
    size += ros::serialization::serializationLength(show_all);
    size += ros::serialization::serializationLength(hide_all);
    size += ros::serialization::serializationLength(draw_object_list);
    return size;
  }

  typedef boost::shared_ptr< ::image_gui::DrawObjects_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::image_gui::DrawObjects_<ContainerAllocator>  const> ConstPtr;
  boost::shared_ptr<std::map<std::string, std::string> > __connection_header;
}; // struct DrawObjects
typedef  ::image_gui::DrawObjects_<std::allocator<void> > DrawObjects;

typedef boost::shared_ptr< ::image_gui::DrawObjects> DrawObjectsPtr;
typedef boost::shared_ptr< ::image_gui::DrawObjects const> DrawObjectsConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::image_gui::DrawObjects_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::image_gui::DrawObjects_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace image_gui

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::image_gui::DrawObjects_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::image_gui::DrawObjects_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::image_gui::DrawObjects_<ContainerAllocator> > {
  static const char* value() 
  {
    return "09167aa6575b6803d68a4405eaa911e3";
  }

  static const char* value(const  ::image_gui::DrawObjects_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x09167aa6575b6803ULL;
  static const uint64_t static_value2 = 0xd68a4405eaa911e3ULL;
};

template<class ContainerAllocator>
struct DataType< ::image_gui::DrawObjects_<ContainerAllocator> > {
  static const char* value() 
  {
    return "image_gui/DrawObjects";
  }

  static const char* value(const  ::image_gui::DrawObjects_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::image_gui::DrawObjects_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool show_all\n\
bool hide_all\n\
DrawObject[] draw_object_list\n\
\n\
================================================================================\n\
MSG: image_gui/DrawObject\n\
bool show\n\
CvPoint object_center\n\
CvLine[] line_list\n\
CvCircle[] circle_list\n\
================================================================================\n\
MSG: image_gui/CvPoint\n\
int32 x\n\
int32 y\n\
\n\
================================================================================\n\
MSG: image_gui/CvLine\n\
CvPoint point1\n\
CvPoint point2\n\
CvColor color\n\
int32 thickness\n\
int32 lineType\n\
int32 shift\n\
================================================================================\n\
MSG: image_gui/CvColor\n\
float64 red\n\
float64 green\n\
float64 blue\n\
================================================================================\n\
MSG: image_gui/CvCircle\n\
CvPoint center\n\
int32 radius\n\
CvColor color\n\
int32 thickness\n\
int32 lineType\n\
int32 shift\n\
";
  }

  static const char* value(const  ::image_gui::DrawObjects_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::image_gui::DrawObjects_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.show_all);
    stream.next(m.hide_all);
    stream.next(m.draw_object_list);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER;
}; // struct DrawObjects_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::image_gui::DrawObjects_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::image_gui::DrawObjects_<ContainerAllocator> & v) 
  {
    s << indent << "show_all: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.show_all);
    s << indent << "hide_all: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.hide_all);
    s << indent << "draw_object_list[]" << std::endl;
    for (size_t i = 0; i < v.draw_object_list.size(); ++i)
    {
      s << indent << "  draw_object_list[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::image_gui::DrawObject_<ContainerAllocator> >::stream(s, indent + "    ", v.draw_object_list[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // IMAGE_GUI_MESSAGE_DRAWOBJECTS_H

