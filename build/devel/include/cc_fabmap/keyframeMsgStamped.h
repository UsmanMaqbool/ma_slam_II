// Generated by gencpp from file cc_fabmap/keyframeMsgStamped.msg
// DO NOT EDIT!


#ifndef CC_FABMAP_MESSAGE_KEYFRAMEMSGSTAMPED_H
#define CC_FABMAP_MESSAGE_KEYFRAMEMSGSTAMPED_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <std_msgs/Header.h>

namespace cc_fabmap
{
template <class ContainerAllocator>
struct keyframeMsgStamped_
{
  typedef keyframeMsgStamped_<ContainerAllocator> Type;

  keyframeMsgStamped_()
    : header()
    , id(0)
    , time(0.0)
    , isKeyframe(false)
    , camToWorld()
    , fx(0.0)
    , fy(0.0)
    , cx(0.0)
    , cy(0.0)
    , height(0)
    , width(0)
    , pointcloud()  {
      camToWorld.assign(0.0);
  }
  keyframeMsgStamped_(const ContainerAllocator& _alloc)
    : header(_alloc)
    , id(0)
    , time(0.0)
    , isKeyframe(false)
    , camToWorld()
    , fx(0.0)
    , fy(0.0)
    , cx(0.0)
    , cy(0.0)
    , height(0)
    , width(0)
    , pointcloud(_alloc)  {
  (void)_alloc;
      camToWorld.assign(0.0);
  }



   typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
  _header_type header;

   typedef int32_t _id_type;
  _id_type id;

   typedef double _time_type;
  _time_type time;

   typedef uint8_t _isKeyframe_type;
  _isKeyframe_type isKeyframe;

   typedef boost::array<float, 7>  _camToWorld_type;
  _camToWorld_type camToWorld;

   typedef float _fx_type;
  _fx_type fx;

   typedef float _fy_type;
  _fy_type fy;

   typedef float _cx_type;
  _cx_type cx;

   typedef float _cy_type;
  _cy_type cy;

   typedef uint32_t _height_type;
  _height_type height;

   typedef uint32_t _width_type;
  _width_type width;

   typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _pointcloud_type;
  _pointcloud_type pointcloud;




  typedef boost::shared_ptr< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> const> ConstPtr;

}; // struct keyframeMsgStamped_

typedef ::cc_fabmap::keyframeMsgStamped_<std::allocator<void> > keyframeMsgStamped;

typedef boost::shared_ptr< ::cc_fabmap::keyframeMsgStamped > keyframeMsgStampedPtr;
typedef boost::shared_ptr< ::cc_fabmap::keyframeMsgStamped const> keyframeMsgStampedConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cc_fabmap

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': True}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'cc_fabmap': ['/home/leo/new_cat/src/cc_fabmap/msg'], 'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> const>
  : TrueType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "edf8042cd030facf41093096999d0310";
  }

  static const char* value(const ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xedf8042cd030facfULL;
  static const uint64_t static_value2 = 0x41093096999d0310ULL;
};

template<class ContainerAllocator>
struct DataType< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cc_fabmap/keyframeMsgStamped";
  }

  static const char* value(const ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
{
  static const char* value()
  {
    return "Header header\n\
\n\
int32 id\n\
float64 time\n\
bool isKeyframe\n\
\n\
# camToWorld as serialization of sophus sim(3).\n\
# may change with keyframeGraph-updates.\n\
float32[7] camToWorld \n\
\n\
\n\
# camera parameter (fx fy cx cy), width, height\n\
# will never change, but required for display.\n\
float32 fx\n\
float32 fy\n\
float32 cx\n\
float32 cy\n\
uint32 height\n\
uint32 width\n\
\n\
\n\
# data as InputPointDense (float idepth, float idepth_var, uchar color[4]), width x height\n\
# may be empty, in that case no associated pointcloud is ever shown.\n\
uint8[] pointcloud\n\
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
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
";
  }

  static const char* value(const ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.header);
      stream.next(m.id);
      stream.next(m.time);
      stream.next(m.isKeyframe);
      stream.next(m.camToWorld);
      stream.next(m.fx);
      stream.next(m.fy);
      stream.next(m.cx);
      stream.next(m.cy);
      stream.next(m.height);
      stream.next(m.width);
      stream.next(m.pointcloud);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct keyframeMsgStamped_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cc_fabmap::keyframeMsgStamped_<ContainerAllocator>& v)
  {
    s << indent << "header: ";
    s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.id);
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
    s << indent << "isKeyframe: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.isKeyframe);
    s << indent << "camToWorld[]" << std::endl;
    for (size_t i = 0; i < v.camToWorld.size(); ++i)
    {
      s << indent << "  camToWorld[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.camToWorld[i]);
    }
    s << indent << "fx: ";
    Printer<float>::stream(s, indent + "  ", v.fx);
    s << indent << "fy: ";
    Printer<float>::stream(s, indent + "  ", v.fy);
    s << indent << "cx: ";
    Printer<float>::stream(s, indent + "  ", v.cx);
    s << indent << "cy: ";
    Printer<float>::stream(s, indent + "  ", v.cy);
    s << indent << "height: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.height);
    s << indent << "width: ";
    Printer<uint32_t>::stream(s, indent + "  ", v.width);
    s << indent << "pointcloud[]" << std::endl;
    for (size_t i = 0; i < v.pointcloud.size(); ++i)
    {
      s << indent << "  pointcloud[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.pointcloud[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // CC_FABMAP_MESSAGE_KEYFRAMEMSGSTAMPED_H
