// Generated by gencpp from file particle_filter/Pose.msg
// DO NOT EDIT!


#ifndef PARTICLE_FILTER_MESSAGE_POSE_H
#define PARTICLE_FILTER_MESSAGE_POSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace particle_filter
{
template <class ContainerAllocator>
struct Pose_
{
  typedef Pose_<ContainerAllocator> Type;

  Pose_()
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }
  Pose_(const ContainerAllocator& _alloc)
    : x(0.0)
    , y(0.0)
    , theta(0.0)  {
    }



   typedef double _x_type;
  _x_type x;

   typedef double _y_type;
  _y_type y;

   typedef double _theta_type;
  _theta_type theta;




  typedef boost::shared_ptr< ::particle_filter::Pose_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::particle_filter::Pose_<ContainerAllocator> const> ConstPtr;

}; // struct Pose_

typedef ::particle_filter::Pose_<std::allocator<void> > Pose;

typedef boost::shared_ptr< ::particle_filter::Pose > PosePtr;
typedef boost::shared_ptr< ::particle_filter::Pose const> PoseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::particle_filter::Pose_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::particle_filter::Pose_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace particle_filter

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'particle_filter': ['/home/rudy/Desktop/UMASS/CICS/data/catkin_ws/src/particle_filter/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::particle_filter::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::particle_filter::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::particle_filter::Pose_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::particle_filter::Pose_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::particle_filter::Pose_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::particle_filter::Pose_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::particle_filter::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "938fa65709584ad8e77d238529be13b8";
  }

  static const char* value(const ::particle_filter::Pose_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x938fa65709584ad8ULL;
  static const uint64_t static_value2 = 0xe77d238529be13b8ULL;
};

template<class ContainerAllocator>
struct DataType< ::particle_filter::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "particle_filter/Pose";
  }

  static const char* value(const ::particle_filter::Pose_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::particle_filter::Pose_<ContainerAllocator> >
{
  static const char* value()
  {
    return "float64 x\n\
float64 y\n\
float64 theta\n\
";
  }

  static const char* value(const ::particle_filter::Pose_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::particle_filter::Pose_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.x);
      stream.next(m.y);
      stream.next(m.theta);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct Pose_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::particle_filter::Pose_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::particle_filter::Pose_<ContainerAllocator>& v)
  {
    s << indent << "x: ";
    Printer<double>::stream(s, indent + "  ", v.x);
    s << indent << "y: ";
    Printer<double>::stream(s, indent + "  ", v.y);
    s << indent << "theta: ";
    Printer<double>::stream(s, indent + "  ", v.theta);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PARTICLE_FILTER_MESSAGE_POSE_H