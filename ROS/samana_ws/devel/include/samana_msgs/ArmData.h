// Generated by gencpp from file samana_msgs/ArmData.msg
// DO NOT EDIT!


#ifndef SAMANA_MSGS_MESSAGE_ARMDATA_H
#define SAMANA_MSGS_MESSAGE_ARMDATA_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace samana_msgs
{
template <class ContainerAllocator>
struct ArmData_
{
  typedef ArmData_<ContainerAllocator> Type;

  ArmData_()
    : current_grabber(0)
    , current_lifter(0)
    , limit_switches(0)  {
    }
  ArmData_(const ContainerAllocator& _alloc)
    : current_grabber(0)
    , current_lifter(0)
    , limit_switches(0)  {
  (void)_alloc;
    }



   typedef int16_t _current_grabber_type;
  _current_grabber_type current_grabber;

   typedef int16_t _current_lifter_type;
  _current_lifter_type current_lifter;

   typedef uint8_t _limit_switches_type;
  _limit_switches_type limit_switches;





  typedef boost::shared_ptr< ::samana_msgs::ArmData_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::samana_msgs::ArmData_<ContainerAllocator> const> ConstPtr;

}; // struct ArmData_

typedef ::samana_msgs::ArmData_<std::allocator<void> > ArmData;

typedef boost::shared_ptr< ::samana_msgs::ArmData > ArmDataPtr;
typedef boost::shared_ptr< ::samana_msgs::ArmData const> ArmDataConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::samana_msgs::ArmData_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::samana_msgs::ArmData_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace samana_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'samana_msgs': ['/home/combinacijus/Documents/SamanaAutonomousRobot/ROS/samana_ws/src/samana_msgs/msg'], 'std_msgs': ['/opt/ros/melodic/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/melodic/share/geometry_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::samana_msgs::ArmData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::samana_msgs::ArmData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::samana_msgs::ArmData_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::samana_msgs::ArmData_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::samana_msgs::ArmData_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::samana_msgs::ArmData_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::samana_msgs::ArmData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3812456ac8efcba0c0c24a88991ba799";
  }

  static const char* value(const ::samana_msgs::ArmData_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3812456ac8efcba0ULL;
  static const uint64_t static_value2 = 0xc0c24a88991ba799ULL;
};

template<class ContainerAllocator>
struct DataType< ::samana_msgs::ArmData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "samana_msgs/ArmData";
  }

  static const char* value(const ::samana_msgs::ArmData_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::samana_msgs::ArmData_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int16 current_grabber\n"
"int16 current_lifter\n"
"uint8 limit_switches\n"
;
  }

  static const char* value(const ::samana_msgs::ArmData_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::samana_msgs::ArmData_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.current_grabber);
      stream.next(m.current_lifter);
      stream.next(m.limit_switches);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmData_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::samana_msgs::ArmData_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::samana_msgs::ArmData_<ContainerAllocator>& v)
  {
    s << indent << "current_grabber: ";
    Printer<int16_t>::stream(s, indent + "  ", v.current_grabber);
    s << indent << "current_lifter: ";
    Printer<int16_t>::stream(s, indent + "  ", v.current_lifter);
    s << indent << "limit_switches: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.limit_switches);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAMANA_MSGS_MESSAGE_ARMDATA_H