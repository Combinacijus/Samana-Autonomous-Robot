// Generated by gencpp from file samana_msgs/ArmCmd.msg
// DO NOT EDIT!


#ifndef SAMANA_MSGS_MESSAGE_ARMCMD_H
#define SAMANA_MSGS_MESSAGE_ARMCMD_H


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
struct ArmCmd_
{
  typedef ArmCmd_<ContainerAllocator> Type;

  ArmCmd_()
    : grabber_cmd(0)
    , lifter_cmd(0)  {
    }
  ArmCmd_(const ContainerAllocator& _alloc)
    : grabber_cmd(0)
    , lifter_cmd(0)  {
  (void)_alloc;
    }



   typedef int8_t _grabber_cmd_type;
  _grabber_cmd_type grabber_cmd;

   typedef int8_t _lifter_cmd_type;
  _lifter_cmd_type lifter_cmd;





  typedef boost::shared_ptr< ::samana_msgs::ArmCmd_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::samana_msgs::ArmCmd_<ContainerAllocator> const> ConstPtr;

}; // struct ArmCmd_

typedef ::samana_msgs::ArmCmd_<std::allocator<void> > ArmCmd;

typedef boost::shared_ptr< ::samana_msgs::ArmCmd > ArmCmdPtr;
typedef boost::shared_ptr< ::samana_msgs::ArmCmd const> ArmCmdConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::samana_msgs::ArmCmd_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::samana_msgs::ArmCmd_<ContainerAllocator> >::stream(s, "", v);
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
struct IsFixedSize< ::samana_msgs::ArmCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::samana_msgs::ArmCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::samana_msgs::ArmCmd_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::samana_msgs::ArmCmd_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::samana_msgs::ArmCmd_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::samana_msgs::ArmCmd_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::samana_msgs::ArmCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4915ef1fb595c9707da4cb79c7caeeb8";
  }

  static const char* value(const ::samana_msgs::ArmCmd_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4915ef1fb595c970ULL;
  static const uint64_t static_value2 = 0x7da4cb79c7caeeb8ULL;
};

template<class ContainerAllocator>
struct DataType< ::samana_msgs::ArmCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "samana_msgs/ArmCmd";
  }

  static const char* value(const ::samana_msgs::ArmCmd_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::samana_msgs::ArmCmd_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int8 grabber_cmd\n"
"int8 lifter_cmd\n"
;
  }

  static const char* value(const ::samana_msgs::ArmCmd_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::samana_msgs::ArmCmd_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.grabber_cmd);
      stream.next(m.lifter_cmd);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ArmCmd_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::samana_msgs::ArmCmd_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::samana_msgs::ArmCmd_<ContainerAllocator>& v)
  {
    s << indent << "grabber_cmd: ";
    Printer<int8_t>::stream(s, indent + "  ", v.grabber_cmd);
    s << indent << "lifter_cmd: ";
    Printer<int8_t>::stream(s, indent + "  ", v.lifter_cmd);
  }
};

} // namespace message_operations
} // namespace ros

#endif // SAMANA_MSGS_MESSAGE_ARMCMD_H