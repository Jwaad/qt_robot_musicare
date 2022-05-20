// Generated by gencpp from file qt_motors_controller/homeResponse.msg
// DO NOT EDIT!


#ifndef QT_MOTORS_CONTROLLER_MESSAGE_HOMERESPONSE_H
#define QT_MOTORS_CONTROLLER_MESSAGE_HOMERESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qt_motors_controller
{
template <class ContainerAllocator>
struct homeResponse_
{
  typedef homeResponse_<ContainerAllocator> Type;

  homeResponse_()
    : status(false)  {
    }
  homeResponse_(const ContainerAllocator& _alloc)
    : status(false)  {
  (void)_alloc;
    }



   typedef uint8_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::qt_motors_controller::homeResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qt_motors_controller::homeResponse_<ContainerAllocator> const> ConstPtr;

}; // struct homeResponse_

typedef ::qt_motors_controller::homeResponse_<std::allocator<void> > homeResponse;

typedef boost::shared_ptr< ::qt_motors_controller::homeResponse > homeResponsePtr;
typedef boost::shared_ptr< ::qt_motors_controller::homeResponse const> homeResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qt_motors_controller::homeResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qt_motors_controller::homeResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace qt_motors_controller

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'qt_motors_controller': ['/home/qtrobot/robot/code/qt_main/qt_hardware_interface/controllers/qt_motors_controller/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qt_motors_controller::homeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_motors_controller::homeResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_motors_controller::homeResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a1255d4d998bd4d6585c64639b5ee9a";
  }

  static const char* value(const ::qt_motors_controller::homeResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a1255d4d998bd4dULL;
  static const uint64_t static_value2 = 0x6585c64639b5ee9aULL;
};

template<class ContainerAllocator>
struct DataType< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qt_motors_controller/homeResponse";
  }

  static const char* value(const ::qt_motors_controller::homeResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool status\n\
\n\
";
  }

  static const char* value(const ::qt_motors_controller::homeResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct homeResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qt_motors_controller::homeResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qt_motors_controller::homeResponse_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QT_MOTORS_CONTROLLER_MESSAGE_HOMERESPONSE_H
