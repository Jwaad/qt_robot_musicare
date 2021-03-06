// Generated by gencpp from file musi_care/qt_commandRequest.msg
// DO NOT EDIT!


#ifndef MUSI_CARE_MESSAGE_QT_COMMANDREQUEST_H
#define MUSI_CARE_MESSAGE_QT_COMMANDREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace musi_care
{
template <class ContainerAllocator>
struct qt_commandRequest_
{
  typedef qt_commandRequest_<ContainerAllocator> Type;

  qt_commandRequest_()
    : action_type()
    , action_content()
    , action_blocking(false)  {
    }
  qt_commandRequest_(const ContainerAllocator& _alloc)
    : action_type(_alloc)
    , action_content(_alloc)
    , action_blocking(false)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _action_type_type;
  _action_type_type action_type;

   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _action_content_type;
  _action_content_type action_content;

   typedef uint8_t _action_blocking_type;
  _action_blocking_type action_blocking;





  typedef boost::shared_ptr< ::musi_care::qt_commandRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::musi_care::qt_commandRequest_<ContainerAllocator> const> ConstPtr;

}; // struct qt_commandRequest_

typedef ::musi_care::qt_commandRequest_<std::allocator<void> > qt_commandRequest;

typedef boost::shared_ptr< ::musi_care::qt_commandRequest > qt_commandRequestPtr;
typedef boost::shared_ptr< ::musi_care::qt_commandRequest const> qt_commandRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::musi_care::qt_commandRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::musi_care::qt_commandRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::musi_care::qt_commandRequest_<ContainerAllocator1> & lhs, const ::musi_care::qt_commandRequest_<ContainerAllocator2> & rhs)
{
  return lhs.action_type == rhs.action_type &&
    lhs.action_content == rhs.action_content &&
    lhs.action_blocking == rhs.action_blocking;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::musi_care::qt_commandRequest_<ContainerAllocator1> & lhs, const ::musi_care::qt_commandRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace musi_care

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::musi_care::qt_commandRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::musi_care::qt_commandRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::musi_care::qt_commandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::musi_care::qt_commandRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::musi_care::qt_commandRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::musi_care::qt_commandRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::musi_care::qt_commandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "8c1433307c282c48513732c9c0efec5a";
  }

  static const char* value(const ::musi_care::qt_commandRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x8c1433307c282c48ULL;
  static const uint64_t static_value2 = 0x513732c9c0efec5aULL;
};

template<class ContainerAllocator>
struct DataType< ::musi_care::qt_commandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "musi_care/qt_commandRequest";
  }

  static const char* value(const ::musi_care::qt_commandRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::musi_care::qt_commandRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string action_type\n"
"string action_content\n"
"bool action_blocking\n"
;
  }

  static const char* value(const ::musi_care::qt_commandRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::musi_care::qt_commandRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_type);
      stream.next(m.action_content);
      stream.next(m.action_blocking);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct qt_commandRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::musi_care::qt_commandRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::musi_care::qt_commandRequest_<ContainerAllocator>& v)
  {
    s << indent << "action_type: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.action_type);
    s << indent << "action_content: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.action_content);
    s << indent << "action_blocking: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.action_blocking);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MUSI_CARE_MESSAGE_QT_COMMANDREQUEST_H
