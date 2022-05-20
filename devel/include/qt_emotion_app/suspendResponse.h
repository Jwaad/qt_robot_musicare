// Generated by gencpp from file qt_emotion_app/suspendResponse.msg
// DO NOT EDIT!


#ifndef QT_EMOTION_APP_MESSAGE_SUSPENDRESPONSE_H
#define QT_EMOTION_APP_MESSAGE_SUSPENDRESPONSE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace qt_emotion_app
{
template <class ContainerAllocator>
struct suspendResponse_
{
  typedef suspendResponse_<ContainerAllocator> Type;

  suspendResponse_()
    : status(false)  {
    }
  suspendResponse_(const ContainerAllocator& _alloc)
    : status(false)  {
  (void)_alloc;
    }



   typedef uint8_t _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::qt_emotion_app::suspendResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::qt_emotion_app::suspendResponse_<ContainerAllocator> const> ConstPtr;

}; // struct suspendResponse_

typedef ::qt_emotion_app::suspendResponse_<std::allocator<void> > suspendResponse;

typedef boost::shared_ptr< ::qt_emotion_app::suspendResponse > suspendResponsePtr;
typedef boost::shared_ptr< ::qt_emotion_app::suspendResponse const> suspendResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::qt_emotion_app::suspendResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::qt_emotion_app::suspendResponse_<ContainerAllocator1> & lhs, const ::qt_emotion_app::suspendResponse_<ContainerAllocator2> & rhs)
{
  return lhs.status == rhs.status;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::qt_emotion_app::suspendResponse_<ContainerAllocator1> & lhs, const ::qt_emotion_app::suspendResponse_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace qt_emotion_app

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::qt_emotion_app::suspendResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::qt_emotion_app::suspendResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::qt_emotion_app::suspendResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "3a1255d4d998bd4d6585c64639b5ee9a";
  }

  static const char* value(const ::qt_emotion_app::suspendResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x3a1255d4d998bd4dULL;
  static const uint64_t static_value2 = 0x6585c64639b5ee9aULL;
};

template<class ContainerAllocator>
struct DataType< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "qt_emotion_app/suspendResponse";
  }

  static const char* value(const ::qt_emotion_app::suspendResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "bool status\n"
"\n"
;
  }

  static const char* value(const ::qt_emotion_app::suspendResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct suspendResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::qt_emotion_app::suspendResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::qt_emotion_app::suspendResponse_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // QT_EMOTION_APP_MESSAGE_SUSPENDRESPONSE_H
