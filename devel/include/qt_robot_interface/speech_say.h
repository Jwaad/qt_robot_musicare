// Generated by gencpp from file qt_robot_interface/speech_say.msg
// DO NOT EDIT!


#ifndef QT_ROBOT_INTERFACE_MESSAGE_SPEECH_SAY_H
#define QT_ROBOT_INTERFACE_MESSAGE_SPEECH_SAY_H

#include <ros/service_traits.h>


#include <qt_robot_interface/speech_sayRequest.h>
#include <qt_robot_interface/speech_sayResponse.h>


namespace qt_robot_interface
{

struct speech_say
{

typedef speech_sayRequest Request;
typedef speech_sayResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct speech_say
} // namespace qt_robot_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qt_robot_interface::speech_say > {
  static const char* value()
  {
    return "27e2edee8a095bc44ea85df9f9df3f10";
  }

  static const char* value(const ::qt_robot_interface::speech_say&) { return value(); }
};

template<>
struct DataType< ::qt_robot_interface::speech_say > {
  static const char* value()
  {
    return "qt_robot_interface/speech_say";
  }

  static const char* value(const ::qt_robot_interface::speech_say&) { return value(); }
};


// service_traits::MD5Sum< ::qt_robot_interface::speech_sayRequest> should match 
// service_traits::MD5Sum< ::qt_robot_interface::speech_say > 
template<>
struct MD5Sum< ::qt_robot_interface::speech_sayRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qt_robot_interface::speech_say >::value();
  }
  static const char* value(const ::qt_robot_interface::speech_sayRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_robot_interface::speech_sayRequest> should match 
// service_traits::DataType< ::qt_robot_interface::speech_say > 
template<>
struct DataType< ::qt_robot_interface::speech_sayRequest>
{
  static const char* value()
  {
    return DataType< ::qt_robot_interface::speech_say >::value();
  }
  static const char* value(const ::qt_robot_interface::speech_sayRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qt_robot_interface::speech_sayResponse> should match 
// service_traits::MD5Sum< ::qt_robot_interface::speech_say > 
template<>
struct MD5Sum< ::qt_robot_interface::speech_sayResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qt_robot_interface::speech_say >::value();
  }
  static const char* value(const ::qt_robot_interface::speech_sayResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_robot_interface::speech_sayResponse> should match 
// service_traits::DataType< ::qt_robot_interface::speech_say > 
template<>
struct DataType< ::qt_robot_interface::speech_sayResponse>
{
  static const char* value()
  {
    return DataType< ::qt_robot_interface::speech_say >::value();
  }
  static const char* value(const ::qt_robot_interface::speech_sayResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QT_ROBOT_INTERFACE_MESSAGE_SPEECH_SAY_H
