// Generated by gencpp from file musi_care/qt_command.msg
// DO NOT EDIT!


#ifndef MUSI_CARE_MESSAGE_QT_COMMAND_H
#define MUSI_CARE_MESSAGE_QT_COMMAND_H

#include <ros/service_traits.h>


#include <musi_care/qt_commandRequest.h>
#include <musi_care/qt_commandResponse.h>


namespace musi_care
{

struct qt_command
{

typedef qt_commandRequest Request;
typedef qt_commandResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct qt_command
} // namespace musi_care


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::musi_care::qt_command > {
  static const char* value()
  {
    return "63c073458379b344b40c9cac0c2eacff";
  }

  static const char* value(const ::musi_care::qt_command&) { return value(); }
};

template<>
struct DataType< ::musi_care::qt_command > {
  static const char* value()
  {
    return "musi_care/qt_command";
  }

  static const char* value(const ::musi_care::qt_command&) { return value(); }
};


// service_traits::MD5Sum< ::musi_care::qt_commandRequest> should match
// service_traits::MD5Sum< ::musi_care::qt_command >
template<>
struct MD5Sum< ::musi_care::qt_commandRequest>
{
  static const char* value()
  {
    return MD5Sum< ::musi_care::qt_command >::value();
  }
  static const char* value(const ::musi_care::qt_commandRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::musi_care::qt_commandRequest> should match
// service_traits::DataType< ::musi_care::qt_command >
template<>
struct DataType< ::musi_care::qt_commandRequest>
{
  static const char* value()
  {
    return DataType< ::musi_care::qt_command >::value();
  }
  static const char* value(const ::musi_care::qt_commandRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::musi_care::qt_commandResponse> should match
// service_traits::MD5Sum< ::musi_care::qt_command >
template<>
struct MD5Sum< ::musi_care::qt_commandResponse>
{
  static const char* value()
  {
    return MD5Sum< ::musi_care::qt_command >::value();
  }
  static const char* value(const ::musi_care::qt_commandResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::musi_care::qt_commandResponse> should match
// service_traits::DataType< ::musi_care::qt_command >
template<>
struct DataType< ::musi_care::qt_commandResponse>
{
  static const char* value()
  {
    return DataType< ::musi_care::qt_command >::value();
  }
  static const char* value(const ::musi_care::qt_commandResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MUSI_CARE_MESSAGE_QT_COMMAND_H
