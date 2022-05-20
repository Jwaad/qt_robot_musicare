// Generated by gencpp from file qt_motors_controller/set_velocity.msg
// DO NOT EDIT!


#ifndef QT_MOTORS_CONTROLLER_MESSAGE_SET_VELOCITY_H
#define QT_MOTORS_CONTROLLER_MESSAGE_SET_VELOCITY_H

#include <ros/service_traits.h>


#include <qt_motors_controller/set_velocityRequest.h>
#include <qt_motors_controller/set_velocityResponse.h>


namespace qt_motors_controller
{

struct set_velocity
{

typedef set_velocityRequest Request;
typedef set_velocityResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct set_velocity
} // namespace qt_motors_controller


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qt_motors_controller::set_velocity > {
  static const char* value()
  {
    return "68003f66a5a441a7e064a7fc5cd19661";
  }

  static const char* value(const ::qt_motors_controller::set_velocity&) { return value(); }
};

template<>
struct DataType< ::qt_motors_controller::set_velocity > {
  static const char* value()
  {
    return "qt_motors_controller/set_velocity";
  }

  static const char* value(const ::qt_motors_controller::set_velocity&) { return value(); }
};


// service_traits::MD5Sum< ::qt_motors_controller::set_velocityRequest> should match 
// service_traits::MD5Sum< ::qt_motors_controller::set_velocity > 
template<>
struct MD5Sum< ::qt_motors_controller::set_velocityRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qt_motors_controller::set_velocity >::value();
  }
  static const char* value(const ::qt_motors_controller::set_velocityRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_motors_controller::set_velocityRequest> should match 
// service_traits::DataType< ::qt_motors_controller::set_velocity > 
template<>
struct DataType< ::qt_motors_controller::set_velocityRequest>
{
  static const char* value()
  {
    return DataType< ::qt_motors_controller::set_velocity >::value();
  }
  static const char* value(const ::qt_motors_controller::set_velocityRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qt_motors_controller::set_velocityResponse> should match 
// service_traits::MD5Sum< ::qt_motors_controller::set_velocity > 
template<>
struct MD5Sum< ::qt_motors_controller::set_velocityResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qt_motors_controller::set_velocity >::value();
  }
  static const char* value(const ::qt_motors_controller::set_velocityResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_motors_controller::set_velocityResponse> should match 
// service_traits::DataType< ::qt_motors_controller::set_velocity > 
template<>
struct DataType< ::qt_motors_controller::set_velocityResponse>
{
  static const char* value()
  {
    return DataType< ::qt_motors_controller::set_velocity >::value();
  }
  static const char* value(const ::qt_motors_controller::set_velocityResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QT_MOTORS_CONTROLLER_MESSAGE_SET_VELOCITY_H
