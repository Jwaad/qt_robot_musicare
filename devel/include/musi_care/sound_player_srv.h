// Generated by gencpp from file musi_care/sound_player_srv.msg
// DO NOT EDIT!


#ifndef MUSI_CARE_MESSAGE_SOUND_PLAYER_SRV_H
#define MUSI_CARE_MESSAGE_SOUND_PLAYER_SRV_H

#include <ros/service_traits.h>


#include <musi_care/sound_player_srvRequest.h>
#include <musi_care/sound_player_srvResponse.h>


namespace musi_care
{

struct sound_player_srv
{

typedef sound_player_srvRequest Request;
typedef sound_player_srvResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct sound_player_srv
} // namespace musi_care


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::musi_care::sound_player_srv > {
  static const char* value()
  {
    return "2a3bef821864039a35e20805c207d7a8";
  }

  static const char* value(const ::musi_care::sound_player_srv&) { return value(); }
};

template<>
struct DataType< ::musi_care::sound_player_srv > {
  static const char* value()
  {
    return "musi_care/sound_player_srv";
  }

  static const char* value(const ::musi_care::sound_player_srv&) { return value(); }
};


// service_traits::MD5Sum< ::musi_care::sound_player_srvRequest> should match
// service_traits::MD5Sum< ::musi_care::sound_player_srv >
template<>
struct MD5Sum< ::musi_care::sound_player_srvRequest>
{
  static const char* value()
  {
    return MD5Sum< ::musi_care::sound_player_srv >::value();
  }
  static const char* value(const ::musi_care::sound_player_srvRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::musi_care::sound_player_srvRequest> should match
// service_traits::DataType< ::musi_care::sound_player_srv >
template<>
struct DataType< ::musi_care::sound_player_srvRequest>
{
  static const char* value()
  {
    return DataType< ::musi_care::sound_player_srv >::value();
  }
  static const char* value(const ::musi_care::sound_player_srvRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::musi_care::sound_player_srvResponse> should match
// service_traits::MD5Sum< ::musi_care::sound_player_srv >
template<>
struct MD5Sum< ::musi_care::sound_player_srvResponse>
{
  static const char* value()
  {
    return MD5Sum< ::musi_care::sound_player_srv >::value();
  }
  static const char* value(const ::musi_care::sound_player_srvResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::musi_care::sound_player_srvResponse> should match
// service_traits::DataType< ::musi_care::sound_player_srv >
template<>
struct DataType< ::musi_care::sound_player_srvResponse>
{
  static const char* value()
  {
    return DataType< ::musi_care::sound_player_srv >::value();
  }
  static const char* value(const ::musi_care::sound_player_srvResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // MUSI_CARE_MESSAGE_SOUND_PLAYER_SRV_H
