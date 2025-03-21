// Generated by gencpp from file arm_control_tcp/ServoJ.msg
// DO NOT EDIT!


#ifndef ARM_CONTROL_TCP_MESSAGE_SERVOJ_H
#define ARM_CONTROL_TCP_MESSAGE_SERVOJ_H

#include <ros/service_traits.h>


#include <arm_control_tcp/ServoJRequest.h>
#include <arm_control_tcp/ServoJResponse.h>


namespace arm_control_tcp
{

struct ServoJ
{

typedef ServoJRequest Request;
typedef ServoJResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct ServoJ
} // namespace arm_control_tcp


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::arm_control_tcp::ServoJ > {
  static const char* value()
  {
    return "acbcde02d4dabec81bbbb65ee94d60dc";
  }

  static const char* value(const ::arm_control_tcp::ServoJ&) { return value(); }
};

template<>
struct DataType< ::arm_control_tcp::ServoJ > {
  static const char* value()
  {
    return "arm_control_tcp/ServoJ";
  }

  static const char* value(const ::arm_control_tcp::ServoJ&) { return value(); }
};


// service_traits::MD5Sum< ::arm_control_tcp::ServoJRequest> should match
// service_traits::MD5Sum< ::arm_control_tcp::ServoJ >
template<>
struct MD5Sum< ::arm_control_tcp::ServoJRequest>
{
  static const char* value()
  {
    return MD5Sum< ::arm_control_tcp::ServoJ >::value();
  }
  static const char* value(const ::arm_control_tcp::ServoJRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::arm_control_tcp::ServoJRequest> should match
// service_traits::DataType< ::arm_control_tcp::ServoJ >
template<>
struct DataType< ::arm_control_tcp::ServoJRequest>
{
  static const char* value()
  {
    return DataType< ::arm_control_tcp::ServoJ >::value();
  }
  static const char* value(const ::arm_control_tcp::ServoJRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::arm_control_tcp::ServoJResponse> should match
// service_traits::MD5Sum< ::arm_control_tcp::ServoJ >
template<>
struct MD5Sum< ::arm_control_tcp::ServoJResponse>
{
  static const char* value()
  {
    return MD5Sum< ::arm_control_tcp::ServoJ >::value();
  }
  static const char* value(const ::arm_control_tcp::ServoJResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::arm_control_tcp::ServoJResponse> should match
// service_traits::DataType< ::arm_control_tcp::ServoJ >
template<>
struct DataType< ::arm_control_tcp::ServoJResponse>
{
  static const char* value()
  {
    return DataType< ::arm_control_tcp::ServoJ >::value();
  }
  static const char* value(const ::arm_control_tcp::ServoJResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // ARM_CONTROL_TCP_MESSAGE_SERVOJ_H
