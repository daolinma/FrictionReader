// Generated by gencpp from file netft_rdt_driver/Zero.msg
// DO NOT EDIT!


#ifndef NETFT_RDT_DRIVER_MESSAGE_ZERO_H
#define NETFT_RDT_DRIVER_MESSAGE_ZERO_H

#include <ros/service_traits.h>


#include <netft_rdt_driver/ZeroRequest.h>
#include <netft_rdt_driver/ZeroResponse.h>


namespace netft_rdt_driver
{

struct Zero
{

typedef ZeroRequest Request;
typedef ZeroResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct Zero
} // namespace netft_rdt_driver


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::netft_rdt_driver::Zero > {
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::netft_rdt_driver::Zero&) { return value(); }
};

template<>
struct DataType< ::netft_rdt_driver::Zero > {
  static const char* value()
  {
    return "netft_rdt_driver/Zero";
  }

  static const char* value(const ::netft_rdt_driver::Zero&) { return value(); }
};


// service_traits::MD5Sum< ::netft_rdt_driver::ZeroRequest> should match 
// service_traits::MD5Sum< ::netft_rdt_driver::Zero > 
template<>
struct MD5Sum< ::netft_rdt_driver::ZeroRequest>
{
  static const char* value()
  {
    return MD5Sum< ::netft_rdt_driver::Zero >::value();
  }
  static const char* value(const ::netft_rdt_driver::ZeroRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_rdt_driver::ZeroRequest> should match 
// service_traits::DataType< ::netft_rdt_driver::Zero > 
template<>
struct DataType< ::netft_rdt_driver::ZeroRequest>
{
  static const char* value()
  {
    return DataType< ::netft_rdt_driver::Zero >::value();
  }
  static const char* value(const ::netft_rdt_driver::ZeroRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::netft_rdt_driver::ZeroResponse> should match 
// service_traits::MD5Sum< ::netft_rdt_driver::Zero > 
template<>
struct MD5Sum< ::netft_rdt_driver::ZeroResponse>
{
  static const char* value()
  {
    return MD5Sum< ::netft_rdt_driver::Zero >::value();
  }
  static const char* value(const ::netft_rdt_driver::ZeroResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::netft_rdt_driver::ZeroResponse> should match 
// service_traits::DataType< ::netft_rdt_driver::Zero > 
template<>
struct DataType< ::netft_rdt_driver::ZeroResponse>
{
  static const char* value()
  {
    return DataType< ::netft_rdt_driver::Zero >::value();
  }
  static const char* value(const ::netft_rdt_driver::ZeroResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // NETFT_RDT_DRIVER_MESSAGE_ZERO_H
