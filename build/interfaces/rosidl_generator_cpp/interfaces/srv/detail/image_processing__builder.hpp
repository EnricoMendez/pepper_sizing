// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/ImageProcessing.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__IMAGE_PROCESSING__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__IMAGE_PROCESSING__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/image_processing__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageProcessing_Request_id
{
public:
  Init_ImageProcessing_Request_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::ImageProcessing_Request id(::interfaces::srv::ImageProcessing_Request::_id_type arg)
  {
    msg_.id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::ImageProcessing_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::ImageProcessing_Request>()
{
  return interfaces::srv::builder::Init_ImageProcessing_Request_id();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_ImageProcessing_Response_data
{
public:
  Init_ImageProcessing_Response_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::ImageProcessing_Response data(::interfaces::srv::ImageProcessing_Response::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::ImageProcessing_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::ImageProcessing_Response>()
{
  return interfaces::srv::builder::Init_ImageProcessing_Response_data();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__IMAGE_PROCESSING__BUILDER_HPP_
