// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/ImageProcessing.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__IMAGE_PROCESSING__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__IMAGE_PROCESSING__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/image_processing__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ImageProcessing_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImageProcessing_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImageProcessing_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::ImageProcessing_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::ImageProcessing_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::ImageProcessing_Request>()
{
  return "interfaces::srv::ImageProcessing_Request";
}

template<>
inline const char * name<interfaces::srv::ImageProcessing_Request>()
{
  return "interfaces/srv/ImageProcessing_Request";
}

template<>
struct has_fixed_size<interfaces::srv::ImageProcessing_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::srv::ImageProcessing_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::srv::ImageProcessing_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const ImageProcessing_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ImageProcessing_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ImageProcessing_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::ImageProcessing_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::ImageProcessing_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::ImageProcessing_Response>()
{
  return "interfaces::srv::ImageProcessing_Response";
}

template<>
inline const char * name<interfaces::srv::ImageProcessing_Response>()
{
  return "interfaces/srv/ImageProcessing_Response";
}

template<>
struct has_fixed_size<interfaces::srv::ImageProcessing_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::ImageProcessing_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::ImageProcessing_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::ImageProcessing>()
{
  return "interfaces::srv::ImageProcessing";
}

template<>
inline const char * name<interfaces::srv::ImageProcessing>()
{
  return "interfaces/srv/ImageProcessing";
}

template<>
struct has_fixed_size<interfaces::srv::ImageProcessing>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::ImageProcessing_Request>::value &&
    has_fixed_size<interfaces::srv::ImageProcessing_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::ImageProcessing>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::ImageProcessing_Request>::value &&
    has_bounded_size<interfaces::srv::ImageProcessing_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::ImageProcessing>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::ImageProcessing_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::ImageProcessing_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__IMAGE_PROCESSING__TRAITS_HPP_
