/*
 * Copyright (C) 2009, Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef ROSLIB_MESSAGE_TRAITS_H
#define ROSLIB_MESSAGE_TRAITS_H

#include "message_forward.h"
#include <ros/time.h>

#include <string>
#include <boost/utility/enable_if.hpp>
#include <boost/type_traits/remove_const.hpp>
#include <boost/type_traits/remove_reference.hpp>

namespace std_msgs
{
  ROS_DECLARE_MESSAGE(Header);
}

// 简单数据类型初始化走这里
#define ROS_IMPLEMENT_SIMPLE_TOPIC_TRAITS(msg, md5sum, datatype, definition) \
  namespace ros \
  { \
  namespace message_traits \
  { \
  template<> struct MD5Sum<msg> \
  { \
    static const char* value() { return md5sum; } \
    static const char* value(const msg&) { return value(); } \
  }; \
  template<> struct DataType<msg> \
  { \
    static const char* value() { return datatype; } \
    static const char* value(const msg&) { return value(); } \
  }; \
  template<> struct Definition<msg> \
  { \
    static const char* value() { return definition; } \
    static const char* value(const msg&) { return value(); } \
  }; \
  } \
  }

namespace ros
{
namespace message_traits
{

/**
@brief 用于编译类型真/假测试的基本类型。与Boost.MPL兼容。从此类型继承的类是真值。
 */
struct TrueType
{
  static const bool value = true;
  typedef TrueType type;
};

/**
@brief 用于编译类型真/假测试的基本类型。与Boost.MPL兼容。从此类型继承的类是假值。
 */
struct FalseType
{
  static const bool value = false;
  typedef FalseType type;
};

/**
@brief 简单数据类型是可以直接以数组形式进行memcpy的数据类型，即它是一个POD、固定大小的类型，
       且sizeof(M) == sum(serializationLength(M:a...))
 */
template<typename M, typename Enable = void> struct IsSimple : public FalseType {};
/**
@brief 固定大小的数据类型是大小恒定的数据类型，即它没有可变长度的数组或字符串
 */
template<typename M, typename Enable = void> struct IsFixedSize : public FalseType {};
/**
@brief HasHeader通知是否有一个标头，该标头作为消息中的第一件事进行序列化
 */
template<typename M, typename Enable = void> struct HasHeader : public FalseType {};

/**
 * \brief Am I message or not
 */
template<typename M, typename Enable = void> struct IsMessage : public FalseType {};

/**
 @brief 为消息提供md5sum，特化
 */
template<typename M, typename Enable = void>
struct MD5Sum
{
  static const char* value()
  {
    return M::__s_getMD5Sum().c_str();
  }

  static const char* value(const M& m)
  {
    return m.__getMD5Sum().c_str();
  }
};

/**
 @brief 为消息提供数据类型，特化
 */
template<typename M, typename Enable = void>
struct DataType
{
  static const char* value()
  {
    return M::__s_getDataType().c_str();
  }

  static const char* value(const M& m)
  {
    return m.__getDataType().c_str();
  }
};

/**
 * \brief Specialize to provide the definition for a message
 @brief 特化为消息提供定义
 */
template<typename M, typename Enable = void>
struct Definition
{
  static const char* value()
  {
    return M::__s_getMessageDefinition().c_str();
  }

  static const char* value(const M& m)
  {
    return m.__getMessageDefinition().c_str();
  }
};

/**
 @brief Header特性。在默认实现中，如果HasHeader<M>::value为true，则pointer()返回&m.header，否则返回NULL
 */
template<typename M, typename Enable = void>
struct Header
{
  static std_msgs::Header* pointer(M& m) { (void)m; return 0; }
  static std_msgs::Header const* pointer(const M& m) { (void)m; return 0; }
};

template<typename M>
struct Header<M, typename boost::enable_if<HasHeader<M> >::type >
{
  static std_msgs::Header* pointer(M& m) { return &m.header; }
  static std_msgs::Header const* pointer(const M& m) { return &m.header; }
};

/**
 @brief FrameId特性。在默认实现中，如果HasHeader<M>::value为true，则pointer()返回&m.header.frame_id，否则返回NULL。value()不存在，会导致编译错误
 */
template<typename M, typename Enable = void>
struct FrameId
{
  static std::string* pointer(M& m) { (void)m; return 0; }
  static std::string const* pointer(const M& m) { (void)m; return 0; }
};

template<typename M>
struct FrameId<M, typename boost::enable_if<HasHeader<M> >::type >
{
  static std::string* pointer(M& m) { return &m.header.frame_id; }
  static std::string const* pointer(const M& m) { return &m.header.frame_id; }
  static std::string value(const M& m) { return m.header.frame_id; }
};

/**
 @brief 时间戳特性。在默认实现中，如果HasHeader<M>::value为true，
        则pointer()返回&m.header.stamp，否则返回NULL。
        value()不存在，会导致编译错误
 */
template<typename M, typename Enable = void>
struct TimeStamp
{
  static ros::Time* pointer(M& m) { (void)m; return 0; }
  static ros::Time const* pointer(const M& m) { (void)m; return 0; }
};

template<typename M>
struct TimeStamp<M, typename boost::enable_if<HasHeader<M> >::type >
{
  static ros::Time* pointer(typename boost::remove_const<M>::type &m) { return &m.header.stamp; }
  static ros::Time const* pointer(const M& m) { return &m.header.stamp; }
  static ros::Time value(const M& m) { return m.header.stamp; }
};

/**
 @brief 返回MD5Sum<M>::value();
 */
template<typename M>
inline const char* md5sum()
{
  return MD5Sum<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

/**
 @brief 返回DataType<M>::value();
 */
template<typename M>
inline const char* datatype()
{
  return DataType<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

/**
 @brief 返回Definition<M>::value();
 */
template<typename M>
inline const char* definition()
{
  return Definition<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value();
}

/**
 @brief 返回MD5Sum<M>::value(m);
 */
template<typename M>
inline const char* md5sum(const M& m)
{
  return MD5Sum<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

/**
 @brief 返回DataType<M>::value(m);
 */
template<typename M>
inline const char* datatype(const M& m)
{
  return DataType<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

/**
 @brief 返回Definition<M>::value(m);
 */
template<typename M>
inline const char* definition(const M& m)
{
  return Definition<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value(m);
}

/**
 @brief 返回Header<M>::pointer(m);
 */
template<typename M>
inline std_msgs::Header* header(M& m)
{
  return Header<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

/**
 @brief 返回Header<M>::pointer(m);
 */
template<typename M>
inline std_msgs::Header const* header(const M& m)
{
  return Header<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

/**
 @brief 返回FrameId<M>::pointer(m);
 */
template<typename M>
inline std::string* frameId(M& m)
{
  return FrameId<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

/**
 @brief 返回FrameId<M>::pointer(m);
 */
template<typename M>
inline std::string const* frameId(const M& m)
{
  return FrameId<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

/**
@brief 返回TimeStamp<M>::pointer(m);
 */
template<typename M>
inline ros::Time* timeStamp(M& m)
{
  return TimeStamp<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

/**
@brief 返回TimeStamp<M>::pointer(m);
 */
template<typename M>
inline ros::Time const* timeStamp(const M& m)
{
  return TimeStamp<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::pointer(m);
}

/**
 @brief 返回IsSimple<M>::value;
 */
template<typename M>
inline bool isSimple()
{
  return IsSimple<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value;
}

/**
 @brief 返回IsFixedSize<M>::value;
 */
template<typename M>
inline bool isFixedSize()
{
  return IsFixedSize<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value;
}

/**
 @brief 返回HasHeader<M>::value;
 */
template<typename M>
inline bool hasHeader()
{
  return HasHeader<typename boost::remove_reference<typename boost::remove_const<M>::type>::type>::value;
}

} // namespace message_traits
} // namespace ros

#endif // ROSLIB_MESSAGE_TRAITS_H
