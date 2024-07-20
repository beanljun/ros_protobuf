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

#ifndef ROSLIB_SERIALIZED_MESSAGE_H
#define ROSLIB_SERIALIZED_MESSAGE_H

#include "roscpp_serialization_macros.h"

#include <boost/shared_array.hpp>
#include <boost/shared_ptr.hpp>

namespace ros
{
/**
  @brief 用于处理ROS消息的序列化和反序列化的存储
  * 用于处理ROS消息的序列化和反序列化的存储，作为一个中转站，便于上层接口直接调用
  * 放在一个公用的接口里面，通过这个接口来存储序列化和反序列化的数据
  * 用户层，可以直接调用这个公用的接口去获取序列化和反序列化的数据
  @param buf            存储被序列化的消息的字节流---每一个字节存储8位
  @param num_bytes      存储字节流的长度 
  @param message_start  指向消息字节流的起始位置的指针
  @param message        用于存储反序列化后的消息
  @param type_info      消息的类型信息用于存储消息的类型信息
 */
class ROSCPP_SERIALIZATION_DECL SerializedMessage
{
public:
  /// 存储字节流  序列化
  boost::shared_array<uint8_t> buf;
  size_t num_bytes;
  uint8_t* message_start;

  /// 反序列化后的对象
  boost::shared_ptr<void const> message;
  /// 对象的类型，元数据
  const std::type_info* type_info;

  SerializedMessage()
  : buf(boost::shared_array<uint8_t>())
  , num_bytes(0)
  , message_start(0)
  , type_info(0)
  {}

  SerializedMessage(boost::shared_array<uint8_t> buf, size_t num_bytes)
  : buf(buf)
  , num_bytes(num_bytes)
  , message_start(buf ? buf.get() : 0)
  , type_info(0)
  { } 
};

} // namespace ros

#endif // ROSLIB_SERIALIZED_MESSAGE_H
