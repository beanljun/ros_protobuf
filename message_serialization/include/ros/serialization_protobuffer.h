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

#ifndef ROSCPP_SERIALIZATION_PROTOBUFFER_H
#define ROSCPP_SERIALIZATION_PROTOBUFFER_H

#include <google/protobuf/message.h>

#include "serialization.h"

namespace ros {
namespace serialization {
// protobuffer 

// Serializer类的模板特化，用于序列化和反序列化google::protobuf::Message类型的数据
// std::is_base_of 是 C++ 标准库 <type_traits> 
// 如果第一个类型是第二个类型的基类，value 将为 true，否则为 false
// std::is_base_of<::google::protobuf::Message， std::string>::value 为 false
// std::is_base_of<::google::protobuf::Message， superbai::sample::PublishInfo>::value 为 true
// 在sample_protos/publish_info.proto中定义了PublishInfo类，该类继承自::google::protobuf::Message
template <typename T>
struct Serializer<T, typename std::enable_if<std::is_base_of<
                         ::google::protobuf::Message, T>::value>::type> {
  template <typename Stream>
  inline static void write(Stream &stream, const T &t) {
    std::string pb_str;
    // t是google::protobuf::Message类型的数据，调用其SerializeToString方法将数据序列化为字符串
    // 二进制数据存到了pb_str中
    t.SerializeToString(&pb_str);
    // 4个字节
    uint32_t len = (uint32_t)pb_str.size();
    // 利用OStream类型的stream对象的next方法将len写入到流中
    stream.next(len);

    if (len > 0) {
      memcpy(stream.advance((uint32_t)len), pb_str.data(), len);
    }
    // std::cout << "pb_str" << std::endl;
    // stream.next(pb_str);
  }

  // ros反序列化的接口
  template <typename Stream>
  inline static void read(Stream &stream, T &t) {
    uint32_t len;
    // IStream
    stream.next(len);
    // std::cout << "len: " << len << std::endl;

    std::string pb_str;
    if (len > 0) {
      const char *data_ptr =
          reinterpret_cast<const char *>(stream.advance(len));

      pb_str = std::string(data_ptr, len);
    } else {
      pb_str.clear();
    }

    // stream.next(pb_str);
    t.ParseFromString(pb_str);
  }

  inline static uint32_t serializedLength(const T &t) {
    std::string pb_str;
    t.SerializeToString(&pb_str);
    return 4 + (uint32_t)pb_str.size();
  }
};

}  // namespace serialization
}  // namespace ros

#endif  // ROSCPP_SERIALIZATION_H
