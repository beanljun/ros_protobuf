cmake_minimum_required(VERSION 3.10)
project(test)

include_directories(
  include
)

add_subdirectory(proto)

add_executable(test_temple src/test_template.cc)

add_executable(test_sfinae src/sfinae_test.cc)

add_executable(test_proto  src/test_proto.cc)

add_executable(test_epoll src/test_epoll.cc)

target_link_libraries(test_proto 
proto_test
)