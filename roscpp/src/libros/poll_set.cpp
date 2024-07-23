/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "ros/poll_set.h"
#include "ros/file_log.h"

#include "ros/transport/transport.h"

#include <ros/assert.h>

#include <boost/bind.hpp>

#include <fcntl.h>

namespace ros
{

PollSet::PollSet()
    : sockets_changed_(false), epfd_(create_socket_watcher())
{
	if ( create_signal_pair(signal_pipe_) != 0 ) {
        ROS_FATAL("create_signal_pair() failed");
    ROS_BREAK();
  }
  addSocket(signal_pipe_[0], boost::bind(&PollSet::onLocalPipeEvents, this, boost::placeholders::_1));
  addEvents(signal_pipe_[0], POLLIN);
}

PollSet::~PollSet()
{
  close_signal_pair(signal_pipe_);
  close_socket_watcher(epfd_);
}

bool PollSet::addSocket(int fd, const SocketUpdateFunc& update_func, const TransportPtr& transport)
{
  SocketInfo info;
  info.fd_ = fd;
  info.events_ = 0;
  info.transport_ = transport;
  info.func_ = update_func;

  {
    boost::mutex::scoped_lock lock(socket_info_mutex_);

    bool b = socket_info_.insert(std::make_pair(fd, info)).second;
    if (!b)
    {
      ROSCPP_LOG_DEBUG("PollSet: Tried to add duplicate fd [%d]", fd);
      return false;
    }

    add_socket_to_watcher(epfd_, fd);

    sockets_changed_ = true;
  }

  signal();

  return true;
}

bool PollSet::delSocket(int fd)
{
  if(fd < 0)
  {
    return false;
  }

  boost::mutex::scoped_lock lock(socket_info_mutex_);
  M_SocketInfo::iterator it = socket_info_.find(fd);
  if (it != socket_info_.end())
  {
    socket_info_.erase(it);

    {
      boost::mutex::scoped_lock lock(just_deleted_mutex_);
      just_deleted_.push_back(fd);
    }

    del_socket_from_watcher(epfd_, fd);

    sockets_changed_ = true;
    signal();

    return true;
  }

  ROSCPP_LOG_DEBUG("PollSet: Tried to delete fd [%d] which is not being tracked", fd);

  return false;
}


bool PollSet::addEvents(int sock, int events)
{
  boost::mutex::scoped_lock lock(socket_info_mutex_);

  M_SocketInfo::iterator it = socket_info_.find(sock);

  if (it == socket_info_.end())
  {
    ROSCPP_LOG_DEBUG("PollSet: Tried to add events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  it->second.events_ |= events;

  set_events_on_socket(epfd_, sock, it->second.events_);

  sockets_changed_ = true;
  signal();

  return true;
}

bool PollSet::delEvents(int sock, int events)
{
  boost::mutex::scoped_lock lock(socket_info_mutex_);

  M_SocketInfo::iterator it = socket_info_.find(sock);
  if (it != socket_info_.end())
  {
    it->second.events_ &= ~events;
  }
  else
  {
    ROSCPP_LOG_DEBUG("PollSet: Tried to delete events [%d] to fd [%d] which does not exist in this pollset", events, sock);
    return false;
  }

  set_events_on_socket(epfd_, sock, it->second.events_);

  sockets_changed_ = true;
  signal();

  return true;
}

void PollSet::signal()
{
  boost::mutex::scoped_try_lock lock(signal_mutex_);

  if (lock.owns_lock())
  {
    char b = 0;
    if (write_signal(signal_pipe_[1], &b, 1) < 0)
    {
      // do nothing... this prevents warnings on gcc 4.3
    }
  }
}


void PollSet::update(int poll_timeout)
{
  createNativePollset();

  // Poll across the sockets we're servicing
  // poll_sockets的底层就是epoll系统调用
  // 调用poll_sockets进行轮询：使用epoll系统调用通过poll_sockets函数轮询监视的套接字。poll_sockets返回一个包含有事件发生的套接字信息的向量。
  boost::shared_ptr<std::vector<socket_pollfd> > ofds = poll_sockets(epfd_, &ufds_.front(), ufds_.size(), poll_timeout);
  
  if (!ofds)
  { 
    // 如果poll_sockets返回空，表示轮询失败。如果失败不是因为EINTR（被信号中断），则记录错误。
    if (last_socket_error() != EINTR)
    {
      ROS_ERROR_STREAM("poll failed with error " << last_socket_error_string());
    }
  }
  else
  {
    // 遍历有事件发生的套接字列表
    for (std::vector<socket_pollfd>::iterator it = ofds->begin() ; it != ofds->end(); ++it)
    {
      int fd = it->fd;
      int revents = it->revents;
      SocketUpdateFunc func;
      TransportPtr transport;
      int events = 0;

      // 如果没有事件（revents == 0），则跳过
      if (revents == 0)
      {
        continue;
      }
      {
        // 使用socket_info_mutex_锁定套接字信息映射，查找当前套接字的信息。如果套接字已被删除，则跳过。
        boost::mutex::scoped_lock lock(socket_info_mutex_);
        M_SocketInfo::iterator it = socket_info_.find(fd);
        // the socket has been entirely deleted
        if (it == socket_info_.end())
        {
          continue;
        }

        const SocketInfo& info = it->second;

        // Store off the function and transport in case the socket is deleted from another thread
        // 从套接字信息中提取回调函数、传输对象和注册的事件。
        func = info.func_;
        transport = info.transport_;
        events = info.events_;
      }

      // If these are registered events for this socket, OR the events are ERR/HUP/NVAL,
      // call through to the registered function
      // 如果有为套接字注册的事件发生，或者发生了错误（POLLERR）、挂起（POLLHUP）或无效事件（POLLNVAL），则准备调用回调函数
      if (func
          && ((events & revents)
              || (revents & POLLERR)
              || (revents & POLLHUP)
              || (revents & POLLNVAL)))
      {
        bool skip = false;
        // 如果事件是POLLNVAL、POLLERR或POLLHUP之一
        if (revents & (POLLNVAL|POLLERR|POLLHUP))
        {
          // If a socket was just closed and then the file descriptor immediately reused, we can
          // get in here with what we think is a valid socket (since it was just re-added to our set)
          // but which is actually referring to the previous fd with the same #.  If this is the case,
          // we ignore the first instance of one of these errors.  If it's a real error we'll
          // hit it again next time through.
          boost::mutex::scoped_lock lock(just_deleted_mutex_);
          // 如果该套接字刚被关闭然后立即重新使用了相同的文件描述符，可能会错误地认为是有效的套接字。
          // 为了避免这种情况，如果这是第一次遇到这些错误之一，会跳过调用回调函数
          if (std::find(just_deleted_.begin(), just_deleted_.end(), fd) != just_deleted_.end())
          {
            skip = true;
          }
        }

        if (!skip)
        {
          // 如果不跳过，则调用回调函数，传入发生的事件。
          func(revents & (events|POLLERR|POLLHUP|POLLNVAL));
        }

        // func就是socketUpdate函数，也就是说之前socket的各种回调函数都将在epoll被唤醒时进行回调
      }
    }
  }

  boost::mutex::scoped_lock lock(just_deleted_mutex_);
  just_deleted_.clear();

}

void PollSet::createNativePollset()
{
  boost::mutex::scoped_lock lock(socket_info_mutex_);

  if (!sockets_changed_)
  {
    return;
  }

  // Build the list of structures to pass to poll for the sockets we're servicing
  ufds_.resize(socket_info_.size());
  M_SocketInfo::iterator sock_it = socket_info_.begin();
  M_SocketInfo::iterator sock_end = socket_info_.end();
  for (int i = 0; sock_it != sock_end; ++sock_it, ++i)
  {
    const SocketInfo& info = sock_it->second;
    socket_pollfd& pfd = ufds_[i];
    pfd.fd = info.fd_;
    pfd.events = info.events_;
    pfd.revents = 0;
  }
  sockets_changed_ = false;
}

void PollSet::onLocalPipeEvents(int events)
{
  if(events & POLLIN)
  {
    char b;
    while(read_signal(signal_pipe_[0], &b, 1) > 0)
    {
      //do nothing keep draining
    };
  }

}

}
