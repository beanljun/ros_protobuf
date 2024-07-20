#!/usr/bin/env bash # 使用bash解释器执行此脚本

# 允许本地的root用户访问X服务器，输出重定向到/dev/null，即忽略所有输出
xhost +local:root 1>/dev/null 2>&1

# 以root用户身份在名为ros_noetic_proto的Docker容器中启动一个交互式的bash会话
docker exec \
    -u root \
    -it ros_noetic_proto \ 
    /bin/bash 

# 在完成操作后，撤销本地root用户访问X服务器的权限，同样忽略所有输出
xhost -local:root 1>/dev/null 2>&1