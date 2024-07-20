#!/usr/bin/env bash # 使用bash解释器执行此脚本

# 计算并设置MONITOR_HOME_DIR变量为脚本所在目录的上两级目录的绝对路径
MONITOR_HOME_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/../.." && pwd )"

# 初始化display变量
display=""
# 如果环境变量DISPLAY未设置，则将display设置为":1"，否则使用环境变量DISPLAY的值
if [ -z ${DISPLAY} ];then
    display=":1"
else
    display="${DISPLAY}"
fi

# 获取并设置本机的主机名
local_host="$(hostname)"
# 获取并设置当前用户的用户名
user="${USER}"
# 获取并设置当前用户的用户ID
uid="$(id -u)"
# 获取并设置当前用户的组名
group="$(id -g -n)"
# 获取并设置当前用户的组ID
gid="$(id -g)"

# 停止并删除名为ros_noetic_proto的docker容器，输出重定向到/dev/null，即不显示任何输出
echo "stop and rm docker" 
docker stop ros_noetic_proto > /dev/null
docker rm -v -f ros_noetic_proto > /dev/null

# 输出开始docker的信息
echo "start docker"
# 使用docker run命令以交互模式启动一个名为ros_noetic_proto的容器，并设置相关环境变量和挂载点
# -it 交互模式 -d 后台运行

docker run -it -d \
--privileged=true \
--name ros_noetic_proto \
-e DISPLAY=$display \
-e DOCKER_USER="${user}" \
-e USER="${user}" \
-e DOCKER_USER_ID="${uid}" \
-e DOCKER_GRP="${group}" \
-e DOCKER_GRP_ID="${gid}" \
-e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
-v ${MONITOR_HOME_DIR}:/work \
-v ${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR} \
--network host \
ros_protobuf:noetic

# --privileged=true \                           # 容器拥有额外的权限
# --name ros_noetic_proto \                     # 设置容器的名称
# -e DISPLAY=$display \                         # 设置DISPLAY环境变量，用于图形界面的显示
# -e DOCKER_USER="${user}" \                    # 设置容器内DOCKER_USER环境变量为当前用户
# -e USER="${user}" \                           # 设置容器内USER环境变量为当前用户
# -e DOCKER_USER_ID="${uid}" \                  # 设置容器内DOCKER_USER_ID环境变量为当前用户ID
# -e DOCKER_GRP="${group}" \                    # 设置容器内DOCKER_GRP环境变量为当前用户组
# -e DOCKER_GRP_ID="${gid}" \                   # 设置容器内DOCKER_GRP_ID环境变量为当前用户组ID
# -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \         # 设置容器内XDG_RUNTIME_DIR环境变量，通常用于存放运行时数据
# -v ${MONITOR_HOME_DIR}:/work \                # 将MONITOR_HOME_DIR目录挂载到容器的/work目录
# -v ${XDG_RUNTIME_DIR}:${XDG_RUNTIME_DIR} \    # 将XDG_RUNTIME_DIR目录挂载到容器内相同路径
# --network host \                              # 容器使用宿主机的网络
# ros_protobuf:noetic                           # 使用的镜像名和标签