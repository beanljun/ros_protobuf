

## 0. 安装docker

推荐使用鱼香ROS的docker安装脚本。
    
```bash
 wget http://fishros.com/install -O fishros && . fishros
```

## 1.通过项目中dockerfile文件，构建项目镜像 

```bash
cd ros_protobuf_msg/docker/build
docker build --network host -t ros_proto:20.04  -f ros_proto.dockerfile .
```

## 2.进入docker容器

```bash
cd ros_protobuf_msg/docker/scripts
#启动容器
./ros_docker_run.sh
#进入容器
./ros_docker_into.sh
```

## 3.编译代码

```bash
#创建build目录
mkdir build
cd build
cmake ..
make -j6
```

## 4.启动程序

```bash
#先启动roscore，并且启动pb_talker节点
cd /work
source devel/setup.bash
roscore &
rosrun myproject pb_talker
```

```bash
#打开新终端，再次进入容器，启动pb_listener节点
#进入容器中
cd ros_protobuf_msg/docker/scripts
#进入容器
./ros_docker_into.sh
rosrun myproject pb_listener
```