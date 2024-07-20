#!/usr/bin/env bash

# 脚本执行出错时立即终止执行
set -e

# cd到脚本所在目录（容器内）${BASH_SOURCE[0]}为脚本的路径, $(dirname)为脚本所在目录 
# 比如脚本在/home/xxx/下，${BASH_SOURCE[0]}为/home/xxx/install_abseil.sh， $(dirname ${BASH_SOURCE[0]})为/home/xxx
cd "$(dirname "${BASH_SOURCE[0]}")"

#https://github.com/abseil/abseil-cpp/archive/refs/tags/20230802.0.tar.gz
# Install abseil.
# 获取当前机器的CPU核心数
THREAD_NUM=$(nproc)
# 设置abseil的版本号，与下载的文件名对应
VERSION="20230802.0"
# 设置abseil压缩包文件名
PKG_NAME="abseil-cpp-${VERSION}.tar.gz"

# 解压abseil压缩包
tar xzf "${PKG_NAME}"

# 进入abseil目录，和cd不同，pushd会将当前目录压入栈中，以便后续可以通过popd返回
pushd "abseil-cpp-${VERSION}"
    mkdir build && cd build
    cmake .. \
        -DBUILD_SHARED_LIBS=ON \
        -DCMAKE_CXX_STANDARD=14 \
        -DCMAKE_INSTALL_PREFIX=/usr/local
    make -j${THREAD_NUM}
    make install
popd

# 更新动态链接库
ldconfig

# Clean up
# 源代码和压缩包在make install后已经不再需要
rm -rf "abseil-cpp-${VERSION}" "${PKG_NAME}"
