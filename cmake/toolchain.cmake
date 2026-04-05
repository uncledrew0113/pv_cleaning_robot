# toolchain.cmake

# 定义目标系统名称和处理器架构
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

# 定义 Sysroot 路径
# 优先从环境变量 RK3576_SDK_PATH 读取，回退到默认路径
if(DEFINED ENV{RK3576_SDK_PATH})
  set(CMAKE_SYSROOT "$ENV{RK3576_SDK_PATH}/sysroots/armv8a-linux")
else()
  set(CMAKE_SYSROOT "/home/tronlong/RK3576/sysroots/armv8a-linux")
  message(WARNING "RK3576_SDK_PATH not set; using fallback sysroot ${CMAKE_SYSROOT}")
endif()

# 指定交叉编译器 (因为后面我们会让 VS Code source 环境变量，所以直接写命令即可，系统能通过 PATH 找到)
set(CMAKE_C_COMPILER "aarch64-linux-gnu-gcc")
set(CMAKE_CXX_COMPILER "aarch64-linux-gnu-g++")

# 指定静态库归档工具和其他工具 (可选，通常 CMake 能自动推断)
set(CMAKE_AR "aarch64-linux-gnu-ar")
set(CMAKE_OBJCOPY "aarch64-linux-gnu-objcopy")
set(CMAKE_NM "aarch64-linux-gnu-nm")
set(CMAKE_STRIP "aarch64-linux-gnu-strip")

# 调整 CMake 的查找策略 (防止交叉编译时链接到宿主机 x86 的库)
# 从不查找宿主机的程序
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
# 只在 Sysroot 查找库文件、头文件和包
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)