# ==============================================================================
# 1. 纯头文件库配置 (Header-only)
# ==============================================================================
# 声明一个名为 3rdparty_headers 的接口目标
add_library(3rdparty_headers INTERFACE)
target_include_directories(3rdparty_headers INTERFACE 
    ${CMAKE_CURRENT_SOURCE_DIR}/header_only
)

# ==============================================================================
# 2. 源码级集成库 (Source Drop-in)
# ==============================================================================
# # 将 SQLite3 编译成一个静态库，供主控程序链接
# add_library(sqlite3_static STATIC 
#     ${CMAKE_CURRENT_SOURCE_DIR}/2_source_dropin/sqlite3/sqlite3.c
# )
# target_include_directories(sqlite3_static PUBLIC 
#     ${CMAKE_CURRENT_SOURCE_DIR}/2_source_dropin/sqlite3
# )
# # 嵌入式通常需要链接多线程库和动态加载库
# target_link_libraries(sqlite3_static pthread dl)

# ==============================================================================
# 2. 串口通信库 (libserialport)
# ==============================================================================
add_library(serialport STATIC IMPORTED)
set_target_properties(serialport PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/libserialport/lib/libserialport.a
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/libserialport/include
)

# ==============================================================================
# 3. 防跌落 GPIO 底层驱动 (libgpiod v1.6.3)
# ==============================================================================
# (1) 先导入 C 底层核心库
add_library(gpiod_c STATIC IMPORTED)
set_target_properties(gpiod_c PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/libgpiod/lib/libgpiod.a
)

# (2) 导入 C++ 封装库，并声明它强制依赖 C 底层库
add_library(gpiodcxx STATIC IMPORTED)
set_target_properties(gpiodcxx PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/libgpiod/lib/libgpiodcxx.a
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/libgpiod/include
)
# 【魔法点】: 告诉 CMake，谁链接了 gpiodcxx，就必须连带链接 gpiod_c
set_property(TARGET gpiodcxx APPEND PROPERTY INTERFACE_LINK_LIBRARIES gpiod_c)


# ==============================================================================
# 4. 4G 云端通信协议栈 (Paho MQTT Async + SSL)
# ==============================================================================
# (1) 导入 C 核心异步 SSL 库
add_library(paho-mqtt3as STATIC IMPORTED)
set_target_properties(paho-mqtt3as PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/paho_mqtt/lib/libpaho-mqtt3as.a
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/paho_mqtt/include
)
# 【魔法点】: 静态链接 SSL 版的 MQTT，必须向系统底层索要 OpenSSL 加解密库、线程库和动态加载库
set_property(TARGET paho-mqtt3as APPEND PROPERTY INTERFACE_LINK_LIBRARIES pthread ssl crypto dl)

# (2) 导入 C++ 封装库，并声明它强制依赖 C 核心库
add_library(paho-mqttpp3 STATIC IMPORTED)
set_target_properties(paho-mqttpp3 PROPERTIES
    IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/paho_mqtt/lib/libpaho-mqttpp3.a
    INTERFACE_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_SOURCE_DIR}/precompiled/paho_mqtt/include
)
# 依赖传递
set_property(TARGET paho-mqttpp3 APPEND PROPERTY INTERFACE_LINK_LIBRARIES paho-mqtt3as)

# ==============================================================================
# 5. 终极超级目标 (供主工程一键链接)
# ==============================================================================
add_library(robot_3rdparty INTERFACE)

# 把所有打好包的组件全部挂载到这个超级接口上
target_link_libraries(robot_3rdparty INTERFACE
    3rdparty_headers
    serialport
    gpiodcxx
    paho-mqttpp3
    ${MODBUS_LIBRARIES}
    ${SQLITE3_LIBRARIES}
)
target_include_directories(robot_3rdparty INTERFACE
    ${MODBUS_INCLUDE_DIRS}
    ${SQLITE3_INCLUDE_DIRS}
)
