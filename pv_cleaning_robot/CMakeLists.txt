# ── 收集全部子目录源文件 ────────────────────────────────────────────────
file(GLOB_RECURSE ROBOT_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/driver/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/protocol/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/device/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/middleware/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/service/*.cc"
    "${CMAKE_CURRENT_SOURCE_DIR}/app/*.cc"
)

# ── 主可执行文件 ─────────────────────────────────────────────────────────
add_executable(${PROJECT_NAME}
    main.cc
    ${ROBOT_SOURCES}
)

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${PROJECT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${MODBUS_INCLUDE_DIRS}
        ${SQLITE3_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        spdlog::spdlog
        Threads::Threads
        robot_3rdparty
        ssl
        crypto
)
