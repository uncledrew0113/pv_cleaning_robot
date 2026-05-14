# NavService Fused Odometry Status

**Goal:** 在不改变现有 `NavService` 构造和原有公开接口命名的前提下，新增上下轮组/GPS/IMU 融合里程查询能力。

**Current implementation:**
- `NavService` 内部新增 `FusedOdometry` 精简状态。
- 融合主线为“上下轮组积分 + GPS 轨道投影低频校正 + IMU 陀螺/加速度短时约束”。
- 未引入 ROS、未引入新的运行时输入、未增加 provider 或样本注入入口。
- `Pose`、悬空检测、现有构造调用点保持保留。

**Validation path:**
- 构建验证：`cmake --build --preset rk3576-build --target unit_tests -j4`
- 构建验证：`cmake --build --preset rk3576-build --target hw_tests -j4`
- 真实硬件验证：`./hw_tests "[hw_system][nav_fused_odometry]"`

**Files:**
- Modify: `include/pv_cleaning_robot/service/nav_service.h`
- Modify: `pv_cleaning_robot/service/nav_service.cc`
- Modify: `test/integration/hardware/system_hw_test.cc`
