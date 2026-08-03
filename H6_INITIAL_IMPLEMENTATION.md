# H6 第六问初始实现

更新时间：2026-08-03（Asia/Shanghai）

## 实现目标

小车位于A点、钢球放在摆杆任意指定位置。按Q3后，程序自动记录该位置作为本轮唯一目标，随后沿黑线顺时针运行一圈并通过A点。行驶过程中控制的是钢球相对启动目标的误差，不改变MaixCAM原有坐标系。

## 启动状态机

1. `WAIT_VISION/READY`：等待有效视觉数据和Q3。
2. `CAPTURE_TARGET`：小车不动、水管回到机械零角度；连续采集静止小球位置。
3. 捕获条件：小球速度不超过8 mm/s、窗口内位置跨度不超过3 mm、持续300 ms且至少5个有效样本。
4. 捕获完成后取样本平均值并永久锁定为本轮 `target_mm`。
5. `HOLD_TARGET`：闭合球位置控制；相对误差在±3 mm、速度不超过8 mm/s连续300 ms后启动小车。
6. `ACCEL/CRUISE/DECEL`：完整复用H5实车验证过的整圈行驶过程。

## 相对误差

```c
relative_mm = vision_position_mm - captured_target_mm;
predicted_mm = relative_mm + prediction_time_s * velocity_mm_s;
```

PID、弯道阻尼、静摩擦短脉冲和超差判断均使用相对误差。静摩擦脉冲的“正在回到目标”判断也改为 `relative_mm * velocity_mm_s < 0`，避免任意目标不在视觉零点时方向判断错误。

## 初始复用参数

- Kp=0.075 deg/mm，Ki=0，Kd=0.045 deg/(mm/s)
- 预测时间20 ms
- 巡航140 rpm，平滑加速2700 ms
- 普通角度上限4°，大误差/启动上限7°
- 角度变化率140 deg/s
- 巡航防卡脉冲5°
- 两个弯道阻尼窗口、第二次A点检测、A点后减速与H5完全一致

## 文件

- `src/h6_balance_any.c`：H6状态机、任意目标捕获和整圈控制
- `src/h6_balance_any.h`：模块接口
- `src/h6_config.h`：捕获条件和H5参数映射
- `tools/h6_rtt_control.py`：后续J-Link RTT实车记录与相对误差统计

## 当前验证边界

- 正式合并固件与H6专用调试固件均已通过编译和链接。
- 当前未连接J-Link、未烧录、未让车辆运动。
- 实车阶段至少应测试目标约-50 mm、0 mm、+50 mm，并重复验证起步、弧顶和出弯最大相对误差。
