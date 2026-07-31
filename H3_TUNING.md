# H题第三问调试说明

## 代码分区

- `src/vehicle_config.h`、`src/app.c`：第二问，第三问不修改这里的参数。
- `src/h3_config.h`：第三问全部可调参数。
- `src/h3_vision.c`：接收 MaixCAM 坐标并由相邻帧计算速度。
- `src/zdt_stepper.c`：ZDT Emm TTL 普通绝对位置命令。
- `src/h3_ball_control.c`：第三问状态机和球位置外环。
- `src/competition_mode.h`：第二问/第三问编译运行切换。
- `maixcam/h3_ball_sender.py`：MaixCAM Pro 识别和发送坐标。

## 串口帧

MaixCAM 每个有效图像帧发送一行：

```text
B,sequence,camera_ms,position_mm,valid\n
```

例如钢球位于 +4.8 cm：

```text
B,135,4821,48,1
```

识别失败时仍发送一帧，但 `valid=0`：

```text
B,136,4854,0,0
```

## 第一次上电调试顺序

1. 不放钢球，断开连杆，只验证 ZDT 能响应使能和 0 度命令。
2. 接连杆但不放球，将 `H3_MAX_MOTOR_ANGLE_DEG` 临时改为 `1.0f`，确认正角度确实让右端升高。
3. MaixCAM 单独运行，先调整 `BALL_ROI`、两个管端像素坐标和 `BALL_THRESHOLDS`，确保 OLED 的 X 坐标方向、数值正确。
4. 水管断电调平，球放 O 点，上电后确认电机显示 0 度。
5. 把 `H3_BALL_KI_DEG_PER_MM_S` 暂时改为 `0.0f`，只调 Kp：能从 O 点平稳到 +5 cm。
6. 再增加 Kd，让折返和 -5 cm 刹车不过冲。
7. 最后恢复很小的 Ki，只消除 -5 cm 附近的静差。
8. 确认稳定后再逐步增大最大角度和响应速度，禁止一开始直接使用大角度。

## 运行顺序

1. 断电时人工把水管调平，钢球放 O 点。
2. 上电，OLED 出现 `READY`，X 坐标应在 ±15 mm 内。
3. 按启动键，程序自动执行 O -> +5 cm -> -5 cm并稳定。
4. `DONE` 表示在 -5 cm ±5 mm 且速度小于 25 mm/s 连续保持 400 ms。
5. 再按一次按键复位为 `READY`。

