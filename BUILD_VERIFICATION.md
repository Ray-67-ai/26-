# 构建验证记录

验证日期：2026-07-29

## 工具版本

- Device：MSPM0G3507，LQFP-64
- MSPM0 SDK：2.10.00.04
- SysConfig CLI：1.27.1+4634
- TI Arm Clang：4.0.4.LTS
- 编译目标：Cortex-M0+、Thumb、soft-float、little-endian、`-O2`

## 结果

- SysConfig：0 errors，0 warnings。
- SysConfig提示：PWM_RIGHT和CONTROL_TIMER寄存器在STOP/STANDBY不保持。本工程
  只使用正常运行和WFI，不进入STOP/STANDBY，因此不影响当前流程。
- C源码：全部通过 `-Wall -Wextra` 编译，无警告。
- 链接：成功。
- Flash文本：11656字节。
- 初始化数据：0字节。
- BSS RAM：2044字节。
- 总计：13700字节。

预编译文件：

- `prebuilt_safe_locked/H2_LineCar_CCS.out`
- `prebuilt_safe_locked/H2_LineCar_CCS.map`

`.out` SHA-256：

```text
118B0A969395FAB5FD3F027F60A4717840D73A847F472FDB98D9DE34627546A5
```

## 预编译文件的限制

该预编译固件保持：

```c
#define CFG_ENCODER_INPUT_LEVEL_SAFE (0)
```

因此它只用于证明工程可编译、显示硬件安全提示，不会启动电机。把编码器供电改成
3.3V并将宏改为1以后，需要在CCS中重新编译，不能继续烧录此预编译文件。

本次未连接车辆，未验证电机方向、编码器正负、巡线电平、PID参数、实际一圈时间
或停车精度。

