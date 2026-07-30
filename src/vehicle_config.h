#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/*
 * H题第二问车辆参数集中配置
 *
 * 重要：拓展板把编码器 VCC 接到了 5 V，A/B 又直接进入 MSPM0G3507
 * 的 PB4/PB5/PB11/PB12。上述 PB 引脚不是 5 V 容忍引脚。
 * 请先把两个编码器 VCC 改接 3.3 V（霍尔编码器允许 3.3~5 V），
 * 或安装可靠电平转换，再把下面的开关改为 1。
 */
#define CFG_ENCODER_INPUT_LEVEL_SAFE       (1)

/*
 * 机械与编码器：MG513XP28_12V，霍尔 13 PPR，减速比1:28。
 * 只对A相开中断，但同时统计上升沿和下降沿，所以每轮为13×28×2=728。
 */
#define CFG_ENCODER_PPR_MOTOR              (13.0f)
#define CFG_GEAR_RATIO                     (28.0f)
#define CFG_COUNTS_PER_WHEEL_REV           (728.0f)
#define CFG_WHEEL_DIAMETER_MM              (65.0f)
#define CFG_PI                             (3.1415926f)

/*
 * 首次必须架空车轮做方向测试。
 * 1 = IN1高、IN2低时该侧车轮使车辆前进；不对就只改对应宏。
 */
#define CFG_RIGHT_FORWARD_IN1_HIGH         (1)
#define CFG_LEFT_FORWARD_IN1_HIGH          (1)

/*
 * A相双边沿计数，以A、B相当前电平判断方向。
 * 1 = 车辆前进时A与B电平相同；速度显示为负就翻转对应宏。
 */
#define CFG_RIGHT_FORWARD_WHEN_A_EQUALS_B  (1)
#define CFG_LEFT_FORWARD_WHEN_A_EQUALS_B   (0)

/*
 * PWM：SysConfig中使用EDGE_ALIGN_UP、周期2000。
 * 该向上计数模式下compare约等于duty×period：0接近0%，2000为100%。
 * 注意：SDK的EDGE_ALIGN向下计数模式与此关系相反，不能混用公式。
 */
#define CFG_PWM_PERIOD_TICKS               (2000U)
#define CFG_PWM_START_DUTY                 (0.20f) //起步占空比
#define CFG_PWM_MAX_DUTY                   (0.90f) //最大占空比

/* 速度环：100 Hz，单位为车轮 rpm。先架空验证，再在地面细调。 */
#define CFG_SPEED_CONTROL_DT_S             (0.010f)
#define CFG_SPEED_KP                       (0.0026f)
#define CFG_SPEED_KI                       (0.0200f)
#define CFG_SPEED_NORMAL_RPM               (200.0f) //正常速度
#define CFG_SPEED_FINISH_RPM               (70.0f)//终点速度
#define CFG_SPEED_MAX_RPM                  (280.0f)//最大速度
#define CFG_SPEED_STEER_LIMIT_RPM          (70.0f)//

/* 巡线：X1/OUT1 位于车辆左侧，黑线为低电平。 */
#define CFG_LINE_BLACK_IS_LOW              (1)
#define CFG_LINE_KP_RPM                    (0.0250f)
#define CFG_LINE_KD_RPM                    (0.00060f)
#define CFG_LINE_LOST_TIMEOUT_MS           (350U)

/* 8路探头要求上电稳定；标定参数已经保存在巡线模块中。 */
#define CFG_SENSOR_WARMUP_MS               (500U)

/*
 * 一圈与终点判定完全使用编码器，不使用ICM-45686。
 * 赛道中心线理论总长约6142 mm。5.40 m开始减速，5.70 m后才允许
 * “至少4路同时检测到黑色”成为终点，留出尺寸/打滑误差。
 */
#define CFG_TRACK_LENGTH_MM                (6142.0f)
#define CFG_FINISH_SLOWDOWN_DISTANCE_MM    (5400.0f)
#define CFG_FINISH_ARM_DISTANCE_MM         (5700.0f)
#define CFG_FINISH_MIN_TIME_MS             (7000U)
#define CFG_FINISH_WIDE_CHANNELS           (4U)//检测到5路停车
#define CFG_FINISH_DEBOUNCE_MS              (20U)
#define CFG_START_LINE_CLEAR_MS            (100U)
#define CFG_ACTIVE_BRAKE_MS                (120U)
#define CFG_RUN_FAILSAFE_MS                (25000U)

/* OLED 自动尝试常见 0x3C、0x3D 地址。 */
#define CFG_OLED_ADDRESS_PRIMARY           (0x3CU)
#define CFG_OLED_ADDRESS_SECONDARY         (0x3DU)

#endif
