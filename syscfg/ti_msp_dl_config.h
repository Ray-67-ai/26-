/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_RIGHT */
#define PWM_RIGHT_INST                                                     TIMA0
#define PWM_RIGHT_INST_IRQHandler                               TIMA0_IRQHandler
#define PWM_RIGHT_INST_INT_IRQN                                 (TIMA0_INT_IRQn)
#define PWM_RIGHT_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_RIGHT_C0_PORT                                             GPIOB
#define GPIO_PWM_RIGHT_C0_PIN                                     DL_GPIO_PIN_14
#define GPIO_PWM_RIGHT_C0_IOMUX                                  (IOMUX_PINCM31)
#define GPIO_PWM_RIGHT_C0_IOMUX_FUNC                 IOMUX_PINCM31_PF_TIMA0_CCP0
#define GPIO_PWM_RIGHT_C0_IDX                                DL_TIMER_CC_0_INDEX

/* Defines for PWM_LEFT */
#define PWM_LEFT_INST                                                      TIMG8
#define PWM_LEFT_INST_IRQHandler                                TIMG8_IRQHandler
#define PWM_LEFT_INST_INT_IRQN                                  (TIMG8_INT_IRQn)
#define PWM_LEFT_INST_CLK_FREQ                                          32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_LEFT_C0_PORT                                              GPIOA
#define GPIO_PWM_LEFT_C0_PIN                                       DL_GPIO_PIN_7
#define GPIO_PWM_LEFT_C0_IOMUX                                   (IOMUX_PINCM14)
#define GPIO_PWM_LEFT_C0_IOMUX_FUNC                  IOMUX_PINCM14_PF_TIMG8_CCP0
#define GPIO_PWM_LEFT_C0_IDX                                 DL_TIMER_CC_0_INDEX



/* Defines for CONTROL_TIMER */
#define CONTROL_TIMER_INST                                               (TIMA1)
#define CONTROL_TIMER_INST_IRQHandler                           TIMA1_IRQHandler
#define CONTROL_TIMER_INST_INT_IRQN                             (TIMA1_INT_IRQn)
#define CONTROL_TIMER_INST_LOAD_VALUE                                   (31999U)




/* Defines for I2C_DISPLAY */
#define I2C_DISPLAY_INST                                                    I2C0
#define I2C_DISPLAY_INST_IRQHandler                              I2C0_IRQHandler
#define I2C_DISPLAY_INST_INT_IRQN                                  I2C0_INT_IRQn
#define I2C_DISPLAY_BUS_SPEED_HZ                                          400000
#define GPIO_I2C_DISPLAY_SDA_PORT                                          GPIOA
#define GPIO_I2C_DISPLAY_SDA_PIN                                   DL_GPIO_PIN_0
#define GPIO_I2C_DISPLAY_IOMUX_SDA                                (IOMUX_PINCM1)
#define GPIO_I2C_DISPLAY_IOMUX_SDA_FUNC                 IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_DISPLAY_SCL_PORT                                          GPIOA
#define GPIO_I2C_DISPLAY_SCL_PIN                                   DL_GPIO_PIN_1
#define GPIO_I2C_DISPLAY_IOMUX_SCL                                (IOMUX_PINCM2)
#define GPIO_I2C_DISPLAY_IOMUX_SCL_FUNC                 IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for DEBUG_UART */
#define DEBUG_UART_INST                                                    UART0
#define DEBUG_UART_INST_FREQUENCY                                        4000000
#define DEBUG_UART_INST_IRQHandler                              UART0_IRQHandler
#define DEBUG_UART_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_DEBUG_UART_RX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_TX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_DEBUG_UART_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_DEBUG_UART_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_DEBUG_UART_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_DEBUG_UART_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_DEBUG_UART_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define DEBUG_UART_BAUD_RATE                                            (115200)
#define DEBUG_UART_IBRD_4_MHZ_115200_BAUD                                    (2)
#define DEBUG_UART_FBRD_4_MHZ_115200_BAUD                                   (11)





/* Port definition for Pin Group MOTOR_DIR */
#define MOTOR_DIR_PORT                                                   (GPIOB)

/* Defines for AIN1: GPIOB.9 with pinCMx 26 on package pin 61 */
#define MOTOR_DIR_AIN1_PIN                                       (DL_GPIO_PIN_9)
#define MOTOR_DIR_AIN1_IOMUX                                     (IOMUX_PINCM26)
/* Defines for AIN2: GPIOB.10 with pinCMx 27 on package pin 62 */
#define MOTOR_DIR_AIN2_PIN                                      (DL_GPIO_PIN_10)
#define MOTOR_DIR_AIN2_IOMUX                                     (IOMUX_PINCM27)
/* Defines for BIN1: GPIOB.7 with pinCMx 24 on package pin 59 */
#define MOTOR_DIR_BIN1_PIN                                       (DL_GPIO_PIN_7)
#define MOTOR_DIR_BIN1_IOMUX                                     (IOMUX_PINCM24)
/* Defines for BIN2: GPIOB.6 with pinCMx 23 on package pin 58 */
#define MOTOR_DIR_BIN2_PIN                                       (DL_GPIO_PIN_6)
#define MOTOR_DIR_BIN2_IOMUX                                     (IOMUX_PINCM23)
/* Port definition for Pin Group ENCODER */
#define ENCODER_PORT                                                     (GPIOB)

/* Defines for RIGHT_A: GPIOB.11 with pinCMx 28 on package pin 63 */
// pins affected by this interrupt request:["RIGHT_A","LEFT_A"]
#define ENCODER_INT_IRQN                                        (GPIOB_INT_IRQn)
#define ENCODER_INT_IIDX                        (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODER_RIGHT_A_IIDX                                (DL_GPIO_IIDX_DIO11)
#define ENCODER_RIGHT_A_PIN                                     (DL_GPIO_PIN_11)
#define ENCODER_RIGHT_A_IOMUX                                    (IOMUX_PINCM28)
/* Defines for RIGHT_B: GPIOB.12 with pinCMx 29 on package pin 64 */
#define ENCODER_RIGHT_B_PIN                                     (DL_GPIO_PIN_12)
#define ENCODER_RIGHT_B_IOMUX                                    (IOMUX_PINCM29)
/* Defines for LEFT_A: GPIOB.4 with pinCMx 17 on package pin 52 */
#define ENCODER_LEFT_A_IIDX                                  (DL_GPIO_IIDX_DIO4)
#define ENCODER_LEFT_A_PIN                                       (DL_GPIO_PIN_4)
#define ENCODER_LEFT_A_IOMUX                                     (IOMUX_PINCM17)
/* Defines for LEFT_B: GPIOB.5 with pinCMx 18 on package pin 53 */
#define ENCODER_LEFT_B_PIN                                       (DL_GPIO_PIN_5)
#define ENCODER_LEFT_B_IOMUX                                     (IOMUX_PINCM18)
/* Defines for OUT1: GPIOB.19 with pinCMx 45 on package pin 16 */
#define LINE_OUT1_PORT                                                   (GPIOB)
#define LINE_OUT1_PIN                                           (DL_GPIO_PIN_19)
#define LINE_OUT1_IOMUX                                          (IOMUX_PINCM45)
/* Defines for OUT2: GPIOB.17 with pinCMx 43 on package pin 14 */
#define LINE_OUT2_PORT                                                   (GPIOB)
#define LINE_OUT2_PIN                                           (DL_GPIO_PIN_17)
#define LINE_OUT2_IOMUX                                          (IOMUX_PINCM43)
/* Defines for OUT3: GPIOA.16 with pinCMx 38 on package pin 9 */
#define LINE_OUT3_PORT                                                   (GPIOA)
#define LINE_OUT3_PIN                                           (DL_GPIO_PIN_16)
#define LINE_OUT3_IOMUX                                          (IOMUX_PINCM38)
/* Defines for OUT4: GPIOA.14 with pinCMx 36 on package pin 7 */
#define LINE_OUT4_PORT                                                   (GPIOA)
#define LINE_OUT4_PIN                                           (DL_GPIO_PIN_14)
#define LINE_OUT4_IOMUX                                          (IOMUX_PINCM36)
/* Defines for OUT5: GPIOB.20 with pinCMx 48 on package pin 19 */
#define LINE_OUT5_PORT                                                   (GPIOB)
#define LINE_OUT5_PIN                                           (DL_GPIO_PIN_20)
#define LINE_OUT5_IOMUX                                          (IOMUX_PINCM48)
/* Defines for OUT6: GPIOB.25 with pinCMx 56 on package pin 27 */
#define LINE_OUT6_PORT                                                   (GPIOB)
#define LINE_OUT6_PIN                                           (DL_GPIO_PIN_25)
#define LINE_OUT6_IOMUX                                          (IOMUX_PINCM56)
/* Defines for OUT7: GPIOA.25 with pinCMx 55 on package pin 26 */
#define LINE_OUT7_PORT                                                   (GPIOA)
#define LINE_OUT7_PIN                                           (DL_GPIO_PIN_25)
#define LINE_OUT7_IOMUX                                          (IOMUX_PINCM55)
/* Defines for OUT8: GPIOA.27 with pinCMx 60 on package pin 31 */
#define LINE_OUT8_PORT                                                   (GPIOA)
#define LINE_OUT8_PIN                                           (DL_GPIO_PIN_27)
#define LINE_OUT8_IOMUX                                          (IOMUX_PINCM60)
/* Port definition for Pin Group USER_KEY */
#define USER_KEY_PORT                                                    (GPIOA)

/* Defines for START: GPIOA.23 with pinCMx 53 on package pin 24 */
// pins affected by this interrupt request:["START"]
#define USER_KEY_INT_IRQN                                       (GPIOA_INT_IRQn)
#define USER_KEY_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define USER_KEY_START_IIDX                                 (DL_GPIO_IIDX_DIO23)
#define USER_KEY_START_PIN                                      (DL_GPIO_PIN_23)
#define USER_KEY_START_IOMUX                                     (IOMUX_PINCM53)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_RIGHT_init(void);
void SYSCFG_DL_PWM_LEFT_init(void);
void SYSCFG_DL_CONTROL_TIMER_init(void);
void SYSCFG_DL_I2C_DISPLAY_init(void);
void SYSCFG_DL_DEBUG_UART_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
