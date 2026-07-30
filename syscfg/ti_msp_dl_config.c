/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_RIGHTBackup;
DL_TimerA_backupConfig gCONTROL_TIMERBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_RIGHT_init();
    SYSCFG_DL_PWM_LEFT_init();
    SYSCFG_DL_CONTROL_TIMER_init();
    SYSCFG_DL_I2C_DISPLAY_init();
    SYSCFG_DL_DEBUG_UART_init();
    /* Ensure backup structures have no valid state */
	gPWM_RIGHTBackup.backupRdy 	= false;
	gCONTROL_TIMERBackup.backupRdy 	= false;


}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_RIGHT_INST, &gPWM_RIGHTBackup);
	retStatus &= DL_TimerA_saveConfiguration(CONTROL_TIMER_INST, &gCONTROL_TIMERBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_RIGHT_INST, &gPWM_RIGHTBackup, false);
	retStatus &= DL_TimerA_restoreConfiguration(CONTROL_TIMER_INST, &gCONTROL_TIMERBackup, false);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_RIGHT_INST);
    DL_TimerG_reset(PWM_LEFT_INST);
    DL_TimerA_reset(CONTROL_TIMER_INST);
    DL_I2C_reset(I2C_DISPLAY_INST);
    DL_UART_Main_reset(DEBUG_UART_INST);

    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_RIGHT_INST);
    DL_TimerG_enablePower(PWM_LEFT_INST);
    DL_TimerA_enablePower(CONTROL_TIMER_INST);
    DL_I2C_enablePower(I2C_DISPLAY_INST);
    DL_UART_Main_enablePower(DEBUG_UART_INST);
    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_RIGHT_C0_IOMUX,GPIO_PWM_RIGHT_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_RIGHT_C0_PORT, GPIO_PWM_RIGHT_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_LEFT_C0_IOMUX,GPIO_PWM_LEFT_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_LEFT_C0_PORT, GPIO_PWM_LEFT_C0_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_DISPLAY_IOMUX_SDA,
        GPIO_I2C_DISPLAY_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_I2C_DISPLAY_IOMUX_SCL,
        GPIO_I2C_DISPLAY_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_I2C_DISPLAY_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_I2C_DISPLAY_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_DEBUG_UART_IOMUX_TX, GPIO_DEBUG_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_DEBUG_UART_IOMUX_RX, GPIO_DEBUG_UART_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_AIN2_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(MOTOR_DIR_BIN2_IOMUX);

    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_RIGHT_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_A_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(ENCODER_LEFT_B_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT3_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT4_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT5_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT6_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT7_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(LINE_OUT8_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(USER_KEY_START_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_23_EDGE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOA, USER_KEY_START_PIN);
    DL_GPIO_enableInterrupt(GPIOA, USER_KEY_START_PIN);
    DL_GPIO_clearPins(GPIOB, MOTOR_DIR_AIN1_PIN |
		MOTOR_DIR_AIN2_PIN |
		MOTOR_DIR_BIN1_PIN |
		MOTOR_DIR_BIN2_PIN);
    DL_GPIO_enableOutput(GPIOB, MOTOR_DIR_AIN1_PIN |
		MOTOR_DIR_AIN2_PIN |
		MOTOR_DIR_BIN1_PIN |
		MOTOR_DIR_BIN2_PIN);
    DL_GPIO_setLowerPinsPolarity(GPIOB, DL_GPIO_PIN_11_EDGE_RISE_FALL |
		DL_GPIO_PIN_4_EDGE_RISE_FALL);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODER_RIGHT_A_PIN |
		ENCODER_LEFT_A_PIN);
    DL_GPIO_enableInterrupt(GPIOB, ENCODER_RIGHT_A_PIN |
		ENCODER_LEFT_A_PIN);

}



SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_enableMFCLK();

}


/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_RIGHTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_RIGHTConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 2000,
    .isTimerWithFourCC = true,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_RIGHT_init(void) {

    DL_TimerA_setClockConfig(
        PWM_RIGHT_INST, (DL_TimerA_ClockConfig *) &gPWM_RIGHTClockConfig);

    DL_TimerA_initPWMMode(
        PWM_RIGHT_INST, (DL_TimerA_PWMConfig *) &gPWM_RIGHTConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerA_setCounterControl(PWM_RIGHT_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerA_setCaptureCompareOutCtl(PWM_RIGHT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_RIGHT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_RIGHT_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_enableClock(PWM_RIGHT_INST);


    
    DL_TimerA_setCCPDirection(PWM_RIGHT_INST , DL_TIMER_CC0_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gPWM_LEFTClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gPWM_LEFTConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 2000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_LEFT_init(void) {

    DL_TimerG_setClockConfig(
        PWM_LEFT_INST, (DL_TimerG_ClockConfig *) &gPWM_LEFTClockConfig);

    DL_TimerG_initPWMMode(
        PWM_LEFT_INST, (DL_TimerG_PWMConfig *) &gPWM_LEFTConfig);

    // Set Counter control to the smallest CC index being used
    DL_TimerG_setCounterControl(PWM_LEFT_INST,DL_TIMER_CZC_CCCTL0_ZCOND,DL_TIMER_CAC_CCCTL0_ACOND,DL_TIMER_CLC_CCCTL0_LCOND);

    DL_TimerG_setCaptureCompareOutCtl(PWM_LEFT_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(PWM_LEFT_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_LEFT_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerG_enableClock(PWM_LEFT_INST);


    
    DL_TimerG_setCCPDirection(PWM_LEFT_INST , DL_TIMER_CC0_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (32000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   32000000 Hz = 32000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gCONTROL_TIMERClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 0U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * CONTROL_TIMER_INST_LOAD_VALUE = (1ms * 32000000 Hz) - 1
 */
static const DL_TimerA_TimerConfig gCONTROL_TIMERTimerConfig = {
    .period     = CONTROL_TIMER_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC_UP,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_CONTROL_TIMER_init(void) {

    DL_TimerA_setClockConfig(CONTROL_TIMER_INST,
        (DL_TimerA_ClockConfig *) &gCONTROL_TIMERClockConfig);

    DL_TimerA_initTimerMode(CONTROL_TIMER_INST,
        (DL_TimerA_TimerConfig *) &gCONTROL_TIMERTimerConfig);
    DL_TimerA_enableInterrupt(CONTROL_TIMER_INST , DL_TIMERA_INTERRUPT_LOAD_EVENT);
    DL_TimerA_enableClock(CONTROL_TIMER_INST);





}


static const DL_I2C_ClockConfig gI2C_DISPLAYClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_I2C_DISPLAY_init(void) {

    DL_I2C_setClockConfig(I2C_DISPLAY_INST,
        (DL_I2C_ClockConfig *) &gI2C_DISPLAYClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(I2C_DISPLAY_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(I2C_DISPLAY_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(I2C_DISPLAY_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(I2C_DISPLAY_INST, 7);
    DL_I2C_setControllerTXFIFOThreshold(I2C_DISPLAY_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(I2C_DISPLAY_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(I2C_DISPLAY_INST);


    /* Enable module */
    DL_I2C_enableController(I2C_DISPLAY_INST);


}

static const DL_UART_Main_ClockConfig gDEBUG_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_MFCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gDEBUG_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_DEBUG_UART_init(void)
{
    DL_UART_Main_setClockConfig(DEBUG_UART_INST, (DL_UART_Main_ClockConfig *) &gDEBUG_UARTClockConfig);

    DL_UART_Main_init(DEBUG_UART_INST, (DL_UART_Main_Config *) &gDEBUG_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115107.91
     */
    DL_UART_Main_setOversampling(DEBUG_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(DEBUG_UART_INST, DEBUG_UART_IBRD_4_MHZ_115200_BAUD, DEBUG_UART_FBRD_4_MHZ_115200_BAUD);



    DL_UART_Main_enable(DEBUG_UART_INST);
}

