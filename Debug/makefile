################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS

GEN_OPTS__FLAG := @"./device.opt" 
GEN_CMDS__FLAG := -Wl,-l"./device_linker.cmd" 

ORDERED_OBJS += \
"./ti_msp_dl_config.o" \
"./startup_mspm0g350x_ticlang.o" \
"./main.o" \
"./src/app.o" \
"./src/h3_ball_control.o" \
"./src/h3_tuning_link.o" \
"./src/h3_vision.o" \
"./src/i2c_bus.o" \
"./src/line_sensor.o" \
"./src/motor_encoder.o" \
"./src/ssd1306.o" \
"./src/zdt_stepper.o" \
"./src/segger_rtt/SEGGER_RTT.o" \
"./src/segger_rtt/SEGGER_RTT_printf.o" \
$(GEN_CMDS__FLAG) \
-Wl,-ldevice.cmd.genlibs \
-Wl,-llibc.a \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src/subdir_vars.mk
-include src/segger_rtt/subdir_vars.mk
-include subdir_rules.mk
-include src/subdir_rules.mk
-include src/segger_rtt/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(SV7A_DEPS)),)
-include $(SV7A_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
H2_LineCar_CCS.out 

EXE_OUTPUTS__QUOTED += \
"H2_LineCar_CCS.out" 


# All Target
all: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@$(MAKE) --no-print-directory -Onone "H2_LineCar_CCS.out"

# Tool invocations
H2_LineCar_CCS.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Arm Linker - building target: "$@"'
	"F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -gdwarf-3 -Wall -Wl,-m"H2_LineCar_CCS.map" -Wl,-i"F:/ti/mspm0_sdk_2_10_00_04/source" -Wl,-i"F:/ti/26H/H2_LineCar_CCS" -Wl,-i"F:/ti/26H/H2_LineCar_CCS/Debug/syscfg" -Wl,-i"F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/lib" -Wl,--diag_wrap=off -Wl,--display_error_number -Wl,--warn_sections -Wl,--xml_link_info="H2_LineCar_CCS_linkInfo.xml" -Wl,--rom_model -o "H2_LineCar_CCS.out" $(ORDERED_OBJS)
	@echo 'Finished building target: "$@"'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(GEN_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)
	-$(RM) "ti_msp_dl_config.o" "startup_mspm0g350x_ticlang.o" "main.o" "src\app.o" "src\h3_ball_control.o" "src\h3_tuning_link.o" "src\h3_vision.o" "src\i2c_bus.o" "src\line_sensor.o" "src\motor_encoder.o" "src\ssd1306.o" "src\zdt_stepper.o" "src\segger_rtt\SEGGER_RTT.o" "src\segger_rtt\SEGGER_RTT_printf.o" 
	-$(RM) "ti_msp_dl_config.d" "startup_mspm0g350x_ticlang.d" "main.d" "src\app.d" "src\h3_ball_control.d" "src\h3_tuning_link.d" "src\h3_vision.d" "src\i2c_bus.d" "src\line_sensor.d" "src\motor_encoder.d" "src\ssd1306.d" "src\zdt_stepper.d" "src\segger_rtt\SEGGER_RTT.d" "src\segger_rtt\SEGGER_RTT_printf.d" 
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

