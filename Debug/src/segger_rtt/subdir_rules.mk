################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
src/segger_rtt/%.o: ../src/segger_rtt/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"F:/ti/26H/H2_LineCar_CCS/src" -I"F:/ti/26H/H2_LineCar_CCS" -I"F:/ti/26H/H2_LineCar_CCS/Debug" -I"F:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"src/segger_rtt/$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


