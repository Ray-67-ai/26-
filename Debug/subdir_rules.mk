################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-194620796: ../empty.syscfg
	@echo 'SysConfig - building file: "$<"'
	"F:/ti/ccs2051/ccs/utils/sysconfig_1.27.1/sysconfig_cli.bat" -s "F:/ti/mspm0_sdk_2_10_00_04/.metadata/product.json" --script "F:/ti/26H/H2_LineCar_CCS/empty.syscfg" -o "." --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

device_linker.cmd: build-194620796 ../empty.syscfg
device.opt: build-194620796
device.cmd.genlibs: build-194620796
ti_msp_dl_config.c: build-194620796
ti_msp_dl_config.h: build-194620796
Event.dot: build-194620796

%.o: ./%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"F:/ti/26H/H2_LineCar_CCS/src" -I"F:/ti/26H/H2_LineCar_CCS" -I"F:/ti/26H/H2_LineCar_CCS/Debug" -I"F:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

startup_mspm0g350x_ticlang.o: F:/ti/mspm0_sdk_2_10_00_04/source/ti/devices/msp/m0p/startup_system_files/ticlang/startup_mspm0g350x_ticlang.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"F:/ti/26H/H2_LineCar_CCS/src" -I"F:/ti/26H/H2_LineCar_CCS" -I"F:/ti/26H/H2_LineCar_CCS/Debug" -I"F:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Arm Compiler - building file: "$<"'
	"F:/ti/ccs2051/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe" -c @"device.opt"  -march=thumbv6m -mcpu=cortex-m0plus -mfloat-abi=soft -mlittle-endian -mthumb -O0 -I"F:/ti/26H/H2_LineCar_CCS/src" -I"F:/ti/26H/H2_LineCar_CCS" -I"F:/ti/26H/H2_LineCar_CCS/Debug" -I"F:/ti/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include" -I"F:/ti/mspm0_sdk_2_10_00_04/source" -gdwarf-3 -Wall -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


