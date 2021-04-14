################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Add inputs and outputs from these tool invocations to the build variables 
CMD_SRCS += \
../source/sys_link.cmd 

ASM_SRCS += \
../source/dabort.asm \
../source/sys_core.asm \
../source/sys_intvecs.asm \
../source/sys_mpu.asm \
../source/sys_pmu.asm 

C_SRCS += \
../source/ADS1018.c \
../source/Device_TMS570LS07.c \
../source/Fapi_UserDefinedFunctions.c \
../source/MCP23S17.c \
../source/PCA2129.c \
../source/SST25PF040C.c \
../source/adc.c \
../source/can.c \
../source/errata_SSWF021_45.c \
../source/esm.c \
../source/gio.c \
../source/het.c \
../source/notification.c \
../source/pinmux.c \
../source/rti.c \
../source/spi.c \
../source/sys_dma.c \
../source/sys_main.c \
../source/sys_pcr.c \
../source/sys_phantom.c \
../source/sys_pmm.c \
../source/sys_selftest.c \
../source/sys_startup.c \
../source/sys_vim.c \
../source/system.c \
../source/ti_fee_Info.c \
../source/ti_fee_cancel.c \
../source/ti_fee_cfg.c \
../source/ti_fee_eraseimmediateblock.c \
../source/ti_fee_format.c \
../source/ti_fee_ini.c \
../source/ti_fee_invalidateblock.c \
../source/ti_fee_main.c \
../source/ti_fee_read.c \
../source/ti_fee_readSync.c \
../source/ti_fee_shutdown.c \
../source/ti_fee_util.c \
../source/ti_fee_writeAsync.c \
../source/ti_fee_writeSync.c 

C_DEPS += \
./source/ADS1018.d \
./source/Device_TMS570LS07.d \
./source/Fapi_UserDefinedFunctions.d \
./source/MCP23S17.d \
./source/PCA2129.d \
./source/SST25PF040C.d \
./source/adc.d \
./source/can.d \
./source/errata_SSWF021_45.d \
./source/esm.d \
./source/gio.d \
./source/het.d \
./source/notification.d \
./source/pinmux.d \
./source/rti.d \
./source/spi.d \
./source/sys_dma.d \
./source/sys_main.d \
./source/sys_pcr.d \
./source/sys_phantom.d \
./source/sys_pmm.d \
./source/sys_selftest.d \
./source/sys_startup.d \
./source/sys_vim.d \
./source/system.d \
./source/ti_fee_Info.d \
./source/ti_fee_cancel.d \
./source/ti_fee_cfg.d \
./source/ti_fee_eraseimmediateblock.d \
./source/ti_fee_format.d \
./source/ti_fee_ini.d \
./source/ti_fee_invalidateblock.d \
./source/ti_fee_main.d \
./source/ti_fee_read.d \
./source/ti_fee_readSync.d \
./source/ti_fee_shutdown.d \
./source/ti_fee_util.d \
./source/ti_fee_writeAsync.d \
./source/ti_fee_writeSync.d 

OBJS += \
./source/ADS1018.obj \
./source/Device_TMS570LS07.obj \
./source/Fapi_UserDefinedFunctions.obj \
./source/MCP23S17.obj \
./source/PCA2129.obj \
./source/SST25PF040C.obj \
./source/adc.obj \
./source/can.obj \
./source/dabort.obj \
./source/errata_SSWF021_45.obj \
./source/esm.obj \
./source/gio.obj \
./source/het.obj \
./source/notification.obj \
./source/pinmux.obj \
./source/rti.obj \
./source/spi.obj \
./source/sys_core.obj \
./source/sys_dma.obj \
./source/sys_intvecs.obj \
./source/sys_main.obj \
./source/sys_mpu.obj \
./source/sys_pcr.obj \
./source/sys_phantom.obj \
./source/sys_pmm.obj \
./source/sys_pmu.obj \
./source/sys_selftest.obj \
./source/sys_startup.obj \
./source/sys_vim.obj \
./source/system.obj \
./source/ti_fee_Info.obj \
./source/ti_fee_cancel.obj \
./source/ti_fee_cfg.obj \
./source/ti_fee_eraseimmediateblock.obj \
./source/ti_fee_format.obj \
./source/ti_fee_ini.obj \
./source/ti_fee_invalidateblock.obj \
./source/ti_fee_main.obj \
./source/ti_fee_read.obj \
./source/ti_fee_readSync.obj \
./source/ti_fee_shutdown.obj \
./source/ti_fee_util.obj \
./source/ti_fee_writeAsync.obj \
./source/ti_fee_writeSync.obj 

ASM_DEPS += \
./source/dabort.d \
./source/sys_core.d \
./source/sys_intvecs.d \
./source/sys_mpu.d \
./source/sys_pmu.d 

OBJS__QUOTED += \
"source\ADS1018.obj" \
"source\Device_TMS570LS07.obj" \
"source\Fapi_UserDefinedFunctions.obj" \
"source\MCP23S17.obj" \
"source\PCA2129.obj" \
"source\SST25PF040C.obj" \
"source\adc.obj" \
"source\can.obj" \
"source\dabort.obj" \
"source\errata_SSWF021_45.obj" \
"source\esm.obj" \
"source\gio.obj" \
"source\het.obj" \
"source\notification.obj" \
"source\pinmux.obj" \
"source\rti.obj" \
"source\spi.obj" \
"source\sys_core.obj" \
"source\sys_dma.obj" \
"source\sys_intvecs.obj" \
"source\sys_main.obj" \
"source\sys_mpu.obj" \
"source\sys_pcr.obj" \
"source\sys_phantom.obj" \
"source\sys_pmm.obj" \
"source\sys_pmu.obj" \
"source\sys_selftest.obj" \
"source\sys_startup.obj" \
"source\sys_vim.obj" \
"source\system.obj" \
"source\ti_fee_Info.obj" \
"source\ti_fee_cancel.obj" \
"source\ti_fee_cfg.obj" \
"source\ti_fee_eraseimmediateblock.obj" \
"source\ti_fee_format.obj" \
"source\ti_fee_ini.obj" \
"source\ti_fee_invalidateblock.obj" \
"source\ti_fee_main.obj" \
"source\ti_fee_read.obj" \
"source\ti_fee_readSync.obj" \
"source\ti_fee_shutdown.obj" \
"source\ti_fee_util.obj" \
"source\ti_fee_writeAsync.obj" \
"source\ti_fee_writeSync.obj" 

C_DEPS__QUOTED += \
"source\ADS1018.d" \
"source\Device_TMS570LS07.d" \
"source\Fapi_UserDefinedFunctions.d" \
"source\MCP23S17.d" \
"source\PCA2129.d" \
"source\SST25PF040C.d" \
"source\adc.d" \
"source\can.d" \
"source\errata_SSWF021_45.d" \
"source\esm.d" \
"source\gio.d" \
"source\het.d" \
"source\notification.d" \
"source\pinmux.d" \
"source\rti.d" \
"source\spi.d" \
"source\sys_dma.d" \
"source\sys_main.d" \
"source\sys_pcr.d" \
"source\sys_phantom.d" \
"source\sys_pmm.d" \
"source\sys_selftest.d" \
"source\sys_startup.d" \
"source\sys_vim.d" \
"source\system.d" \
"source\ti_fee_Info.d" \
"source\ti_fee_cancel.d" \
"source\ti_fee_cfg.d" \
"source\ti_fee_eraseimmediateblock.d" \
"source\ti_fee_format.d" \
"source\ti_fee_ini.d" \
"source\ti_fee_invalidateblock.d" \
"source\ti_fee_main.d" \
"source\ti_fee_read.d" \
"source\ti_fee_readSync.d" \
"source\ti_fee_shutdown.d" \
"source\ti_fee_util.d" \
"source\ti_fee_writeAsync.d" \
"source\ti_fee_writeSync.d" 

ASM_DEPS__QUOTED += \
"source\dabort.d" \
"source\sys_core.d" \
"source\sys_intvecs.d" \
"source\sys_mpu.d" \
"source\sys_pmu.d" 

C_SRCS__QUOTED += \
"../source/ADS1018.c" \
"../source/Device_TMS570LS07.c" \
"../source/Fapi_UserDefinedFunctions.c" \
"../source/MCP23S17.c" \
"../source/PCA2129.c" \
"../source/SST25PF040C.c" \
"../source/adc.c" \
"../source/can.c" \
"../source/errata_SSWF021_45.c" \
"../source/esm.c" \
"../source/gio.c" \
"../source/het.c" \
"../source/notification.c" \
"../source/pinmux.c" \
"../source/rti.c" \
"../source/spi.c" \
"../source/sys_dma.c" \
"../source/sys_main.c" \
"../source/sys_pcr.c" \
"../source/sys_phantom.c" \
"../source/sys_pmm.c" \
"../source/sys_selftest.c" \
"../source/sys_startup.c" \
"../source/sys_vim.c" \
"../source/system.c" \
"../source/ti_fee_Info.c" \
"../source/ti_fee_cancel.c" \
"../source/ti_fee_cfg.c" \
"../source/ti_fee_eraseimmediateblock.c" \
"../source/ti_fee_format.c" \
"../source/ti_fee_ini.c" \
"../source/ti_fee_invalidateblock.c" \
"../source/ti_fee_main.c" \
"../source/ti_fee_read.c" \
"../source/ti_fee_readSync.c" \
"../source/ti_fee_shutdown.c" \
"../source/ti_fee_util.c" \
"../source/ti_fee_writeAsync.c" \
"../source/ti_fee_writeSync.c" 

ASM_SRCS__QUOTED += \
"../source/dabort.asm" \
"../source/sys_core.asm" \
"../source/sys_intvecs.asm" \
"../source/sys_mpu.asm" \
"../source/sys_pmu.asm" 


