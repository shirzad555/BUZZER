################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Each subdirectory must supply rules for building sources it contributes
configPkg/linker.cmd: ../appBLE.cfg
	@echo 'Building file: $<'
	@echo 'Invoking: XDCtools'
	"C:/ti/xdctools_3_31_01_33_core/xs" --xdcpath="C:/ti/tirtos_simplelink_2_13_00_06/packages;C:/ti/tirtos_simplelink_2_13_00_06/products/bios_6_42_00_08/packages;C:/ti/tirtos_simplelink_2_13_00_06/products/uia_2_00_02_39/packages;C:/ti/ccsv6/ccs_base;" xdc.tools.configuro -o configPkg -t ti.targets.arm.elf.M3 -p ti.platforms.simplelink:CC2640F128 -r release -c "C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7" --compileOptions "-mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 --include_path=\"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/OS\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/BLE\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/Hardware\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/PROFILES\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/Includes\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/Application\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/Startup\" --include_path=\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600\" -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_UART_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --display_error_number --diag_warning=225 --gen_func_subsections=on " "$<"
	@echo 'Finished building: $<'
	@echo ' '

configPkg/compiler.opt: | configPkg/linker.cmd
configPkg/: | configPkg/linker.cmd


