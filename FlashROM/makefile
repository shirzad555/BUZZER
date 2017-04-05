################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7

GEN_OPTS__FLAG := --cmd_file="configPkg/compiler.opt"
GEN_CMDS__FLAG := -l"configPkg/linker.cmd"

ORDERED_OBJS += \
"../ccsLinkerDefines.cmd" \
"../cc26xx_ble_app.cmd" \
"./Startup/Board.obj" \
"./Startup/ccfg_appBLE.obj" \
"./Startup/main.obj" \
"./PROFILES/devinfoservice.obj" \
"./PROFILES/gatt_uuid.obj" \
"./PROFILES/gattservapp_util.obj" \
"./PROFILES/peripheral.obj" \
"./PROFILES/simpleGATTprofile.obj" \
"./OS/ICall.obj" \
"./OS/ICallCC2650.obj" \
"./Hardware/PINCC26XX.obj" \
"./Hardware/PWM.obj" \
"./Hardware/SPICC26XXDMA.obj" \
"./Hardware/UART.obj" \
"./Hardware/UARTCC26XX.obj" \
"./Hardware/UDMACC26XX.obj" \
"./BLE/ICallBleAPI.obj" \
"./BLE/bleUserConfig.obj" \
"./Application/CC26XX_BLEPeripheral.obj" \
"./Application/LIS3DH_Driver.obj" \
"./Application/uart_logs.obj" \
"./Application/util.obj" \
$(GEN_CMDS__FLAG) \
-l"libc.a" \
-l"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/../ti/tirtos_simplelink_2_13_00_06/products/cc26xxware_2_21_01_15600/driverlib/bin/ccs/driverlib.lib" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include Startup/subdir_vars.mk
-include PROFILES/subdir_vars.mk
-include OS/subdir_vars.mk
-include Hardware/subdir_vars.mk
-include BLE/subdir_vars.mk
-include Application/subdir_vars.mk
-include subdir_rules.mk
-include Startup/subdir_rules.mk
-include PROFILES/subdir_rules.mk
-include OS/subdir_rules.mk
-include Hardware/subdir_rules.mk
-include BLE/subdir_rules.mk
-include Application/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
CC26XX_BLE.out \

EXE_OUTPUTS__QUOTED += \
"CC26XX_BLE.out" \

BIN_OUTPUTS += \
CC26XX_BLE.hex \

BIN_OUTPUTS__QUOTED += \
"CC26XX_BLE.hex" \


# All Target
all: CC26XX_BLE.out

# Tool invocations
CC26XX_BLE.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM Linker'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armcl" --cmd_file="C:\Users\Shirzad\Documents\Shirzad\Work\PC_BUZZER\FW\Amoo\ProjectAngel_Lite\CC26XX_BLE\ccsCompilerDefines.bcfg"  -mv7M3 --code_state=16 --abi=eabi -me -O4 --opt_for_speed=0 -g --gcc --define=USE_ICALL --define=POWER_SAVING --define=SBP_TASK_STACK_SIZE=700 --define=GAPROLE_TASK_STACK_SIZE=520 --define=HEAPMGR_SIZE=2672 --define=TI_DRIVERS_SPI_DMA_INCLUDED --define=TI_DRIVERS_UART_INCLUDED --define=ICALL_MAX_NUM_TASKS=3 --define=ICALL_MAX_NUM_ENTITIES=6 --define=xdc_runtime_Assert_DISABLE_ALL --define=MAX_NUM_BLE_CONNS=1 --define=CC26XXWARE --define=CC26XX --diag_wrap=off --diag_suppress=48 --display_error_number --diag_warning=225 --gen_func_subsections=on -z -m"CC26XX_BLE.map" --heap_size=0 --stack_size=256 -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/lib" -i"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/include" --reread_libs --define=ICALL_RAM0_ADDR=0x200043EC --display_error_number --diag_wrap=off --warn_sections --diag_suppress=10247-D --xml_link_info="CC26XX_BLE_linkInfo.xml" --rom_model --compress_dwarf=on --unused_section_elimination=on -o "CC26XX_BLE.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

CC26XX_BLE.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: ARM Hex Utility'
	"C:/ti/ccsv6/tools/compiler/ti-cgt-arm_5.2.7/bin/armhex" --memwidth=8 --romwidth=8 --intel -o "CC26XX_BLE.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(GEN_MISC_FILES__QUOTED)$(EXE_OUTPUTS__QUOTED)$(GEN_FILES__QUOTED)$(BIN_OUTPUTS__QUOTED)$(GEN_OPTS__QUOTED)$(GEN_CMDS__QUOTED)
	-$(RMDIR) $(GEN_MISC_DIRS__QUOTED)
	-$(RM) "Startup\Board.pp" "Startup\ccfg_appBLE.pp" "Startup\main.pp" "PROFILES\devinfoservice.pp" "PROFILES\gatt_uuid.pp" "PROFILES\gattservapp_util.pp" "PROFILES\peripheral.pp" "PROFILES\simpleGATTprofile.pp" "OS\ICall.pp" "OS\ICallCC2650.pp" "Hardware\PINCC26XX.pp" "Hardware\PWM.pp" "Hardware\SPICC26XXDMA.pp" "Hardware\UART.pp" "Hardware\UARTCC26XX.pp" "Hardware\UDMACC26XX.pp" "BLE\ICallBleAPI.pp" "BLE\bleUserConfig.pp" "Application\CC26XX_BLEPeripheral.pp" "Application\LIS3DH_Driver.pp" "Application\uart_logs.pp" "Application\util.pp" 
	-$(RM) "Startup\Board.obj" "Startup\ccfg_appBLE.obj" "Startup\main.obj" "PROFILES\devinfoservice.obj" "PROFILES\gatt_uuid.obj" "PROFILES\gattservapp_util.obj" "PROFILES\peripheral.obj" "PROFILES\simpleGATTprofile.obj" "OS\ICall.obj" "OS\ICallCC2650.obj" "Hardware\PINCC26XX.obj" "Hardware\PWM.obj" "Hardware\SPICC26XXDMA.obj" "Hardware\UART.obj" "Hardware\UARTCC26XX.obj" "Hardware\UDMACC26XX.obj" "BLE\ICallBleAPI.obj" "BLE\bleUserConfig.obj" "Application\CC26XX_BLEPeripheral.obj" "Application\LIS3DH_Driver.obj" "Application\uart_logs.obj" "Application\util.obj" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

