## THIS IS A GENERATED FILE -- DO NOT EDIT
.configuro: .libraries,em3 linker.cmd package/cfg/appBLE_pem3.oem3

# To simplify configuro usage in makefiles:
#     o create a generic linker command file name 
#     o set modification times of compiler.opt* files to be greater than
#       or equal to the generated config header
#
linker.cmd: package/cfg/appBLE_pem3.xdl
	$(SED) 's"^\"\(package/cfg/appBLE_pem3cfg.cmd\)\"$""\"C:/Users/Shirzad/Documents/Shirzad/Work/PC_BUZZER/FW/Amoo/ProjectAngel_Lite/CC26XX_BLE/FlashOnly_ST_OAD_ExtFlash/configPkg/\1\""' package/cfg/appBLE_pem3.xdl > $@
	-$(SETDATE) -r:max package/cfg/appBLE_pem3.h compiler.opt compiler.opt.defs
