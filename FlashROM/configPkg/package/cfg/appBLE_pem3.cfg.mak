# invoke SourceDir generated makefile for appBLE.pem3
appBLE.pem3: .libraries,appBLE.pem3
.libraries,appBLE.pem3: package/cfg/appBLE_pem3.xdl
	$(MAKE) -f C:\Users\Shirzad\Documents\Shirzad\Work\PC_BUZZER\FW\Amoo\ProjectAngel_Lite\CC26XX_BLE/src/makefile.libs

clean::
	$(MAKE) -f C:\Users\Shirzad\Documents\Shirzad\Work\PC_BUZZER\FW\Amoo\ProjectAngel_Lite\CC26XX_BLE/src/makefile.libs clean

