nrfjprog --eraseall

cp ../build/mcuboot/zephyr/zephyr.bin mcu.bin
#cp ../build/zephyr/app_update.bin old/new.bin

JLinkExe -device CORTEX-M4 -si SWD -CommandFile flash_dual.jtag 
