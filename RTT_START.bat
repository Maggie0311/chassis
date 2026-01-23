@echo off
set INTERFACE=cmsis-dap.cfg
set TARGET_CFG=stm32f4x.cfg
set SPEED=4000

echo starting openocd with DAPLINK...
openocd -f interface/%INTERFACE% ^
         -f target/%TARGET_CFG% ^
         -c "adapter speed %SPEED%" ^
         -c "init" ^
         -c "rtt setup 0x20000000 0x20000 \"SEGGER RTT\"" ^
         -c "rtt start" ^
         -c "rtt server start 8880 0" ^
         -c "rtt server start 8881 1" ^
         -c "rtt server start 8882 2" ^