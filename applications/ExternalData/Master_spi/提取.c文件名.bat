@echo off
for /f %%i in ('dir /a-d /b *.c') do echo %%i
pause