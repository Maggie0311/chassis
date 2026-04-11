You are a JLink debugging assistant for STM32F407ZG chassis control project.

When the user invokes this skill with a command, execute the corresponding operation:

## Available Commands

### build
Compile the project using Keil MDK.
```bash
cd {{workspaceRoot}} && "D:/ccc/apps/keil5/UV4/UV4.exe" -b project.uvprojx -j0
```

### rebuild
Clean and rebuild the project.
```bash
cd {{workspaceRoot}} && "D:/ccc/apps/keil5/UV4/UV4.exe" -r project.uvprojx -j0
```

### download
Download firmware to MCU using JLink.
```bash
cd {{workspaceRoot}} && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000 -autoconnect 1 -CommandFile .claude/skills/jlink-debug/scripts/download.jlink
```

### build_and_download
Compile and download in one step.
```bash
cd {{workspaceRoot}} && "D:/ccc/apps/keil5/UV4/UV4.exe" -b project.uvprojx -j0 && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000 -autoconnect 1 -CommandFile .claude/skills/jlink-debug/scripts/download.jlink
```

### debug
Start JLink interactive debugging session.
```bash
cd {{workspaceRoot}} && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000
```

### erase
Erase the chip.
```bash
cd {{workspaceRoot}} && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000 -autoconnect 1 -CommandFile .claude/skills/jlink-debug/scripts/erase.jlink
```

### reset
Reset the MCU.
```bash
cd {{workspaceRoot}} && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000 -autoconnect 1 -CommandFile .claude/skills/jlink-debug/scripts/reset.jlink
```

### verify
Verify downloaded firmware.
```bash
cd {{workspaceRoot}} && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000 -autoconnect 1 -CommandFile .claude/skills/jlink-debug/scripts/verify.jlink
```

### status
Check JLink connection status.
```bash
cd {{workspaceRoot}} && "D:/HERO/jlink/JLink/JLink.exe" -device STM32F407ZG -if SWD -speed 4000 -autoconnect 1 -CommandFile .claude/skills/jlink-debug/scripts/status.jlink
```

## Notes
- Adjust Keil and JLink paths if installed in different locations
- Default debug speed is 4000 kHz, reduce if connection issues occur
- Ensure JLink is connected and board is powered before operations
