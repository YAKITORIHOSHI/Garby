---
name: project-sketch-scope
description: "AI workspace instructions for MCU Flash GUI project. Specifies files to read vs ignore."
---

# 🚨 MCU FLASHER — ACTIVE SKETCH & HARDWARE SPECIFICATION (LIVE) 🚨

## 📋 LIVE HARDWARE & PROJECT SPECIFICATIONS (ALWAYS USE THIS LIST)
- **Active Project Directory**: `C:\Users\napht\Documents\GarbyMostLatest\BLE_Receiver-Final`
- **Authoritative Hardware Status**: `No board selected in GUI and no microcontroller connected.`
- **Currently Selected Board**: `None (No board currently selected in GUI)`
- **Target Platform / Architecture**: `N/A` (N/A)
- **Connected COM Port**: `None (No microcontroller connected)`
- **Serial Monitor Baud Rate**: `115200`
- **Upload Speed**: `460800` bps
- **Main Sketch Files in Project Root**: `BLE_Receiver-Final.ino`

*CRITICAL DIRECTIVE FOR AI: The specifications above are live and authoritative from MCU Flasher GUI. When the user asks what board is selected, what port is connected, or what baud rate/upload speed is configured, answer directly from this specification list immediately without searching the disk or running PowerShell commands.*

## ⚡ EXECUTION SEQUENCE

### 1️⃣ STEP 1: Hardware & Board Questions
- Use the Live Hardware Specifications above or inspect `.mcu_flasher_build_cache/project_state.json` (Read-Only).
  - If `hardware.mcu_connected` is false or `hardware.port` is null: answer immediately: "No microcontroller is currently connected."
  - If `hardware.board_selected` is false or `hardware.board_name` is null: answer immediately: "No board is currently selected in the GUI."
  *Never run recursive PowerShell/find/glob searches or inspect `platformio.ini` to guess board selection.*

### 2️⃣ STEP 2: Read & Edit ONLY Main Sketch Files at Root
- Work strictly on primary sketch source files located at the root of this project folder:
  - `*.ino` (Main Arduino sketch file at root)
  - `*.h` / `*.hpp` (C/C++ header files at root)
  - `*.cpp` / `*.c` (C/C++ source code files at root)
  - `NOTE.txt` / `*.txt` (Notes & project documentation files created for the user)
- Do NOT search, traverse, or inspect parent directories or external sibling folders.
- Do NOT create nested source directories or submodules unless explicitly requested.

### 3️⃣ STEP 3: Check Notifications & History When Troubleshooting
- If the user asks about a compilation error, upload failure, or reset issue, read `.mcu_flasher_build_cache/dbs_notif.json` (Read-Only) to see:
  - Recent compiler errors, missing library warnings, and toolchain logs
  - Upload status, device connect/disconnect logs, and library installations.

### 4️⃣ STEP 4: Strict No-Touch on Build & Cache Files
- Files inside `.mcu_flasher_build_cache/` are internal build inputs, backups, and PlatformIO toolchain data.
- **NEVER** edit, modify, rename, or delete files inside `.mcu_flasher_build_cache/`, `.pio/`, `.vscode/`, `.clangd/`, or `.opencode/`.

### 5️⃣ AI Edit Backup & Recovery
- Backup root: `C:/Users/napht/Documents/GarbyMostLatest/BLE_Receiver-Final/.mcu_flasher_build_cache/.mcu_ai_edits`
- Folder layout: `M-D-YY/sessionN/editN.txt`. Each edit file contains exact BEFORE, AI/AFTER, current-applied, and Undo-target copies.
- The hidden `.mcu_flasher_build_cache/.mcu_ai_edits` folder travels with this sketch project across drives and computers.
- Treat the backup tree as READ-ONLY. Never modify, rename, or delete backup files.
- Never read or edit `.mcu_flasher_build_cache/.mcu_ai_edits/.state`; it is application journal data.
- When the user explicitly asks to recover or compare an earlier AI edit, locate the matching project/file entry and restore only the requested content section.

---
*Generated automatically by MCU Flash GUI by Naph for OpenCode AI Assistant.*
