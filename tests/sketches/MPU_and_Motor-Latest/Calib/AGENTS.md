# MCU Flasher Project Instructions

## Project and Live Hardware State

- The project root is the directory containing this `AGENTS.md`. Use project-relative paths so this project can move between computers.
- Hardware selection and connection details change at runtime. When asked, use the current MCU Flasher GUI state. If it is unavailable, read `.mcu_flasher_build_cache/project_state.json` (read-only) and report only what that saved state says. If the file or requested value is absent, say the state is unavailable; never infer a board or port from project files.
- Do not treat board, port, baud-rate, upload-speed, absolute-path, or source-file values from older generated instructions as current.

## EXECUTION SEQUENCE

### 1STEP 1: Hardware & Board Questions
- Use current GUI state or inspect `.mcu_flasher_build_cache/project_state.json` (read-only). If it reports `hardware.mcu_connected` as false or `hardware.port` as null, say no microcontroller is connected. If it reports `hardware.board_selected` as false or `hardware.board_name` as null, say no board is selected.
- Never inspect `platformio.ini`, scan hardware package folders, or guess board selection.

### 2STEP 2: Read & Edit ONLY Main Sketch Files at Root
- For firmware work, use only primary sketch source files at this project's root:
  - `*.ino` (Main Arduino sketch file at root)
  - `*.h` / `*.hpp` (C/C++ header files at root)
  - `*.cpp` / `*.c` (C/C++ source code files at root)
  - `NOTE.txt` / `*.txt` (Notes & project documentation files created for the user)
- Do NOT search, traverse, or inspect parent directories or external sibling folders.
- Do NOT create nested source directories or submodules unless explicitly requested.

### 3STEP 3: Check Notifications & History When Troubleshooting
- If the user asks about a compilation error, upload failure, or reset issue, read `.mcu_flasher_build_cache/dbs_notif.json` (read-only) to see:
  - Recent compiler errors, missing library warnings, and toolchain logs
  - Upload status, device connect/disconnect logs, and library installations.

### 4STEP 4: Strict No-Touch on Build & Cache Files
- Files inside `.mcu_flasher_build_cache/` are internal build inputs, backups, and PlatformIO toolchain data.
- **NEVER** edit, modify, rename, or delete files inside `.mcu_flasher_build_cache/`, `.pio/`, `.vscode/`, `.clangd/`, or `.opencode/`.

### 5AI Edit Backup & Recovery
- Backup root: `.mcu_flasher_build_cache/.mcu_ai_edits` (relative to the project root).
- Folder layout: `M-D-YY/sessionN/editN.txt`. Each edit file contains exact BEFORE, AI/AFTER, current-applied, and Undo-target copies.
- The hidden `.mcu_flasher_build_cache/.mcu_ai_edits` folder travels with this sketch project across drives and computers.
- Treat the backup tree as READ-ONLY. Never modify, rename, or delete backup files.
- Never read or edit `.mcu_flasher_build_cache/.mcu_ai_edits/.state`; it is application journal data.
- When the user explicitly asks to recover or compare an earlier AI edit, locate the matching project/file entry and restore only the requested content section.

---
*Generated automatically by MCU Flash GUI for OpenCode and compatible coding assistants. Paths are project-relative; hardware state is read at runtime.*
