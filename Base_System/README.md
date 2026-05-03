# Base System

WebSocket-to-Modbus bridge for the FRA502 Circular Pick and Place Robot.

```
Browser UI (port 3000) ─── WebSocket ──► Backend (port 8765) ─── Modbus RTU ──► STM32
```

---

## Quick Start

Use the launcher script for your operating system.
No manual commands needed.

| OS | Script | Action |
|----|--------|--------|
| macOS | start_mac.command | Double-click or run in Terminal |
| Windows | start_windows.bat | Double-click |
| Linux | start_linux.sh | Run in Terminal |

### First-time setup (all platforms)

Step 1 — Load frontend Docker image (once only):
```bash
docker load -i frontend-image.tar
```

Step 2 — Make scripts executable (macOS/Linux only, once only):
```bash
chmod +x start_mac.command start_linux.sh stop_all.sh
```

Step 3 — Run your platform script and open browser at http://localhost:3000

---

## What the scripts do

- Auto-detect STM32 serial port (macOS/Linux)
- Start Docker frontend container (using `docker compose`)
- Install Python requirements automatically
- Start Python backend
- Windows: no port needed, Web UI lets you select COM port

---

## File Structure

```
Base_System/
├── start_mac.command
├── start_linux.sh
├── start_windows.bat
├── stop_all.sh
├── stop_all.bat
├── docker-compose.yml
├── frontend-image.tar
└── backend/
    ├── main.py
    ├── protocol.py
    ├── requirements.txt
    └── Dockerfile
```

---

## Stopping the system

| OS | Script | Alternative |
|----|--------|-------------|
| macOS/Linux | ./stop_all.sh | Ctrl+C in terminal |
| Windows | stop_all.bat | Close the terminal window |

---

## Manual commands (advanced)

### macOS (Apple Silicon / Intel)

**Find your serial port:**
```bash
ls /dev/tty.usb*
```

**Run backend:**
```bash
python backend/main.py --port /dev/tty.usbmodem1203
```

**Checklist:**
- STM32 USB plugged into this Mac
- STM32CubeIDE Debugger stopped
- Serial port visible in `ls /dev/tty.usb*`

---

### Windows

**Find your serial port:**
1. Open Device Manager -> Ports (COM & LPT)
2. Find STMicroelectronics STLink Virtual COM Port (COMx)

**Run backend:**
```bash
pip install -r backend/requirements.txt
docker compose up -d frontend
python backend/main.py --port COM3
```

**Checklist:**
- STM32 USB plugged into this Windows machine
- STM32CubeIDE Debugger stopped
- Only one instance of main.py running

---

### Linux

**Find your serial port:**
```bash
ls /dev/ttyACM*
```

**Run backend natively:**
```bash
python backend/main.py --port /dev/ttyACM0
```

**Checklist:**
- STM32 USB plugged into this machine
- STM32CubeIDE Debugger stopped
- User is in `dialout` group (`sudo usermod -a -G dialout $USER`)

---

## Verify everything is running

Run these commands to confirm all services are up:

```bash
# Serial port visible
ls /dev/tty.usb*          # macOS
ls /dev/ttyACM*           # Linux

# Frontend container running
docker ps | grep frontend

# Backend process running
ps aux | grep main.py

# Ports in use
lsof -i :3000             # macOS/Linux
lsof -i :8765             # macOS/Linux
```

Expected results:
- Port 3000 — nginx (Frontend)
- Port 8765 — python (Backend)
- Browser shows Connection dialog at http://localhost:3000

---

## Common commands

| Action | Command |
|--------|---------|
| Start frontend | `docker compose up -d frontend` |
| Stop frontend | `docker compose down` |
| View frontend logs | `docker compose logs -f frontend` |
| List serial ports | `python backend/main.py --list-ports` |
| Restart backend | Kill and re-run `python backend/main.py` |

---

## Troubleshooting

**Script won't open on macOS**
- Run once: `chmod +x start_mac.command`
- Then double-click or run: `./start_mac.command`

**Script permission denied on Linux**
- Run once: `chmod +x start_linux.sh`
- Then run: `./start_linux.sh`

**Port already in use (8765)**
- Run `stop_all.sh` (macOS/Linux) or `stop_all.bat` (Windows)
- Then start again

**Browser shows "Server is offline"**
- Backend is not running — start `python backend/main.py`
- Port 8765 already in use — kill existing process first

**Connected but Status ERROR on all commands**
- STM32 Debugger is still running — stop it in STM32CubeIDE
- Wrong COM port — check Device Manager (Windows) or `ls /dev/tty.usb*` (macOS)
- printf redirected to wrong UART — ensure `_write()` uses `huart3` not `hlpuart1`

**macOS: No serial port found**
- STM32 USB is plugged into Windows, not Mac
- Try unplugging and replugging the USB cable

**Windows: Address already in use**
```bash
taskkill /F /IM main.exe
taskkill /F /IM python.exe
python backend/main.py --port COM3
```

---

## STM32 firmware settings

| Setting    | Value                   |
|-----------|-------------------------|
| Baud rate  | 19200                   |
| Data bits  | 8                       |
| Parity     | Even                    |
| Stop bits  | 1                       |
| Slave ID   | 21                      |
| UART       | LPUART1 via ST-Link VCP |
| Modbus     | RTU Slave mode          |
