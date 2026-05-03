#!/bin/bash
# If this script won't open, run this once in Terminal:
# chmod +x start_mac.command

# Get the directory where the script is located
cd "$(dirname "$0")"

PYTHON=$(command -v python3 2>/dev/null || command -v python 2>/dev/null)
if [ -z "$PYTHON" ]; then
    echo "ERROR: Python not found. Please install Python 3 from https://python.org"
    read -p "Press Enter to close..."
    exit 1
fi
echo "(+) Using Python: $PYTHON"

echo "================================================="
echo "  PickPlace Robot - Base System (macOS)"
echo "================================================="

# 1. Port Auto-detection
echo "[1/4] Searching for STM32 Controller..."
PORT=$(ls /dev/tty.usbmodem* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "(!) STM32 not auto-detected."
    read -p "    Please enter port manually (e.g. /dev/tty.usbmodem1101) or press Enter to skip: " PORT
else
    echo "(+) Detected: $PORT"
fi

# 2. Check Docker and start frontend
echo "[2/4] Checking Docker..."
if command -v docker &> /dev/null && docker ps &> /dev/null; then
    echo "(+) Docker is running. Starting Frontend container..."
    docker compose up -d frontend
else
    echo "(!) Docker is not running or not installed."
    echo "    Frontend UI will not be available. Please start Docker Desktop."
fi

# 3. Install requirements
echo "[3/4] Installing Python requirements..."
$PYTHON -m pip install -r backend/requirements.txt --quiet

# 4. Start Backend
echo "[4/4] Starting Backend..."
echo "-------------------------------------------------"
if [ -n "$PORT" ]; then
    echo "Backend running on $PORT"
    $PYTHON backend/main.py --port "$PORT"
else
    echo "Backend running (Manual port selection required in Web UI)"
    $PYTHON backend/main.py
fi

echo "-------------------------------------------------"
echo "Process stopped."
read -p "Press Enter to close this window..."
