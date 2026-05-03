#!/bin/bash
# If permission denied, run this once in Terminal:
# chmod +x start_linux.sh

# Get the directory where the script is located
cd "$(dirname "$0")"

PYTHON=$(command -v python3 2>/dev/null || command -v python 2>/dev/null)
if [ -z "$PYTHON" ]; then
    echo "ERROR: Python not found. Please install Python 3 from https://python.org"
    read -p "Press Enter to close..."
    exit 1
fi
echo "(+) Using Python: $PYTHON"

echo "------------------------------------------------"
echo "  PickPlace Robot - Base System (Linux)"
echo "------------------------------------------------"

# 1. Port Auto-detection
echo "Searching for STM32 Controller..."
PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1)

if [ -z "$PORT" ]; then
    echo "Warning: STM32 not auto-detected."
    read -p "Enter port manually (e.g. /dev/ttyACM0) or press Enter to skip: " PORT
fi

# 2. Check Permissions
if ! groups | grep -q "dialout"; then
    echo "Warning: You are not in the 'dialout' group."
    echo "You may need to run: sudo usermod -a -G dialout $USER"
    echo "Then log out and log back in."
fi

# 3. Check Docker
if command -v docker &> /dev/null && docker ps &> /dev/null; then
    echo "Docker detected. Starting Frontend container..."
    docker compose up -d frontend
else
    echo "Warning: Docker is not running or needs sudo."
    echo "Starting Frontend may require manual docker compose up -d frontend"
fi

# 4. Backend Setup
echo "Checking Python requirements..."
$PYTHON -m pip install -r backend/requirements.txt --quiet

# 5. Start Backend
if [ -n "$PORT" ]; then
    echo "Starting Backend on port: $PORT"
    $PYTHON backend/main.py --port "$PORT"
else
    echo "Starting Backend (Manual port selection required in Web UI)"
    $PYTHON backend/main.py
fi
