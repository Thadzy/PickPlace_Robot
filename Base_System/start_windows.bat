@echo off
setlocal

echo ------------------------------------------------
echo   PickPlace Robot - Base System (Windows)
echo ------------------------------------------------

:: 1. Check Docker
docker ps >nul 2>&1
if %errorlevel% equ 0 (
    echo Docker detected. Starting Frontend container...
    docker compose up -d frontend
) else (
    echo Warning: Docker is not running. Frontend UI may not be available.
    echo Please start Docker Desktop and refresh.
)

:: 2. Backend Setup
echo Checking Python requirements...
python -m pip install -r backend\requirements.txt --quiet

:: 3. Start Backend
echo Starting Backend...
echo Access the Web UI at http://localhost:3000
echo Select your STM32 COM port in the 'Connect' tab.
python backend\main.py

echo ------------------------------------------------
echo Process stopped.
pause
