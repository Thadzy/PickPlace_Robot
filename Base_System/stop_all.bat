@echo off

echo Stopping all Base System processes...

:: Kill Python backend
taskkill /F /IM python.exe /T >nul 2>&1

:: Stop Docker containers
docker compose stop frontend

echo Done.
pause
