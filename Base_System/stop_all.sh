#!/bin/bash

cd "$(dirname "$0")"

echo "Stopping all Base System processes..."

# Kill Python backend
pkill -f "python.*backend/main.py"

# Stop Docker containers
docker compose stop frontend

echo "Done."
