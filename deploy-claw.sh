#!/bin/bash

# Deploy Claw Control Script
# Uses pico-mount.sh to mount Pico and copies ENGINE_CONTROL_CLAW.uf2
# Runs without sudo using udisksctl via pico-mount.sh

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
UF2_FILE="$SCRIPT_DIR/build/claw-control/ENGINE_CONTROL_CLAW.uf2"
PICO_MOUNT_SCRIPT="$SCRIPT_DIR/pico-mount.sh"
MOUNT_POINT=""

# Colors for better output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "🦾 Claw Control Deployment Script"
echo "==============================================="

# Check if pico-mount.sh exists
if [ ! -f "$PICO_MOUNT_SCRIPT" ]; then
    echo -e "${RED}❌ pico-mount.sh not found: $PICO_MOUNT_SCRIPT${NC}"
    echo "   Make sure pico-mount.sh is in the same directory as this script."
    exit 1
fi

# Build the project first
echo -e "${BLUE}🔨 Building project...${NC}"
echo "Running: cmake --build build"
echo

if ! cmake --build build; then
    echo
    echo -e "${RED}❌ Build failed!${NC}"
    echo "   Please fix build errors before deploying."
    exit 1
fi

echo
echo -e "${GREEN}✅ Build completed successfully${NC}"

# Check UF2 file exists after build
if [ ! -f "$UF2_FILE" ]; then
    echo -e "${RED}❌ UF2 file not found after build: $UF2_FILE${NC}"
    echo "   Build may have failed or not targeted claw-control."
    exit 1
fi

echo -e "${GREEN}✅ Found UF2 file: $UF2_FILE${NC}"
echo -e "   File size: $(ls -lh "$UF2_FILE" | awk '{print $5}')"
echo

# Turn off engines power
echo -e "${BLUE}🔄 Turning off engines...${NC}"
curl -X POST http://192.168.1.120/set \
     -H "Content-Type: application/json" \
     -d '{"enginesEnabled": false }' \
     -m 10 --silent --fail


# Switch Pico to boot mode via HTTP request
echo -e "${BLUE}🔄 Switching Pico to boot mode...${NC}"
curl -X POST http://192.168.1.120/upgrade \
     -H "Content-Type: application/json" \
     -d '{"part": "claw"}' \
     -m 10 --silent --fail

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ Boot mode request sent successfully${NC}"
    echo -e "${BLUE}⏳ Waiting 10 seconds for Pico to switch to boot mode...${NC}"
    sleep 10
else
    echo -e "${YELLOW}⚠️  Boot mode request failed or timed out${NC}"
    echo "   The Pico might already be in boot mode or not reachable."
    echo "   Continuing with mount attempt..."
fi
echo

# Use pico-mount.sh to mount the Pico
echo -e "${BLUE}🔄 Mounting Pico using pico-mount.sh...${NC}"
MOUNT_OUTPUT=$("$PICO_MOUNT_SCRIPT" mount 2>&1)
MOUNT_EXIT_CODE=$?

if [ $MOUNT_EXIT_CODE -ne 0 ]; then
    echo -e "${RED}❌ Failed to mount Pico${NC}"
    echo "$MOUNT_OUTPUT"
    exit 1
fi

# Extract mount point from pico-mount.sh output
MOUNT_POINT=$(echo "$MOUNT_OUTPUT" | grep -E "(Successfully mounted to|Already mounted at):" | sed -E 's/.*(Successfully mounted to|Already mounted at): (.+)/\2/')

if [ -z "$MOUNT_POINT" ]; then
    echo -e "${RED}❌ Failed to determine mount point from pico-mount.sh output${NC}"
    echo "Mount output: $MOUNT_OUTPUT"
    exit 1
fi

echo -e "${GREEN}✅ Pico mounted at: $MOUNT_POINT${NC}"
echo

# Copy UF2 file
echo -e "${BLUE}🚀 Copying claw control firmware...${NC}"
echo "   Source: $UF2_FILE"
echo "   Target: $MOUNT_POINT/"

# Copy the file
cp "$UF2_FILE" "$MOUNT_POINT/"

if [ $? -eq 0 ]; then
    echo -e "${GREEN}✅ File copied successfully!${NC}"
    
    # Verify the copy
    if [ -f "$MOUNT_POINT/ENGINE_CONTROL_ELBOW.uf2" ]; then
        echo -e "   Target file size: $(ls -lh "$MOUNT_POINT/ENGINE_CONTROL_ELBOW.uf2" | awk '{print $5}')"
    fi
else
    echo -e "${RED}❌ Copy failed${NC}"
    exit 1
fi

echo

# Sync and prepare for unmount
echo -e "${BLUE}🔄 Syncing filesystem...${NC}"
sync

echo -e "${GREEN}✅ Deployment complete!${NC}"
echo
echo "The Pico should automatically restart and load the new firmware."
echo "You can safely disconnect the USB cable now, or run:"
echo "   $PICO_MOUNT_SCRIPT unmount"
echo
echo -e "${YELLOW}Note: The Pico will automatically unmount once it starts running the firmware.${NC}"
