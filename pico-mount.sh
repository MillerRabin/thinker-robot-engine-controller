#!/bin/bash

# Manual Pico Mount/Unmount Script
# Uses udisksctl for sudo-free mounting

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MOUNT_POINT=""

case "$1" in
    "mount")
        # Check if udisksctl is available
        if ! command -v udisksctl >/dev/null 2>&1; then
            echo "❌ udisksctl not found"
            echo "   Please install udisks2 package: sudo apt install udisks2"
            exit 1
        fi
        
        echo "🔍 Searching for Pico device..."
        
        # Look for Pico device
        PICO_DEVICE=$(lsblk -no NAME,LABEL,FSTYPE,SIZE | grep "RPI-RP2\|pico" | head -1)
        
        if [ -z "$PICO_DEVICE" ]; then
            echo "❌ No Pico found in mass storage mode"
            echo "   Make sure Pico is connected in BOOTSEL mode:"
            echo "   1. Hold BOOTSEL button"
            echo "   2. Connect USB cable"
            echo "   3. Release BOOTSEL button"
            exit 1
        fi
        
        # Extract device path (remove any tree characters)
        DEVICE_NAME=$(echo "$PICO_DEVICE" | awk '{print $1}' | sed 's/[^a-zA-Z0-9]//g')
        DEVICE_PATH="/dev/$DEVICE_NAME"
        
        echo "✅ Found Pico: $PICO_DEVICE"
        echo "📁 Device path: $DEVICE_PATH"
        
        # Check if already mounted
        EXISTING_MOUNT=$(udisksctl info -b "$DEVICE_PATH" 2>/dev/null | grep "MountPoints:" | awk '{print $2}')
        if [ -n "$EXISTING_MOUNT" ]; then
            echo "⚠️  Already mounted at: $EXISTING_MOUNT"
            MOUNT_POINT="$EXISTING_MOUNT"
        else
            # Mount using udisksctl
            echo "🔄 Mounting with udisksctl..."
            MOUNT_RESULT=$(udisksctl mount -b "$DEVICE_PATH" 2>&1)
            
            if [ $? -eq 0 ]; then
                # Extract mount point from udisksctl output
                MOUNT_POINT=$(echo "$MOUNT_RESULT" | grep -o '/media[^[:space:]]*' || echo "$MOUNT_RESULT" | grep -o '/run/media[^[:space:]]*')
                
                if [ -n "$MOUNT_POINT" ]; then
                    echo "✅ Successfully mounted to: $MOUNT_POINT"
                else
                    echo "❌ Failed to determine mount point"
                    echo "Mount result: $MOUNT_RESULT"
                    exit 1
                fi
            else
                echo "❌ Mount failed"
                echo "Error: $MOUNT_RESULT"
                exit 1
            fi
        fi
        
        # Show contents
        echo "📂 Contents:"
        ls -la "$MOUNT_POINT" | head -5
        ;;
        
    "unmount"|"umount")
        # Check if udisksctl is available
        if ! command -v udisksctl >/dev/null 2>&1; then
            echo "❌ udisksctl not found"
            echo "   Please install udisks2 package: sudo apt install udisks2"
            exit 1
        fi
        
        echo "🔄 Unmounting Pico..."
        
        # Find Pico device
        PICO_DEVICE=$(lsblk -no NAME,LABEL,FSTYPE,SIZE | grep "RPI-RP2\|pico" | head -1)
        
        if [ -z "$PICO_DEVICE" ]; then
            echo "⚠️  No Pico device found"
            exit 0
        fi
        
        # Extract device path
        DEVICE_NAME=$(echo "$PICO_DEVICE" | awk '{print $1}' | sed 's/[^a-zA-Z0-9]//g')
        DEVICE_PATH="/dev/$DEVICE_NAME"
        
        # Check if mounted
        MOUNT_POINT=$(udisksctl info -b "$DEVICE_PATH" 2>/dev/null | grep "MountPoints:" | awk '{print $2}')
        
        if [ -z "$MOUNT_POINT" ]; then
            echo "⚠️  Pico is not mounted"
            exit 0
        fi
        
        echo "Found mounted Pico at: $MOUNT_POINT"
        
        # Unmount using udisksctl
        UNMOUNT_RESULT=$(udisksctl unmount -b "$DEVICE_PATH" 2>&1)
        
        if [ $? -eq 0 ]; then
            echo "✅ Successfully unmounted"
        else
            echo "❌ Unmount failed"
            echo "Error: $UNMOUNT_RESULT"
            exit 1
        fi
        ;;
        
    "status")
        echo "=== Pico Mount Status ==="
        echo
        
        # Check if Pico is connected
        if lsusb | grep -q "2e8a"; then
            PICO_MODE=$(lsusb | grep "2e8a" | awk '{print $NF}')
            echo "✅ Pico detected: $PICO_MODE"
            
            if [[ "$PICO_MODE" == *"Boot"* ]]; then
                echo "   Mode: Bootloader (BOOTSEL mode)"
            else
                echo "   Mode: Mass Storage"
            fi
        else
            echo "❌ No Pico detected"
        fi
        
        echo
        
        # Check mount status for Pico devices
        PICO_DEVICE=$(lsblk -no NAME,LABEL,FSTYPE,SIZE | grep "RPI-RP2\|pico" | head -1)
        
        if [ -n "$PICO_DEVICE" ]; then
            DEVICE_NAME=$(echo "$PICO_DEVICE" | awk '{print $1}' | sed 's/[^a-zA-Z0-9]//g')
            DEVICE_PATH="/dev/$DEVICE_NAME"
            
            # Check if udisksctl is available for mount info
            if command -v udisksctl >/dev/null 2>&1; then
                MOUNT_POINT=$(udisksctl info -b "$DEVICE_PATH" 2>/dev/null | grep "MountPoints:" | awk '{print $2}')
                
                if [ -n "$MOUNT_POINT" ]; then
                    echo "✅ Mounted at: $MOUNT_POINT"
                    echo "   Contents:"
                    ls -la "$MOUNT_POINT" | head -5
                else
                    echo "❌ Not mounted"
                fi
            else
                echo "   (udisksctl not available for mount status)"
            fi
        else
            echo "❌ No Pico storage device found"
        fi
        ;;
        
    *)
        echo "Manual Pico Mount Control (sudo-free with udisksctl)"
        echo
        echo "Usage: $0 {mount|unmount|status}"
        echo
        echo "Commands:"
        echo "  mount    - Mount Pico (auto-detects device and mount point)"
        echo "  unmount  - Unmount Pico (auto-detects mounted device)"
        echo "  status   - Show current connection and mount status"
        echo
        echo "Note: Uses udisksctl for user-space mounting (no sudo required)"
        echo "      Mount points are automatically managed by the system"
        ;;
esac
