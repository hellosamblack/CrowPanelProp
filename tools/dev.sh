#!/usr/bin/env bash
# Dev helper for the communicator prop firmware (Linux / macOS / ESP-IDF).
# Activates the IDF environment if needed, forces UTF-8, and runs build/flash/monitor.
#
# Usage (from anywhere):
#   ./tools/dev.sh build
#   ./tools/dev.sh flash   -Port /dev/ttyUSB0
#   ./tools/dev.sh bf      -Port /dev/ttyUSB0   # build + flash
#   ./tools/dev.sh bfw     -Port /dev/ttyUSB0   # build + flash + wait until API answers
#   ./tools/dev.sh monitor -Port /dev/ttyUSB0
#   ./tools/dev.sh ota                          # build + OTA push to mDNS host
#   ./tools/dev.sh ota    -DeviceHost 172.17.2.167 # explicit IP
#
# After flashing, drive/inspect the UI with tools/prop.py:
#   python tools/prop.py shot out.png --screen spectrum --wait

ACTION="build"

# Detect default serial port by looking for CH340/CH341 (vendor 1a86)
detect_ch340_port() {
    # 1. Check /dev/serial/by-id for a matching 1a86 / CH340 / CH341 string (working usbipd / native Linux)
    if [ -d "/dev/serial/by-id" ]; then
        for link in /dev/serial/by-id/*; do
            if [ -e "$link" ]; then
                local link_lower
                link_lower=$(basename "$link" | tr '[:upper:]' '[:lower:]')
                if [[ "$link_lower" =~ 1a86 || "$link_lower" =~ ch340 || "$link_lower" =~ ch341 ]]; then
                    local target
                    target=$(readlink -f "$link")
                    if [ -e "$target" ]; then
                        echo "$target"
                        return 0
                    fi
                fi
            fi
        done
    fi

    # 2. Scan active /dev/ttyUSB* or /dev/ttyACM* using sysfs (check vendor 1a86)
    for path in /sys/class/tty/ttyUSB* /sys/class/tty/ttyACM*; do
        if [ -d "$path" ]; then
            local dev_path
            dev_path=$(readlink -f "$path/device")
            local usb_dev_path
            usb_dev_path=$(dirname "$(dirname "$dev_path")")
            if [ -f "$usb_dev_path/idVendor" ]; then
                local vendor
                vendor=$(cat "$usb_dev_path/idVendor" 2>/dev/null)
                if [ "$vendor" = "1a86" ]; then
                    local dev_name
                    dev_name=$(basename "$path")
                    echo "/dev/$dev_name"
                    return 0
                fi
            fi
        fi
    done

    # 3. Check if we are in WSL and powershell.exe is available (WSL serial fallback - COM ports mapping)
    if grep -qi Microsoft /proc/version 2>/dev/null && command -v powershell.exe >/dev/null 2>&1; then
        local win_name
        win_name=$(powershell.exe -NoProfile -Command "Get-CimInstance Win32_PnPEntity | Where-Object { \$_.Name -like '*CH340*' -or \$_.Caption -like '*CH340*' -or \$_.Name -like '*CH341*' } | Select-Object -First 1 -ExpandProperty Name" 2>/dev/null | tr -d '\r')
        if [[ "$win_name" =~ \((COM([0-9]+))\) ]]; then
            local com_num="${BASH_REMATCH[2]}"
            echo "/dev/ttyS${com_num}"
            return 0
        fi
    fi

    # 4. Final fallback: return first existing ttyUSB or ttyACM, or default to /dev/ttyUSB0
    if [ -e "/dev/ttyUSB0" ]; then
        echo "/dev/ttyUSB0"
    elif [ -e "/dev/ttyACM0" ]; then
        echo "/dev/ttyACM0"
    else
        echo "/dev/ttyUSB0"
    fi
}

PORT=$(detect_ch340_port)

DEVICE_HOST="comm-unit-7.local"
TOKEN="prop-ota-2024"

# Get project path relative to this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJ_DIR="$(dirname "$SCRIPT_DIR")"

# Activate ESP-IDF environment if idf.py is not in PATH
if ! command -v idf.py >/dev/null 2>&1; then
    export_paths=(
        "$HOME/.local/esp/esp-idf/export.sh"
        "$HOME/esp/esp-idf/export.sh"
        "$IDF_PATH/export.sh"
    )
    activated=false
    for ep in "${export_paths[@]}"; do
        if [ -n "$ep" ] && [ -f "$ep" ]; then
            echo "Activating ESP-IDF environment using $ep..."
            # Source export.sh in the current shell
            . "$ep"
            activated=true
            break
        fi
    done
    if [ "$activated" = false ]; then
        echo "Warning: idf.py not found in PATH and export.sh was not found in standard paths."
        echo "Please ensure ESP-IDF is installed and either activated or available at ~/esp/esp-idf or ~/.local/esp/esp-idf."
    fi
fi

# Force UTF-8 encoding
export PYTHONIOENCODING="utf-8"
export PYTHONUTF8="1"

# Handle arguments
if [ $# -ge 1 ]; then
    case "$1" in
        build|flash|bf|bfw|monitor|reconfigure|ota|list)
            ACTION="$1"
            shift
            ;;
    esac
fi

while [ $# -gt 0 ]; do
    case "$1" in
        -Port|-port|-p|--port)
            PORT="$2"
            shift 2
            ;;
        -DeviceHost|-devicehost|-d|--device)
            DEVICE_HOST="$2"
            shift 2
            ;;
        -Token|-token|-t|--token)
            TOKEN="$2"
            shift 2
            ;;
        *)
            if [ -z "$ACTION" ]; then
                ACTION="$1"
                shift
            else
                echo "Unknown argument: $1"
                exit 1
            fi
            ;;
    esac
done

if command -v python3 >/dev/null 2>&1; then
    PYTHON_EXE="python3"
else
    PYTHON_EXE="python"
fi

case "$ACTION" in
    build)
        idf.py -C "$PROJ_DIR" build
        ;;
    flash)
        idf.py -C "$PROJ_DIR" -p "$PORT" flash
        ;;
    bf)
        idf.py -C "$PROJ_DIR" build && idf.py -C "$PROJ_DIR" -p "$PORT" flash
        ;;
    bfw)
        idf.py -C "$PROJ_DIR" build && idf.py -C "$PROJ_DIR" -p "$PORT" flash && "$PYTHON_EXE" "$SCRIPT_DIR/prop.py" wait
        ;;
    monitor)
        idf.py -C "$PROJ_DIR" -p "$PORT" monitor
        ;;
    reconfigure)
        idf.py -C "$PROJ_DIR" reconfigure
        ;;
    ota)
        idf.py -C "$PROJ_DIR" build || exit $?
        BIN_FILE="$PROJ_DIR/build/communicator.bin"
        URL="http://$DEVICE_HOST/ota?token=$TOKEN"
        echo "OTA: pushing $BIN_FILE -> $URL"
        if command -v curl >/dev/null 2>&1; then
            curl -X POST "$URL" --data-binary "@$BIN_FILE" --fail || exit 1
        elif command -v wget >/dev/null 2>&1; then
            wget --post-file="$BIN_FILE" "$URL" -O - || exit 1
        else
            echo "Error: Neither curl nor wget is installed. Cannot perform OTA."
            exit 1
        fi
        echo ""
        echo "OTA: device is rebooting into new firmware"
        ;;
    list)
        echo "Available serial ports:"
        "$PYTHON_EXE" -m serial.tools.list_ports -v
        ;;
    *)
        echo "Unknown action: $ACTION"
        echo "Valid actions: build, flash, bf, bfw, monitor, reconfigure, ota, list"
        exit 1
        ;;
esac
