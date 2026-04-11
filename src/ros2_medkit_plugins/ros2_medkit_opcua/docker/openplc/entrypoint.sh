#!/bin/bash
set -e
cd /openplc

echo "Compiling PLC program..."
bash scripts/compile.sh 2>&1 || {
    echo "ERROR: PLC compilation failed"
    exit 1
}

if [ -f build/new_libplc.so ]; then
    mv build/new_libplc.so build/libplc_tank.so
    echo "PLC program compiled successfully"
else
    echo "ERROR: Compiled library not found"
    exit 1
fi

echo "Starting OpenPLC runtime..."
exec bash start_openplc.sh
