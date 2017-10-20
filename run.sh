#!/bin/bash

# Set library path, why isn't this default somewhere?
export LD_LIBRARY_PATH=/usr/local/lib

# Turn off Auto Exposure
uvcdynctrl -s "Exposure" 250
uvcdynctrl -s "Auto Exposure" 1
uvcdynctrl -s "Exposure" 250
# Turn off Auto Gain and White Balance
uvcdynctrl -s 'Gain, Automatic' 0
uvcdynctrl -s 'White Balance, Automatic' 0
uvcdynctrl -s 'White Balance, Automatic' 1


# Run the server
cd "$(dirname "$0")"
cd build
./AmbiLightServer
