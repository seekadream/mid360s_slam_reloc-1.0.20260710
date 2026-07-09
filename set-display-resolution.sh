#!/bin/bash
sleep 5
export DISPLAY=:0
xrandr --output HDMI-1 --mode 1920x1080 --rate 60 2>/dev/null
