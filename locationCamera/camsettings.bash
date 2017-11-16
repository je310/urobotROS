#!/bin/bash
v4l2-ctl  --set-ctrl=brightness=-64 \
--set-ctrl=contrast=30 \
--set-ctrl=saturation=0 \
--set-ctrl=hue=0 \
--set-ctrl=white_balance_temperature_auto=0 \
--set-ctrl=gamma=100 \
--set-ctrl=gain=100 \
--set-ctrl=power_line_frequency=2 \
--set-ctrl=white_balance_temperature=4239 \
--set-ctrl=sharpness=1 \
--set-ctrl=backlight_compensation=3  \
--set-ctrl=exposure_absolute=120 \
--set-ctrl=exposure_auto_priority=1\
--set-ctrl=exposure_auto=1
