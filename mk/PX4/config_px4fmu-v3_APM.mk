#
# Makefile for the px4fmu-v3_APM configuration
#
include $(SKETCHBOOK)/mk/PX4/px4_common.mk

MODULES		+= drivers/lsm303d
MODULES		+= drivers/l3gd20
#MODULES		+= drivers/mpu9250
MODULES		+= drivers/boards/px4fmu-v3
MODULES		+= drivers/pwm_input
#MODULES         += modules/uavcan
MODULES         += lib/mathlib
#MODULES		+= drivers/px4io
#MODULES		+= drivers/px4flow
MODULES		+= drivers/rgbled_gpio
#MODULES		+= drivers/oreoled