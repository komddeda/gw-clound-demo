CROSS_COMPILE := /work/yx_prj/rk3506/rk3506_linux6.1_sdk/buildroot/output/rockchip_rk3506/host/bin/arm-buildroot-linux-gnueabihf-
SYSROOT       := /work/yx_prj/rk3506/rk3506_linux6.1_sdk/buildroot/output/rockchip_rk3506/host/arm-buildroot-linux-gnueabihf/sysroot

CC      := $(CROSS_COMPILE)gcc
CFLAGS  := --sysroot=$(SYSROOT) \
           -I$(SYSROOT)/usr/include \
           -Os -Wall \
           -ffunction-sections -fdata-sections

PAHO_SO := $(notdir $(firstword $(wildcard $(SYSROOT)/usr/lib/libpaho-mqtt3c.so*)))
LDFLAGS := --sysroot=$(SYSROOT) \
           -L$(SYSROOT)/usr/lib \
           -Wl,--gc-sections \
           -l:$(PAHO_SO) \
           -lpthread

SRCS := mqtt_gateway_demo.c mqtt_gateway_mqtt.c mqtt_gateway_modbus.c \
        mqtt_gateway_tailer.c mqtt_gateway_telemetry.c cJSON.c
OBJS := $(SRCS:.c=.o)

all: mqtt_gateway_demo

mqtt_gateway_demo: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) mqtt_gateway_demo
