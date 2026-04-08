CROSS_COMPILE := /work/yx_prj/rk3506/rk3506_linux6.1_sdk/buildroot/output/rockchip_rk3506/host/bin/arm-buildroot-linux-gnueabihf-
SYSROOT       := /work/yx_prj/rk3506/rk3506_linux6.1_sdk/buildroot/output/rockchip_rk3506/host/arm-buildroot-linux-gnueabihf/sysroot

CC      := $(CROSS_COMPILE)gcc
CFLAGS  := --sysroot=$(SYSROOT) \
           -I$(SYSROOT)/usr/include \
           -Os -Wall \
           -ffunction-sections -fdata-sections
LDFLAGS := --sysroot=$(SYSROOT) \
           -L$(SYSROOT)/usr/lib \
           -Wl,--gc-sections \
           -lcurl

SRCS := http_uploader_demo.c cJSON.c
OBJS := $(SRCS:.c=.o)

all: http_uploader_demo

http_uploader_demo: $(OBJS)
	$(CC) $(CFLAGS) $(OBJS) -o $@ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) http_uploader_demo
