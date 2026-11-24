# Builds libinsn_count.so for the QEMU on this machine.
# Ubuntu 24.04 ships QEMU 8.2.2; change QEMU_TAG if yours differs
# (check with: qemu-system-arm --version).

QEMU_TAG ?= v8.2.2
HDR_URL   = https://raw.githubusercontent.com/qemu/qemu/$(QEMU_TAG)/include/qemu/qemu-plugin.h
GLIB      = $(shell pkg-config --cflags glib-2.0 2>/dev/null)

all: libinsn_count.so

include/qemu-plugin.h:
	mkdir -p include
	curl -sSfL -o $@ $(HDR_URL)

libinsn_count.so: insn_count.c include/qemu-plugin.h
	gcc -O2 -fPIC -shared -Iinclude $(GLIB) insn_count.c -o $@

clean:
	rm -rf include libinsn_count.so
