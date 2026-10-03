PS5_HOST ?= ps5
PS5_PORT ?= 9021

HOME_SDK := $(HOME)/.local/opt/ps5-payload-sdk
HOME_LLVM := $(HOME)/.local/opt/llvm-18/usr/lib/llvm-18

ifeq ($(origin PS5_PAYLOAD_SDK), undefined)
ifneq ($(wildcard $(HOME_SDK)/toolchain/prospero.mk),)
PS5_PAYLOAD_SDK := $(HOME_SDK)
endif
endif

ifneq ($(wildcard $(HOME_LLVM)/bin/llvm-config),)
export PATH := $(HOME_LLVM)/bin:$(PATH)
export LLVM_CONFIG := $(HOME_LLVM)/bin/llvm-config
export LD_LIBRARY_PATH := $(HOME_LLVM)/lib:$(HOME)/.local/opt/llvm-18/usr/lib/x86_64-linux-gnu:$(LD_LIBRARY_PATH)
endif

ifdef PS5_PAYLOAD_SDK
include $(PS5_PAYLOAD_SDK)/toolchain/prospero.mk
endif

ELF := YetAnotherControllerEnabler.elf
SRC := src/main.c src/hid_map.c src/usb_switch.c src/pad_assign.c src/bt_switch.c
HOST_CC ?= cc

CFLAGS := -Wall -Wextra -Werror -g -Iinclude
LDLIBS := -lSceIpmi -lSceSysmodule -lSceUserService -lScePad

.PHONY: all clean test host-test

all: $(ELF)

$(ELF): $(SRC)
ifndef PS5_PAYLOAD_SDK
	$(error PS5_PAYLOAD_SDK is undefined. Install https://github.com/ps5-payload-dev/sdk and export PS5_PAYLOAD_SDK)
endif
	$(CC) $(CFLAGS) -o $@ $(SRC) $(LDLIBS)

host-test:
	$(HOST_CC) $(CFLAGS) -o tests/test_hid_map tests/test_hid_map.c src/hid_map.c
	./tests/test_hid_map

clean:
	rm -f $(ELF) tests/test_hid_map

ifdef PS5_PAYLOAD_SDK
test: $(ELF)
	$(PS5_DEPLOY) -h $(PS5_HOST) -p $(PS5_PORT) $(ELF)
endif
