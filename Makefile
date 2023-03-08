TARGET=Proteus
export TARGET

SPECS = nosys.specs

build:
	$(MAKE) ARGS="-O2 -ffunction-sections -fdata-sections -fno-exceptions -c -fmessage-length=0 -Wno-psabi -specs=$(SPECS)" -C fehproteusfirmware all

clean:
	$(MAKE) -C fehproteusfirmware clean

ifeq ($(OS),Windows_NT)
deploy: build
	$(MAKE) -C fehproteusfirmware deploy
else
UNAME_S := $(shell uname -s)
deploy: build
ifeq ($(UNAME_S),Darwin)
	$(MAKE) -C fehproteusfirmware deploy
else
	sudo cp $(TARGET).s19 $(SD_PATH)/CODE.S19 && sudo umount $(SD_PATH)
endif
endif
