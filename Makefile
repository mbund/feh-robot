FEHSD_DEVICE=/dev/sda1
TARGET=Proteus
export TARGET

build:
	$(MAKE) -C fehproteusfirmware all

clean:
	$(MAKE) -C fehproteusfirmware clean

ifeq ($(OS),Windows_NT)
deploy:
	$(MAKE) -C fehproteusfirmware deploy
else
UNAME_S := $(shell uname -s)
deploy:
ifeq ($(UNAME_S),Darwin)
	$(MAKE) -C fehproteusfirmware deploy
else
	sudo mkdir -p /media/FEHSD
	sudo mount $(FEHSD_DEVICE) /media/FEHSD
	sudo cp *.s19 /media/FEHSD/CODE.S1
	sudo umount $(FEHSD_DEVICE)	
endif
endif
