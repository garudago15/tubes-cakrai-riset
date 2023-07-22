APP_TARGET_NAME=testing-constant-PWM-motor
BOARD_SERIES=NUCLEO_F446RE
MBED_OS_PATH="../../mbed-os"

# if using wsl
# sudo mkdir /mnt/d/
# sudo mount -t drvfs d: /mnt/d -o uid=$(id -u $USER),gid=$(id -g $USER),metadata
MIKON_PATH=/mnt/d/

# if using window powershell
# MIKON_PATH=D:\ 
# copy cmake_build/$(BOARD_SERIES)/develop/GCC_ARM/$(APP_TARGET_NAME).bin $(MIKON_PATH)


all: configure build

run: fast_build copy

configure:
	mbed-tools configure -m $(BOARD_SERIES) -t GCC_ARM --mbed-os-path $(MBED_OS_PATH)

build:
	cmake -S . -B cmake_build/$(BOARD_SERIES)/develop/GCC_ARM -GNinja
	ninja -C cmake_build/$(BOARD_SERIES)/develop/GCC_ARM

fast_build:
	ninja -C cmake_build/$(BOARD_SERIES)/develop/GCC_ARM

copy:
	cp cmake_build/$(BOARD_SERIES)/develop/GCC_ARM/$(APP_TARGET_NAME).bin $(MIKON_PATH)