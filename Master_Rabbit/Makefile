# Windows
# NUCLEO_F446RE , ARCH_MAX
# NUCLEO =  NUCLEO_F446RE
NUCLEO =  ARCH_MAX
PROGRAM = Master_Elephant2
DRIVE_NAME = NODE_F446RE
GET_DRIVE_CMD = $(shell wmic volume where "label='$(DRIVE_NAME)'" get driveletter)
DRIVE_LETTER = $(word 2, $(GET_DRIVE_CMD))


configure:
	mbed-tools configure -m $(NUCLEO) -t GCC_ARM --mbed-os-path "../mbed-os"

cmake:
	cmake -S . -B cmake_build/$(NUCLEO)/develop/GCC_ARM -GNinja

ninja:
	ninja -C cmake_build/$(NUCLEO)/develop/GCC_ARM

clean:
	rmdir /s /q cmake_build

reset:
	make clean
	make configure
	make cmake
	make ninja

full:
	make configure
	make cmake
	make ninja

# xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\Slave_Elephant.bin D:\ /I /Y /C /H /R
# wmic volume where "label='NODE_F446RE'" get driveletter
# @echo NUCLEO drive letter is $(DRIVE_LETTER)

# xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\$(PROGRAM).bin D:\

push:
	xcopy cmake_build\$(NUCLEO)\develop\GCC_ARM\$(PROGRAM).bin $(DRIVE_LETTER)\

np:
	make ninja
	make push