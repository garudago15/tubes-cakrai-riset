# tubes-cakrai-riset

<h2>To Do List</h2>

```
TO DO:
- Testing Motor
    - Menentukan PPR Motor
    - Testing PID
        - Menentukan Kp, Ki, Kd untuk tiap motor
    - Testing SMC
        - Mempelajari cara makainya
    - Menentukan TS yang tepat untuk PID dan SMC
- Testing Shooter (Rangkaian Only)
    - Test Motor Fly (Flywheel)
    - Test Motor Rld (Dorong bola ke flywheel)
    - Test Motor Ang (Pengatur sudut shooter)
    - Test Servo Chamber
- Testing Omniwheel (Rangkaian Only)
- Testing Interrupt (Rangkaian Only)
    - Efisiensi fungsi callback
    - Make sure no error
- Testing LED RGB
    - Control RGB sesuai state
- Testing LIDAR
    - Buat fungsi getDist() return jarak
```

```
TO DO After Mekanik:
- Testing Omniwheel Robot
- Testing Kalkulasi Shooter (Testing (Sudut, RPM) => (Jarak))
- Testing Shooter (Shoot, ToggleFlywheel, ToggleChamber)
```


<h2>File Structure</h2>

```
tubes-cakrai-riset
|- mbed-os
|- libs
    |- Configs
        |- Constants.h
    |- Shooter
        |- Shooter.h
        |- Shooter.cpp
    |- servoKRAI
        |- servoKRAI.h
        |- servoKRAI.cpp
    |- ...
|- projects
    |-  testing-shooter
        |- main.cpp
        |- CMakeLists.txt
        |- ...
    |- testing-omni
        |- main.cpp
        |- CMakeLists.txt
        |- ...
    |- ...

```

<h2>Note</h2>

```
- Jangan lupa path di CMakeList dan include "../../libs/...".
- Untuk menghindari conflict, tetap memakai branch ya.
- Jika diatas ada yang kurang jelas / mau diganti bilang aja ya =V
```