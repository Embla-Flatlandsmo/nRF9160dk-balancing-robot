# nRF9160 DK Balancing Robot

This is a project that aims to port a balancing robot project written for the nRF52832 DK using the nRF5 SDK v13, to the nRF9160 DK using the nRF Connect SDK.

## Building
To build a project, follow [the official nRF Connect SDK guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_programming.html)
## Project setup (Using existing nRF Connect SDK installation)
- Clone this project into any folder that has an [nRF Connect SDK installation](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_assistant.html). I used ncs v1.9.0, but I think anything after v1.6.0 should work (pre-v1.6.0 has no support for the IMU, MPU9250).
- Build and flash the project as you would with any sample. Make sure that the physical switch `SW10` on the DK is set to the chip you want to flash to.

## Project setup (New nRF Connect SDK installation)
- Ensure all required software for building nRF Connect SDK v1.9.0 is installed. A list of required software and appropriate versions can be found at https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.0/nrf/gs_recommended_versions.html.
- Create project directory
- Inside project directory, run command ```west init -m https://github.com/Embla-Flatlandsmo/nRF9160dk-balancing-robot.git```. This will set up the west configurations for the project directory.
- In the same directory, run ```west update```. This will download the project files, nRF Connect SDK v1.9.0, and their dependencies.
- Navigate to folder containing firmware for the microcontroller you want to work with (```./balancing_robot_firmware/<MCU NAME>```) and use west commands for building and flashing the firmware as described in the [developer guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.0/zephyr/guides/west/build-flash-debug.html#west-build-flash-debug).

### nRF9160
Implements most (all?) of the functionalities that was present in the original nRF52832 sample. Except bluetooth, perhaps?
This is still a work in progress, so far the only thing implemented is IMU messages.

### nRF52840
This chip is currently not in use, but some gpio pin routing settings might cause LEDs on the DK to light up when the motor board is connected. If this is a problem, feel free to build+flash the basic program in `nRF52840` which disables the pin routings.

# Notes:
`CONFIG_LOG_MODE_IMMEDIATE` is required to log doubles (which is what the IMU module does). Be aware that it's blocking (and slow), so it can affect timing-critical parts of the application. If this becomes an issue down the road, feel free to disable it (the program will work, it's just that accelerometer/gyro log outputs will be all zero).

## Useful links
- [MPU9250 (IMU) datasheet](https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf) ()
