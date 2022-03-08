# nRF9160 DK Balancing Robot

This is a project that aims to port a balancing robot project written for the nRF52832 DK using the nRF5 SDK v13, to the nRF9160 DK using the nRF Connect SDK.

## Project setup
- Clone this project into any folder that has an [nRF Connect SDK installation](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/latest/nrf/gs_assistant.html). I used ncs v1.9.0, but I think anything after v1.6.0 should work (pre-v1.6.0 has no support for the IMU, MPU9250).
- Build and flash the project as you would with any sample. Make sure that the physical switch `SW10` on the DK is set to the chip you want to flash to.

### nRF9160
Implements most (all?) of the functionalities that was present in the nRF52832 sample. Except bluetooth, perhaps?


### nRF52840
This chip is currently not in use, but some gpio pin routing settings might cause LEDs on the DK to light up when the motor board is connected. If this is a problem, feel free to build+flash the basic program in `nRF52840` which disables the pin routings.