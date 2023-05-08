# nRF9160 DK Balancing Robot

This is a project that aims to port a [balancing robot project written for the nRF52832 DK using the nRF5 SDK v13](https://github.com/MartinSivertsen/Hardware-Decoder-for-Balancer-SDK14.2), to the nRF9160 DK using the nRF Connect SDK.

The problem is that the hardware used in this project requires a REALLY tight tuning (if the balancing robot goes +-10 degrees from equilibrium, it cannot get back to the equilibrium) so the current robot does not balance. Stronger motors and/or larger wheels is needed to make it easier to balance this robot.

## Building
To build a project, follow [the official nRF Connect SDK guide](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.1.2/nrf/getting_started.html)
## Project setup (Using existing nRF Connect SDK installation)
- Clone this project into a folder that has an [nRF Connect SDK installation v2.1.2](https://developer.nordicsemi.com/nRF_Connect_SDK/doc/2.1.2/nrf/getting_started.html). I used ncs v2.1.2.
- Build and flash the project as you would with any sample. Make sure that the physical switch `SW10` on the DK is set to the chip you want to flash to.

### nRF9160
The nRF9160 implements the IMU and Encoder sampling and ties it all together in a control loop.

The project uses [zscilib](https://github.com/zephyrproject-rtos/zscilib/tree/0035be5e6a45e4ab89755b176d305d7a877fc79c) to get the device's orientation based on gyro and accelerometer values.

### nRF52840
This chip is currently not in use, but some gpio pin routing settings might cause LEDs on the DK to light up when the motor board is connected. If this is a problem, feel free to build+flash the basic program in `nRF52840` which disables the pin routings.

# Notes
- You might notice that the IMUs `MPU6050` and `MPU9250` are used a bit interchangably. These are pretty similar. A driver for MPU9250 was recently implemented (see [PR#40702](https://github.com/zephyrproject-rtos/zephyr/pull/40702)). Before that, MPU9250-support was "hidden" in the MPU6050 driver (as seen in [PR#32623](https://github.com/zephyrproject-rtos/zephyr/pull/32623))
- The IMU is not always found upon initialization. If the balancing does not start after 3 seconds, press the reset button on the DK.
- The robot uses a custom PCB whose schematics have been included in the `schematics` folder.

## Useful links
- [MPU9250 (IMU) datasheet](https://3cfeqx1hf82y3xcoull08ihx-wpengine.netdna-ssl.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf)
- [The original nRF5 SDK code](https://github.com/MartinSivertsen/Hardware-Decoder-for-Balancer-SDK14.2)
