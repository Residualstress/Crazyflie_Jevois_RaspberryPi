# Crazyflie_Jevois_RaspberryPi
This repository is used to connect Crazyflie to Jevois or Raspberry Pi. The main steps run on a Raspberry Pi with an operating system similar to Jevois. And the specification will be given when the steps turn to different on Jevois. 

## Platform
1. Crazyflie 2.0 and Crazyflie Bolt
2. Jevois-A33
3. Raspberry Pi 4

## UART communication
Overall, the UART connection can be established by referring to [this tutorial](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/development/uart_communication/). However, there are still a lot of things to keep in mind and will be described below.

## Building and flashing on Crazyflie
Building and flashing are used several times during the configuration of the connection. [This official document](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/) can be used as a reference. In response to it, here are a few suggestions:
1. Among three ways, using the ARM tool chain is recommded for simple installation and faster compilation speed.
2. To build the firmware on crazyflie Bolt, you will need to configure it by
   ```
   make bolt_defconfig
   ```
   Or using [kbuild](https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/development/kbuild/)
   ```
   make menuconfig
   ```
   In Platform configuration-> Platform to build-> Build for Bolt.
   ![image](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/assets/92587824/c101d6d3-043b-45fe-adae-166baeb29a78)

4. To compile in App layer, both "make bolt_defconfig" and "make menuconfig" will faile. So you need to add parametes in app-config. For example, in [crazyflie-firmware/examples/app_hello_world/app-config](https://github.com/bitcraze/crazyflie-firmware/blob/master/examples/app_hello_world/app-config):
   ```
   CONFIG_ENABLE_CPX = y
   CONFIG_ENABLE_CPX_ON_UART2 = y
   CONFIG_CPX_UART2_BAUDRATE = 115200
   CONFIG_DECK_FORCE = "cpxOverUART2"
   CONFIG_DECK_CPX_HOST_ON_UART2 = y
   ```
   ![image](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/assets/92587824/cdfe669e-0270-43c7-9034-1bfdc7f128c0)

## Pyserial configuration
when you use `with SyncCrazyflie(URI) as scf:` to connect with Crazyflie, It will check if the list returned by `serial.tools.list_ports` contains the URI. However, on both Jevois and Raspberry pi, when you use `python3 -m serial.tools.list_ports -v` to look at the ports, pyserial filters out the `ttysS0` port. The reason is clearly stated under this [topic](https://github.com/pyserial/pyserial/issues/489). And you also could check this [discussion](https://github.com/orgs/bitcraze/discussions/1224) in Crazyflie repository which gives a complete description of the cause and solution of the error.
