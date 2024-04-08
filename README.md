# Crazyflie_Jevois_RaspberryPi
This repository is a few notes of caution when trying to connect Crazyflie with Jevois or Raspberry Pi. The main steps run on a Raspberry Pi with an operating system similar to Jevois. And the specification will be given when the steps turn to different on Jevois. 

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
   You also can find the revised example cfg file in this repository [Crazyflie_Jevois_RaspberryPi/Crazyflie/app_stm_jevois_cpx/app-config](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Crazyflie/app_stm_jevois_cpx/app-config)
## Pyserial configuration
When you use `with SyncCrazyflie(URI) as scf:` to connect with Crazyflie, It will check if the list returned by `serial.tools.list_ports` contains the URI. However, on both Jevois and Raspberry pi, when you use `python3 -m serial.tools.list_ports -v` to look at the ports, pyserial filters out the `ttysS0` port. The reason is clearly stated under this [topic](https://github.com/pyserial/pyserial/issues/489). And you also could check this [discussion](https://github.com/orgs/bitcraze/discussions/1224) in Crazyflie repository which gives a complete description of the cause and solution of the error. Please remeber to reinstall Pyserial after you change its' source code.

##  Crazyflie-lib-python
This is a [python library to communicate with Crazyflie](https://github.com/bitcraze/crazyflie-lib-python). WHen you run `SyncCrazyflie(URI) as scf:` in [this tutorial](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/development/uart_communication/). It will call the [self.cpx = CPX(UARTTransport(device, 576000))](https://github.com/bitcraze/crazyflie-lib-python/blob/a77b4023867c27d814c6820373ec2a7d158a2ef5/cflib/crtp/serialdriver.py#L94) with the default baudrate of 576000. This baudrate can not change automatically. So if you modify baudrated fo the UART, you should manually replace 576000 here. Otherwise, the scripy will hang at  `SyncCrazyflie(URI) as scf:` with information printed on the command line `Connecting to UART on /dev/ttyS0 @ 576000`. Please remeber to reinstall Crazyflie-lib-python after you change its' source code.

## CPX in Raspberry Pi
We use The Crazyflie Packet eXchange protocol (CPX) to enable communications between Crazyflie 2.0 and the host. Here is the example on Raspberry PI and Crazyflie 2.0. We will use this two scripts to complete the work that ]Raspberry Pi send the frame number](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Raspberry%20Pi/Pi_to_Crazyflie.py) and [Crazyflie will receive and print it](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Crazyflie/app_stm_jevois_cpx/stm_jevois_cpx.c).

## CPX in Jevois
The script used in Jevois to send data to Crazyflie is in [Crazyflie_Jevois_RaspberryPi/Crazyflie/app_stm_jevois_cpx/stm_jevois_cpx.c](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Crazyflie/app_stm_jevois_cpx/stm_jevois_cpx.c). Because Jevois will processNoUSB() is called on every frame, it will run something we can't see between two frames, i.e., after one frame and before the later frame. I have tested many times that if we do not add [`elf.SerialSend.reset_output_buffer()`](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/ed8307df3d564f15fc908739c47cf7a9afac05d3/JeVois/PythonTest.py#L47C10-L47C46) at the beginning of processNoUSB, the crazyflie always recieved only the first number [1] and report the UART error. It's just like Jevois write something into the buffer. But we cann't see that even if I use [`jevois.LINFO('buff is {}'.format(buff))`](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/ed8307df3d564f15fc908739c47cf7a9afac05d3/JeVois/PythonTest.py#L60C9-L60C48) to show every buffers. The result shown that they are totaly same.
