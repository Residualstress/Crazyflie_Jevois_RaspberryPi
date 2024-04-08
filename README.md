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

## Python package installation on Jevois
Python package installation comes to difficult when it is on Jevois because it is a embedded offline system. For basic python package installation, you can refer to this [tuotorial](http://jevois.org/doc/Change113log.html). Except from intalling by .whl file, we also can download the source code from packages' github repository and install it by typing `!python pip install /jevois/packages/pyesrial/` or `!python /jevois/packages/pyesrial/setup.py` in the console of JeVois.All above methods are only workable on only pure-python packages with no compiled dependencies. The reason are explained [here](http://jevois.org/qa/index.php?qa=2202&qa_1=can-other-python-libraries-be-installed-on-the-jevois). 

Note: Since the default time of jevois is in 1970, we need to change the system time to the present before installing the package with this command `date 040815242024` in the console of JeVois. `date 040815242024` means on 8 April 2024 at 3.15 p.m.

For the installation of Crazyflie-lib-python, I didn't install it in a right way because the cross-compile is too difficult for me to conduct. Since we only will use the source code related to the [CPX](https://github.com/bitcraze/crazyflie-lib-python/tree/master/cflib/cpx) in Crazyflie-lib-python. So I run `!python pip install /jevois/packages/cflib/` with revising the [`install_requires:` in setup.py](https://github.com/bitcraze/crazyflie-lib-python/blob/a77b4023867c27d814c6820373ec2a7d158a2ef5/setup.py#L39) to delete the packages which are not purly written in Python.  

Installing packages on jevois can be a long and painful process, as each package has requirements for other packages, so you need to satisfy all of them one by one! Usually, it will take the whole day to equiped your Jevois with the packages you need. 

## CPX in Raspberry Pi
We use The Crazyflie Packet eXchange protocol (CPX) to enable communications between Crazyflie 2.0 and the host. Here is the example on Raspberry PI and Crazyflie 2.0. We will use this two scripts to complete the work that ]Raspberry Pi send the frame number](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Raspberry%20Pi/Pi_to_Crazyflie.py) and [Crazyflie will receive and print it](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Crazyflie/app_stm_jevois_cpx/stm_jevois_cpx.c).

## CPX in Jevois
The script used in Jevois to send data to Crazyflie is in [Crazyflie_Jevois_RaspberryPi/Crazyflie/app_stm_jevois_cpx/stm_jevois_cpx.c](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/Crazyflie/app_stm_jevois_cpx/stm_jevois_cpx.c). Because Jevois will processNoUSB() is called on every frame, it will run something we can't see between two frames, i.e., after one frame and before the later frame. I have tested many times that if we do not add [`elf.SerialSend.reset_output_buffer()`](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/ed8307df3d564f15fc908739c47cf7a9afac05d3/JeVois/PythonTest.py#L47C10-L47C46) at the beginning of processNoUSB, the crazyflie always recieved only the first number [1] and report the UART error. It's just like Jevois write something into the buffer. But we cann't see that even if I use [`jevois.LINFO('buff is {}'.format(buff))`](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/ed8307df3d564f15fc908739c47cf7a9afac05d3/JeVois/PythonTest.py#L60C9-L60C48) to show every buffers, results shown that they are totaly same.

And the [/jevois/config/initscript.cfg](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/JeVois/config/initscript.cfg) and [/jevois/config/params.cfg](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/JeVois/config/params.cfg) is also need to be revised to enable PythonTest module run in [Headless mode](http://jevois.org/tutorials/UserHeadless.html).

In the end, You have to pay special attention to the format of the packet when transferring data to crazyflie. It must perfectly match [crazyflie's requirements](https://github.com/bitcraze/crazyflie-firmware/blob/cca276bc6e952307a0a81c475fc6856c776a4a1c/src/modules/src/cpx/cpx_uart_transport.c#L138). Otherwise, once the crazyflie receives a packet that fails to meet the requirements, it will report [an error](https://github.com/bitcraze/crazyflie-firmware/blob/cca276bc6e952307a0a81c475fc6856c776a4a1c/src/modules/src/cpx/cpx_uart_transport.c#L164) :
<img width="695" alt="8afb19ab04403d5a2bc122aa1d13acc" src="https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/assets/92587824/629e600a-630c-456b-9102-fbabcee80af8">

Sometimes it's not the data you're sending that's the problem, it's the underlying jevois program that's sending some data to crazyflie, so you need to [keep a tight rein on jevois's port outputs](http://jevois.org/doc/UserCli.html#:~:text=parameter.-,serout%20%3Cstring%3E%20%2D%20forward%20string%20to%20the%20serial%20port(s)%20specified%20by%20the%20serout%20parameter,-This%20operates%20like), as well as other related outputs. You can run `setpar serout None` in the console of JeVois or add lines in [/jevois/config/initscript.cfg](https://github.com/Residualstress/Crazyflie_Jevois_RaspberryPi/blob/main/JeVois/config/initscript.cfg).
