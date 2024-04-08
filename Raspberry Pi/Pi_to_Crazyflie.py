import logging
import struct
import sys
import threading
import time
import numpy as np

import cflib.crtp
import cflib.cpx
from cflib.cpx.transports import UARTTransport
from cflib.cpx import CPXFunction
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.utils import uri_helper

def main():
 cflib.crtp.init_drivers(enable_serial_driver=True)

 transport = UARTTransport('/dev/ttyS0', 115200)
 cpxRaspberry = cflib.cpx.CPX(transport)
 try:
     packet = cflib.cpx.CPXPacket()
     packet.destination = cflib.cpx.CPXTarget.STM32
     packet.function = cflib.cpx.CPXFunction.APP

     num_frames = 10

     for frame in range(1, num_frames + 1):
         packet.data = [frame]  
         print("Sending Frame:", frame)
         cpxRaspberry.sendPacket(packet)
         time.sleep(1)  
         #uart.readPacket()
     print("All frames sent successfully!")

 except Exception as e:
     print("An error occurred:", str(e))


if __name__ == "__main__":
 main()
