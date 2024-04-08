import pyjevois
import time

if pyjevois.pro:
    import libjevoispro as jevois
else:
    import libjevois as jevois

import serial
import cflib.crtp
import cflib.cpx


class PythonTest:
    # ###################################################################################################
    ## Constructor
    def __init__(self):
        jevois.LINFO("PythonTest Constructor")
        self.frame = 0  # a simple frame counter used to demonstrate sendSerial()
        self.timer = jevois.Timer("pytest", 100, jevois.LOG_INFO)
        cflib.crtp.init_drivers(enable_serial_driver=True)
        self.packet = cflib.cpx.CPXPacket()
        self.packet.destination = cflib.cpx.CPXTarget.STM32
        self.packet.function = cflib.cpx.CPXFunction.APP
        self.frame = 1

        self.SerialSend = serial.Serial('/dev/ttyS0', 115200, timeout=2, write_timeout=2)


    # ###################################################################################################
    ## JeVois optional extra init once the instance is fully constructed
    def init(self):
        jevois.LINFO("PythonTest JeVois init")

        # Examples of adding user-tunable parameters to the module. Users can modify these using the JeVois console,
        # over a serial connection, using JeVois-Inventor, or using the JeVois-Pro GUI:
        pc = jevois.ParameterCategory("PythonTest Parameters", "")
        self.cx = jevois.Parameter(self, 'cx', 'int', "Circle horizontal center, in pixels", 320, pc)
        self.cy = jevois.Parameter(self, 'cy', 'int', "Circle vertical center, in pixels", 240, pc)
        self.radius = jevois.Parameter(self, 'radius', 'byte', "Circle radius, in pixels", 50, pc)


    # ###################################################################################################
    ## Process function with no USB output
    def processNoUSB(self, inframe):

        self.SerialSend.reset_output_buffer()
        self.packet.data = [1]
        data = self.packet.wireData
        if len(data) > 100:
            raise 'Packet too large!'
        buff = bytearray([0xFF, len(data)])
        buff.extend(data)
        checksum = 0
        for i in buff:
            checksum ^= i

        buff.extend([checksum])
        self.SerialSend.write(buff)
        jevois.LINFO('buff is {}'.format(buff))
        jevois.LINFO('SerialSend is {}'.format(self.SerialSend))
        self.frame += 1

        time.sleep(2)
        self.SerialSend.reset_output_buffer()
    # ###################################################################################################
    ## Process function with USB output
    def process(self, inframe, outframe):
        # Get the next camera image (may block until it is captured):
        inimg = inframe.get()
        if self.frame == 0:
            jevois.LINFO("Input image is {} {}x{}".format(jevois.fccstr(inimg.fmt), inimg.width, inimg.height))

        # Get the next available USB output image:
        outimg = outframe.get()
        if self.frame == 0:
            jevois.LINFO("Output image is {} {}x{}".format(jevois.fccstr(outimg.fmt), outimg.width, outimg.height))

        # Example of getting pixel data from the input and copying to the output:
        jevois.paste(inimg, outimg, 0, 0)

        # We are done with the input image:
        inframe.done()

        # Example of in-place processing:
        jevois.hFlipYUYV(outimg)

        # Example of simple drawings and of accessing parameter values (users can change them via console, JeVois
        # Inventor, or JeVois-Pro GUI):
        jevois.writeText(outimg, "Hi from Python!", 20, 20, jevois.YUYV.White, jevois.Font.Font10x20)

        jevois.drawCircle(outimg, self.cx.get(), self.cy.get(), self.radius.get(), 2, jevois.YUYV.White)

        # We are done with the output, ready to send it to host over USB:
        outframe.send()

        # Send a string over serial (e.g., to an Arduino). Remember to tell the JeVois Engine to display those messages,
        # as they are turned off by default. For example: 'setpar serout All' in the JeVois console:
        # if self.frame % 100 == 0:
        # jevois.sendSerial("DONE frame {}".format(self.frame));
        self.frame += 1

    # ###################################################################################################
    ## JeVois optional extra uninit before the instance is destroyed
    def uninit(self):
        jevois.LINFO("PythonTest JeVois uninit")
