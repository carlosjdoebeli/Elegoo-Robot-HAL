import time
from pni_libs.helpers import *
import microcontroller as mc
import board
from digitalio import DigitalInOut, Direction, Pull
import pulseio

class constants:

    LOW = 0
    HIGH = 1

    CM = 28
    INC = 71

    TIMEOUT = 20 # in milliseconds

    # default pins on elegoo v3 bot
    TRIGPIN = board.A5
    ECHOPIN = board.A4

    # distance at which object is considered present
    RANGE = 0.03

class ultra:

    def __init__(self, trigPin = constants.TRIGPIN, echoPin = constants.ECHOPIN):
        self.trig = trigPin
        self.echo = echoPin
        if (self.trig == self.echo):
            self.threePins = True
        else:
            self.threePins = False

        self.trigdig = DigitalInOut(self.trig)
        self.trigdig.direction = Direction.OUTPUT

        self.echodig = DigitalInOut(self.echo)
        self.echodig.direction = Direction.INPUT

        self.previousmillis = 0
        self.timeout = constants.TIMEOUT

    def update(self):
        # This will be called by the car control module
        # once every loop, use this function to do
        # any task that needs to happen in the background
        pass


    def timing(self):
        if (self.threePins):
            self.trigdig.direction = digitalio.Direction.OUTPUT

        self.trigdig.value = constants.LOW
        mc.delay_us(2)
        self.trigdig.value = constants.HIGH
        mc.delay_us(10)
        self.trigdig.value = constants.LOW

        if (self.threePins):
            self.trigdig.direction = digitalio.Direction.INPUT

        self.previousmillis = millis()
        while(self.echodig.value == constants.LOW):
            if((millis() - self.previousmillis) > self.timeout):
                break
            else:
                continue

        self.previousmillis = millis()
        while(self.echodig.value == constants.HIGH):
            if((millis() - self.previousmillis) > self.timeout):
                break
            else:
                continue

        return millis() - self.previousmillis


    # If the unit of measure is not passed as a parameter,
    # by default, it will return the distance in centimeters.
    # To change the default, replace CM by INC.
    def read(self, und = constants.CM):
        return self.timing() / und / 2 # distance by divisors


    # This method is too verbal, so, it's deprecated.
    # Use read() instead.

    def get_obj(self, und = constants.CM):
        return self.read(und)

    # Return true or false value if object is present

    def is_there_obj(self, distance = constants.RANGE):
        return (self.get_obj() < distance)
