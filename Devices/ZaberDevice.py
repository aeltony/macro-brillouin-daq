from __future__ import division
import Devices.BrillouinDevice
import time
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import numpy as np
import queue as Queue
import threading
import zaber.serial as zs

# Zaber motor does not need to run in its own thread, so we don't implement a getData() method
class ZaberDevice(Devices.BrillouinDevice.Device):

	# This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(ZaberDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Zaber"
        self.enqueueData = False
        self.commandQueue = Queue.Queue()
        self.homeLoc = 0.

        self.port = zs.AsciiSerial("COM5", baud=115200, timeout = 20, inter_char_timeout = 0.05)
        self.device = zs.AsciiDevice(self.port, 1)
        self.z_axis = zs.AsciiAxis(self.device, 1)
        reply = self.z_axis.home()
        if self.checkReply(reply):
            print("[ZaberDevice] Z-axis homed")
        else:
            print("[ZaberDevice] Z-axis home failed")

        self.microstep_size = 0.047625 #Microstep resolution in um
        microstep_cmd = zs.AsciiCommand(1, "set resolution", 64)  #set microstep resolution
        self.device.send(microstep_cmd)
        print("[ZaberDevice] Microstep resolution set to %.6f um" % self.microstep_size)
        speed = 26
        speed_cmd = zs.AsciiCommand("set speed", int(speed/26*894455))  # 894455 = 26mm/s
        self.z_axis.send(speed_cmd)
        print("[ZaberDevice] Motor speed set to %f mm/s" % speed)
        acceleration_cmd = zs.AsciiCommand("set accel", 600)
        self.z_axis.send(acceleration_cmd)

        self.ZaberLock = app.ZaberLock

    def shutdown(self):
        with self.ZaberLock:
            self.port.close()
        print("[ZaberDevice] Closed")

    # Zaber doesn't do any data acquisition. This method sends out commands from the command queue
    # so they can be called asynchronously. Right now don't have a way to Get data, only set
    # command should be a tuple of ('method name', argument)
    def getData(self):
        try:
            cmd = self.commandQueue.get(block=True, timeout=1.0)
            method = getattr(self, cmd[0])
            if cmd[1] is None:
                method()
            else:
                method(cmd[1])
        except Queue.Empty:
            pass

    # Checks for warnings + errors from Zaber devices
    def checkReply(self, reply):
        if reply.reply_flag != "OK":
            print ("[ZaberDevice] Command rejected because: {}".format(reply.data))
            return False
        else: # Command was accepted
            return True

    def setMotorAsync(self, methodName, arg=[]):
        if (arg == []):
            self.commandQueue.put((methodName, None))
        else:
            self.commandQueue.put((methodName, arg[0]))

    # returns current position of motor, in um
    def updatePosition(self):
        #print('[ZaberDevice] updatePosition')
        with self.ZaberLock:
            try:
                reply_z = self.z_axis.send("get pos")
            except:
                print("[ZaberDevice] Motor busy")
                return self._lastPosition
            self._lastPosition = float(reply_z.data) * self.microstep_size
        return self._lastPosition

    # moves Zaber stages to home position
    def moveHome(self):
        self.moveAbs(self.homeLoc)

    # moves Zaber motor, called on by forwards and backwards buttons
    # distance in um
    def moveRelative(self, distance=0):
        with self.ZaberLock:
            reply = self.z_axis.move_rel(int(distance/self.microstep_size))
            if self.checkReply(reply)==False:
                print("[ZaberDevice] Z-axis moveRelative failed")
        self.updatePosition()

    # moves Zaber motor to a set location, called on above
    def moveAbs(self, pos=0):
        #print("[ZaberDevice] moveAbs called with pos %d encoder counts" % int(pos/self.microstep_size))
        with self.ZaberLock:
            reply = self.z_axis.move_abs(int(pos/self.microstep_size))
            if self.checkReply(reply)==False:
                print("[ZaberDevice] Z-axis moveAbs failed")
        self.updatePosition()