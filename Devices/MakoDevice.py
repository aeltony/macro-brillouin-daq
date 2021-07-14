import Devices.BrillouinDevice
import time
from Devices.pymba import *
from PupilDetection import *
import cv2
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import numpy as np

# This is the CMOS camera. Included in this file in the processing class
# is pupil tracking.

class MakoDevice(Devices.BrillouinDevice.Device):

    # This class always runs, so it takes app as an argument
    def __init__(self, stop_event, app):
        super(MakoDevice, self).__init__(stop_event)   #runMode=0 default
        self.deviceName = "Mako"
        self.camera = None
        self.vimba = Vimba()
        self.set_up()
        self.mako_lock = app.mako_lock
        self.runMode = 0    #0 is free running, 1 is scan
        self.camera.ExposureTimeAbs = 5000    # us
        self.imageHeight = 1000
        self.imageWidth = 1000
        self.bin_size = 2
        self.camera.Height = self.imageHeight # max: 2048
        self.camera.Width = self.imageWidth # max: 2048
        self.camera.OffsetX = 524
        self.camera.OffsetY = 524
        self.camera.startCapture()
        self.camera.runFeatureCommand('AcquisitionStart')
        self.frame.queueFrameCapture()

    # set up default parameters
    def set_up(self):
        self.vimba.startup()
        system = self.vimba.getSystem()

        if system.GeVTLIsPresent:
            system.runFeatureCommand("GeVDiscoveryAllOnce")
            time.sleep(0.2)
        camera_ids = self.vimba.getCameraIds()

        print("[MakoDevice] CMOS camera found: ",camera_ids)
        self.camera = self.vimba.getCamera(camera_ids[0])
        self.camera.openCamera()
        # list camera features
        # cameraFeatureNames = self.camera.getFeatureNames()
        # for name in cameraFeatureNames:
        #     print('Camera feature:', name)

        self.camera.AcquisitionMode = 'Continuous'
        #print("Frame rate limit: ")
        #print(self.camera.AcquisitionFrameRateLimit)
        #print(self.camera.AcquisitionFrameRateAbs)

        self.frame = self.camera.getFrame()
        self.frame.announceFrame()

    def __del__(self):
        return

    def shutdown(self):
        print("[MakoDevice] Closing Device")
        try:
            self.camera.runFeatureCommand('AcquisitionStop')
            self.camera.endCapture()
            self.camera.revokeAllFrames()
            self.vimba.shutdown()
        except:
            print("[MakoDevice] Not closed properly")

    # getData() acquires an image from Mako
    def getData(self):
        with self.mako_lock:
            self.frame.waitFrameCapture(100000)
            self.frame.queueFrameCapture()
            imgData = self.frame.getBufferByteData()
            image_arr = np.ndarray(buffer = imgData,
                           dtype = np.uint8,
                           shape = (self.frame.height,self.frame.width))
            image_arr = image_arr.reshape((self.frame.height//self.bin_size, self.bin_size, \
                self.frame.width//self.bin_size, self.bin_size)).max(3).max(1)
        # print("[MakoDevice] frame acquired, queue = %d" % self.dataQueue.qsize())
        return image_arr

    def setExpTime(self, expTime):
        #print('[MakoDevice] setExpTime got called with value=', expTime)
        with self.mako_lock:
            self.camera.ExposureTimeAbs = expTime*1e3
        #self.changeSetting(self.mako_lock, lambda:self.camera.ExposureTimeAbs.SetValue(expTime*1e6))
        #print("[MakoDevice] Exposure time set to %.3f ms" % expTime)

    def setFrameRate(self, frameRate):
        #print('[MakoDevice] setFrameRate got called with value=', frameRate)
        with self.mako_lock:
            self.camera.AcquisitionFrameRateAbs = frameRate
        #self.changeSetting(self.mako_lock, lambda:self.camera.AcquisitionFrameRateAbs.SetValue(frameRate))
        #print("[MakoDevice] Frame rate set to %.3f Hz" % frameRate)

# This class does the computation for free running mode, mostly displaying
# to the GUI
class MakoFreerun(Devices.BrillouinDevice.DeviceProcess):
    updateCMOSImageSig = pyqtSignal('PyQt_PyObject')

    def __init__(self, device, stopProcessingEvent, finishedTrigger = None):
        super(MakoFreerun, self).__init__(device, stopProcessingEvent, finishedTrigger)

    # data is an numpy array of type int32
    def doComputation(self, data):

        image = np.flip(data.transpose((1,0)),1)
        self.updateCMOSImageSig.emit(image)
        return image