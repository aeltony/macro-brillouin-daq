import threading
import datetime
import numpy as np
import time
import sys
import os
import csv
from PyQt5 import QtGui, QtCore, QtWidgets
from PyQt5.QtCore import QTimer
import pyqtgraph as pg
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType
from pyqtgraphCustomize import *
import qt_ui # UI import
from ctypes import *
from configparser import ConfigParser
from Devices.SynthDevice import SynthDevice
from Devices.AndorDevice import AndorDevice, AndorProcessFreerun
from Devices.MakoDevice import MakoDevice, MakoFreerun
from Devices.TempSensorDevice import TempSensorDevice, TempSensorFreerun
from Devices.ZaberDevice import ZaberDevice
from Devices.ShutterDevice import ShutterDevice
from BrillouinScanManager import ScanManager
from ExperimentData import *
from BrillouinTreeModel import *

class CustomViewBox(pg.ViewBox):
    def __init__(self, *args, **kwds):
        pg.ViewBox.__init__(self, *args, **kwds)
        self.setMouseMode(self.RectMode)
        self.setBackgroundColor((255,255,255,0))
        
    ## reimplement right-click to zoom out
    def mouseClickEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            self.autoRange(padding=0)
            
    def mouseDragEvent(self, ev):
        if ev.button() == QtCore.Qt.RightButton:
            ev.ignore()
        else:
            pg.ViewBox.mouseDragEvent(self, ev)


class App(QtGui.QMainWindow,qt_ui.Ui_MainWindow):
 
    def __init__(self):
        super(App,self).__init__()
        self.setupUi(self)

        # load default/save parameter first
        self.configFilename = 'BrillouinScanConfig.ini'
        self.configParser = ConfigParser()
        self.configParser.read(self.configFilename)
        # TODO: validate configuration file, set defaults if configuration file is corrupted or unavailable
        self.maxScanPoints = 400  # Number of data points in view in freerunning mode
        self.maxRowPoints = 20 # Number of pixels per row in freerunning Brillouin map
        self.maxColPoints = 20 # Number of pixels per column in freerunning Brillouin map
        self.calPoints = 0 # Number of calibration points (used when scanning)
        laserX = self.configParser.getint('More Settings', 'laser_position_x')
        laserY = self.configParser.getint('More Settings', 'laser_position_y')
        FSR = self.configParser.getfloat('More Settings', 'FSR')
        SD = self.configParser.getfloat('More Settings', 'SD')
        RFpower = self.configParser.getfloat('Synth', 'RF_power')
        spectColumn = self.configParser.getint('Andor', 'spect_column')
        spectRow = self.configParser.getint('Andor', 'spect_row')

        self.params = [
            {'name': 'Scan', 'type': 'group', 'children': [
                {'name': 'Start Position', 'type': 'float', 'value': -500, 'suffix':' um', 'step': 100, 'limits': (-5000, 5000)},
                {'name': 'Step Size', 'type': 'float', 'value': 30, 'suffix':' um', 'step': 1, 'limits': (0, 1000), 'decimals':5},
                {'name': 'Frame Number', 'type': 'int', 'value': 40, 'step': 1, 'limits':(1, 2000)},
                {'name': 'End Position', 'type': 'float', 'value': 1200, 'suffix':' um', 'readonly': True, 'decimals':5},
                {'name': 'ToggleReference', 'type':'toggle', 'ButtonText':('Switch to Reference', 'Switch to Sample')},         #False=Sample="Switch to Ref"
                {'name': 'Scan/Cancel', 'type': 'action2', 'ButtonText':('Scan', 'Cancel')},
            ]},
            {'name': 'Display', 'type': 'group', 'children': [
                {'name': 'Colormap (min.)', 'type': 'float', 'value': 5.58, 'suffix':' GHz', 'step': 0.01, 'limits':(0.0, 22.0), 'decimals':3},
                {'name': 'Colormap (max.)', 'type': 'float', 'value': 5.82, 'suffix':' GHz', 'step': 0.01, 'limits':(0.1, 22.0), 'decimals':3}
            ]},
            {'name': 'Motor', 'type': 'group', 'children': [
                {'name': 'Current Location', 'type': 'float', 'value':0, 'suffix':' um', 'readonly': True, 'decimals':5},
                {'name': 'Jog Step', 'type': 'float', 'value': 10, 'suffix':' um', 'step': 1, 'limits':(0, 10000)},
                {'name': 'Jog Z', 'type': 'action3', 'ButtonText':('Jog Z +', 'Jog Z -', 'Home Z')},
                {'name': 'Target Location', 'type': 'float', 'value':0, 'suffix':' um', 'limits':(0, 25400), 'decimals':5},
                {'name': 'Move to Location', 'type':'action', 'ButtonText':('Move to Location')}
            ]},
            {'name': 'Spectrometer Camera', 'type': 'group', 'children': [
                {'name': 'Background Subtraction', 'type':'toggle', 'ButtonText':('Turn on background subtraction', 'Turn off background subtraction')},
                {'name': 'Exposure', 'type':'float', 'value':0.1, 'suffix':' s', 'step':0.01, 'limits':(0.001, 60)},
                {'name': 'Ref. Exposure', 'type':'float', 'value':0.1, 'suffix':' s', 'step':0.01, 'limits':(0.001, 10)},
                #{'name': 'AutoExposure', 'type':'toggle', 'ButtonText':('Auto exposure', 'Fixed exposure')}, #False=Fixed exposure
                #{'name': 'Camera Temp.', 'type': 'float', 'value':0, 'suffix':' C', 'readonly': True}
            ]},
            {'name': 'Monitor Camera', 'type': 'group', 'children': [
                {'name': 'Exposure Time', 'type': 'float', 'value': 20, 'suffix':' ms', 'limits':(0.001, 10000)},
                {'name': 'Frame Rate', 'type': 'int', 'value': 5, 'suffix':' Hz', 'limits':(1, 20)}
            ]},
            {'name': 'Microwave Source', 'type': 'group', 'children': [
                {'name': 'Cal. Freq (min.)', 'type': 'float', 'value': 5.58, 'suffix':' GHz', 'step': 0.01, 'limits': (0.05, 13.0), 'decimals':3},
                {'name': 'Cal. Freq (max.)', 'type': 'float', 'value': 5.82, 'suffix':' GHz', 'step': 0.01, 'limits': (0.05, 13.0), 'decimals':3},
                {'name': 'Cal. Freq (step)', 'type': 'float', 'value': 0.08, 'suffix':' GHz', 'step': 0.001, 'limits': (0.001, 13.0), 'decimals':3},
                {'name': 'RF Power', 'type': 'float', 'value': RFpower, 'suffix':' dBm', 'step': 0.1, 'limits': (-20, 10), 'decimals':2},
                {'name': 'RF Frequency', 'type': 'float', 'value': 5.75, 'suffix':' GHz', 'step': 0.01, 'limits': (0.05, 13.0), 'decimals':3}
            ]},
            {'name': 'More Settings', 'type': 'group', 'children': [
                {'name': 'Ambient Temp.', 'type': 'float', 'value': 0.0, 'suffix':' deg. C', 'readonly':True, 'decimals':4},
                {'name': 'FSR', 'type': 'float', 'value': FSR, 'suffix':' GHz', 'limits':(5, 100), 'decimals':5},
                {'name': 'SD', 'type': 'float', 'value': SD, 'suffix':' GHz/px', 'limits':(0, 2), 'decimals':4},
                {'name': 'Spectrum Col.', 'type':'int', 'value': spectColumn, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
                {'name': 'Spectrum Row', 'type':'int', 'value': spectRow, 'suffix':' px', 'step':1, 'limits':(0, 2048)},
                {'name': 'Laser Focus X', 'type': 'int', 'value': laserX, 'suffix':' px', 'limits':(-10000,10000),'decimals':4},
                {'name': 'Laser Focus Y', 'type': 'int', 'value': laserY, 'suffix':' px', 'limits':(-10000,10000),'decimals':4}
            ]}
        ]
        ## Create tree of Parameter objects
        self.allParameters = Parameter.create(name='params', type='group', children=self.params)
        self.parameterTreeWidget.setParameters(self.allParameters, showTop=False)

        self.session = None
        self.dataFile = None
        self.dataFileName = None
        # self.activeExperiment = None

        #Lock used to halt other threads upon app closing
        self.stop_event = threading.Event()
        self.synth_lock = threading.Lock()
        self.andor_lock = threading.Lock()
        self.mako_lock = threading.Lock()
        self.TempSensor_lock = threading.Lock()
        self.ZaberLock = threading.Lock()
        self.scan_lock = threading.Lock()

        # Even though ZaberDevice is a QThread, for now we don't need to run it in a separate thread
        # It is implemented this way for future-proofing
        self.ZaberDevice = ZaberDevice(self.stop_event, self)
        self.ZaberDevice.start()

        self.TempSensorDeviceThread = TempSensorDevice(self.stop_event, self)
        self.TempSensorProcessThread = TempSensorFreerun(self.TempSensorDeviceThread, self.stop_event)
        self.TempSensorProcessThread.updateTempSeqSig.connect(self.UpdateAmbientTemp)
        self.TempSensorDeviceThread.start()
        self.TempSensorProcessThread.start()

        self.SynthDevice = SynthDevice(self.stop_event, self)
        self.SynthDevice.start()

        self.ShutterDevice = ShutterDevice(self)    # Initialized to SAMPLE_STATE

        self.MakoDeviceThread = MakoDevice(self.stop_event, self)
        self.MakoProcessThread = MakoFreerun(self.MakoDeviceThread, self.stop_event)
        self.MakoProcessThread.updateCMOSImageSig.connect(self.MakoProcessUpdate)
        self.MakoDeviceThread.start()
        self.MakoProcessThread.start()

        self.AndorDeviceThread = AndorDevice(self.stop_event, self)
        self.AndorProcessThread = AndorProcessFreerun(self.AndorDeviceThread, self.stop_event)
        self.AndorProcessThread.updateSampleImageSig.connect(self.AndorSampleProcessUpdate)
        self.AndorProcessThread.updateBinnedImageSig.connect(self.AndorBinnedProcessUpdate)

        # self.CMOSview = CustomViewBox(invertY=True)             # The ViewBox is a zoomable/pannable box
        self.CMOSview = pg.PlotItem()
        self.graphicsViewCMOS.setCentralItem(self.CMOSview)     # GraphicsView is the main graphics container
        self.CMOSview.setAspectLocked(True)
        self.CMOSImage = pg.ImageItem(np.zeros((self.MakoDeviceThread.imageWidth//self.MakoDeviceThread.bin_size, self.MakoDeviceThread.imageHeight//self.MakoDeviceThread.bin_size)))      # ImageItem contains what needs to be plotted
        self.CMOSview.addItem(self.CMOSImage)
        self.CMOSview.autoRange(padding=0)                      # gets rid of padding
        self.CMOSvLine = pg.InfiniteLine(angle=90, movable=False)
        self.CMOShLine = pg.InfiniteLine(angle=0, movable=False)
        self.CMOSview.addItem(self.CMOSvLine, ignoreBounds=True)
        self.CMOSview.addItem(self.CMOShLine, ignoreBounds=True)
        self.CMOSvLine.setPos(laserX)
        self.CMOShLine.setPos(laserY)
        self.makoPoints = np.array([])    # To display previous scan points

        # Import standard Brillouin colormap data
        arr = []
        with open('Utilities\\colormap.txt', 'r') as f:
            reader = csv.reader(f, delimiter='\t')
            for row in reader:
                arr.append([float(x) for x in row] + [1]) # RGBA, 0.0 to 1.0
        self.heatmapColormapArray = np.array(arr)
        self.colormap = pg.ColorMap(np.linspace(0, 1, len(self.heatmapColormapArray)), self.heatmapColormapArray)

        # Data acquisition Heatmap
        self.heatmapScanData = np.array([])
        self.blankHeatmap = np.zeros((self.maxRowPoints, self.maxColPoints))
        self.heatmapPlot = pg.PlotItem()
        self.heatmapPlot.setAspectLocked(False)
        self.graphicsViewHeatmap.setCentralItem(self.heatmapPlot)     # GraphicsView is the main graphics container
        self.heatmapImage = pg.ImageItem(self.blankHeatmap)
        self.heatmapImage.setLookupTable(self.colormap.getLookupTable())
        self.heatmapPlot.addItem(self.heatmapImage)
        self.heatmapPlot.setXRange(0, self.maxRowPoints)
        self.heatmapPlot.setYRange(0, self.maxColPoints)
        self.heatmapPlot.autoRange(padding=0)

        # Andor Live image - Sample
        self.SampleView = CustomViewBox(invertY=True)
        self.graphicsViewSample.setCentralItem(self.SampleView)
        self.SampleView.setAspectLocked(True)
        self.SampleImage = pg.ImageItem(np.zeros((860, 200)))
        self.SampleView.addItem(self.SampleImage)
        self.SampleView.autoRange(padding=0)

         # Andor Live image - Binned
        self.BinnedView = CustomViewBox(invertY=True)
        self.graphicsViewBinned.setCentralItem(self.BinnedView)
        self.BinnedView.setAspectLocked(True)
        self.BinnedImage = pg.ImageItem(np.zeros((440, 100)))
        self.BinnedView.addItem(self.BinnedImage)
        self.BinnedView.autoRange(padding=0)

        # sampleSpectrumPlot is the plot of the raw data and Lorentzian fit
        self.sampleSpectrumPlot = pg.PlotItem()
        # self.sampleSpectrumPlot.setYRange(0, 17500)
        self.sampleSpectrumPlot.enableAutoRange(axis=self.sampleSpectrumPlot.vb.YAxis, enable=True)
        self.graphicsViewSampleSpectrum.setCentralItem(self.sampleSpectrumPlot)
        self.sampleSpectrumItem = pg.PlotDataItem(symbol='o')
        self.sampleSpectrumItem.setSymbolPen(color='g')
        self.sampleSpectrumItem.setPen(None)
        self.sampleSpectrumItem.setData(100*np.ones(100))
        self.sampleSpectrumItem2 = pg.PlotDataItem()
        self.sampleSpectrumItem2.setPen(width=2.5, color='r')
        self.sampleSpectrumItem2.setData(200*np.ones(100))
        self.sampleSpectrumPlot.addItem(self.sampleSpectrumItem)
        self.sampleSpectrumPlot.addItem(self.sampleSpectrumItem2)
        axBottom = self.sampleSpectrumPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleSpectrumPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        # sampleScanDepthPlot is the Brillouin vs z axis plot
        self.sampleScanDepthPlot = pg.PlotItem()
        self.sampleScanDepthPlot.setYRange(4.95,5.95)
        self.graphicsViewSampleScanDepth.setCentralItem(self.sampleScanDepthPlot)
        self.sampleScanDepthPlot.enableAutoRange(axis=self.sampleScanDepthPlot.vb.XAxis, enable=True)
        self.sampleScanDepthItem = pg.PlotDataItem() 
        self.sampleScanDepthItem.setPen(width=2.5, color='g')
        self.sampleScanDepthPlot.addItem(self.sampleScanDepthItem)
        axBottom = self.sampleScanDepthPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleScanDepthPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')
        self.sampleScanDepthData = np.array([])

        # This is the spectrograph:
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0
        self.sampleSpecSeriesPlot = pg.PlotItem()
        self.graphicsViewSampleSpecSeries.setCentralItem(self.sampleSpecSeriesPlot)
        self.sampleSpecSeriesImage = pg.ImageItem(self.sampleSpecSeriesData)
        self.sampleSpecSeriesPlot.addItem(self.sampleSpecSeriesImage)
        axBottom = self.sampleSpecSeriesPlot.getAxis('bottom')
        axBottom.setPen(width=2, color='w')
        axLeft = self.sampleSpecSeriesPlot.getAxis('left')
        axLeft.setPen(width=2, color='w')

        self.mainUI()

        self.AndorDeviceThread.start()
        self.AndorDeviceThread.setPriority(QtCore.QThread.TimeCriticalPriority)
        self.AndorProcessThread.start()
        time.sleep(0.1)   # CRUCIAL!! DON'T REMOVE. Need to let the threads fully start before continuing

        # create the figures for plotting Brillouin shifts and fits
        self.AndorProcessThread.updateSampleBrillouinSeqSig.connect(self.UpdateSampleBrillouinSeqPlot)
        self.AndorProcessThread.updateSampleSpectrum.connect(self.UpdateSampleSpectrum)

        self.BrillouinScan = ScanManager(self.stop_event, self.ZaberDevice, self.ShutterDevice, self.SynthDevice)
        self.BrillouinScan.addToSequentialList(self.AndorDeviceThread, self.AndorProcessThread)
        self.BrillouinScan.addToSequentialList(self.MakoDeviceThread, self.MakoProcessThread)
        self.BrillouinScan.addToSequentialList(self.TempSensorDeviceThread, self.TempSensorProcessThread)
        self.BrillouinScan.finished.connect(self.onFinishScan)
        self.BrillouinScan.clearGUISig.connect(self.clearGUIElements)

        # Apply settings from config file
        self.AndorDeviceThread.setTopPx(int(spectColumn))
        self.AndorDeviceThread.setLeftPx(int(spectRow))
        self.SynthDevice.setPower(RFpower)

        self.InitHardwareParameterTree()

    #############################################################################################
    # This next group of methods are used to set/get hardware settings, using the parameterTree #
    #############################################################################################
    def InitHardwareParameterTree(self):
        # print("[InitHardwareParameterTree]")

        # ========================= Spectrometer Camera =============================
        pItem = self.allParameters.child('Spectrometer Camera')
        pItem.child('Background Subtraction').sigActivated.connect(self.bgSubtraction)
        #pItem.child('AutoExposure').sigActivated.connect(self.switchAutoExp)
        pItem.child('Exposure').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setExposure))
        pItem.child('Ref. Exposure').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.AndorDeviceThread.setExposure))
        pItem.child('Exposure').setValue(self.AndorDeviceThread.getExposure())
        #pItem.child('Camera Temp.').setValue(self.AndorDeviceThread.getTemperature())

        # ========================= Monitor Camera ================================
        pItem = self.allParameters.child('Monitor Camera')
        pItem.child('Exposure Time').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.MakoDeviceThread.setExpTime))
        pItem.child('Frame Rate').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.MakoDeviceThread.setFrameRate))

        # ========================= Microwave Source ================================
        pItem = self.allParameters.child('Microwave Source')
        pItem.child('RF Frequency').sigValueChanged.connect(
            lambda data: self.changeHardwareSetting(data, self.SynthDevice.setFreq))
        pItem.child('RF Frequency').setValue(self.SynthDevice.getFreq())
        pItem.child('RF Power').sigValueChanged.connect(self.synthPowerValueChange)
        pItem.child('RF Power').setValue(self.SynthDevice.getPower())

        # ========================= Motor =================================
        pItem = self.allParameters.child('Motor')
        self.MotorPositionUpdate()

        # connect Jog buttons
        motorFwdFun = lambda: self.ZaberDevice.moveRelative( 
            self.allParameters.child('Motor').child('Jog Step').value() )
        motorBackFun = lambda: self.ZaberDevice.moveRelative( 
            -self.allParameters.child('Motor').child('Jog Step').value() )
        pItem.child('Jog Z').sigActivated.connect(motorFwdFun)
        pItem.child('Jog Z').sigActivated2.connect(motorBackFun)
        pItem.child('Jog Z').sigActivated3.connect(lambda: self.ZaberDevice.setMotorAsync('moveHome'))

        # connect Move button
        motorMoveFun = lambda: self.ZaberDevice.setMotorAsync(
            'moveAbs', [self.allParameters.child('Motor').child('Target Location').value()])
        pItem.child('Move to Location').sigActivated.connect(motorMoveFun)

        # ========================= Scan ===================
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').value()
        stepSize = pItem.child('Step Size').value()
        frameNum = pItem.child('Frame Number').value()
        endPos = startPos + stepSize * frameNum
        pItem.child('End Position').setValue(endPos)
        pItem.child('Scan/Cancel').sigActivated.connect(self.startScan)
        pItem.child('Scan/Cancel').sigActivated2.connect(self.cancelScan)
        pItem.child('Start Position').sigValueChanged.connect(lambda param, value: self.updateScanEndPos(0, param, value))
        pItem.child('Step Size').sigValueChanged.connect(lambda param, value: self.updateScanEndPos(1, param, value))
        pItem.child('Frame Number').sigValueChanged.connect(lambda param, value: self.updateScanEndPos(2, param, value))
        pItem.child('ToggleReference').sigActivated.connect(self.toggleReference)

        # ========================= More Settings ==================================
        pItem = self.allParameters.child('More Settings')
        pItem.child('FSR').sigValueChanged.connect(self.FSRValueChange)
        pItem.child('SD').sigValueChanged.connect(self.SDValueChange)
        # Spectrum Column / Row adjustment
        pItem.child('Spectrum Col.').sigValueChanged.connect(self.spectColumnValueChange)
        pItem.child('Spectrum Row').sigValueChanged.connect(self.spectRowValueChange)
        # Laser crosshair adjustment
        pItem.child('Laser Focus X').sigValueChanged.connect(self.CMOSvLineValueChange)
        pItem.child('Laser Focus Y').sigValueChanged.connect(self.CMOShLineValueChange)

        # ========================= Hardware monitor timers ===================

        #self.hardwareGetTimer = QTimer()
        #self.hardwareGetTimer.timeout.connect(self.HardwareParamUpdate)
        #self.hardwareGetTimer.start(10000)

        # Separate timer for motor position, update at faster rate
        self.motorPositionTimer = QTimer()
        self.motorPositionTimer.timeout.connect(self.MotorPositionUpdate)
        self.motorPositionTimer.start(500)

    def changeHardwareSetting(self, data, funcHandle):
        # print("[changeHardwareSetting]")
        funcHandle(data.value())

    #def HardwareParamUpdate(self):
    #    # print("[HardwareParamUpdate]")
    #    temp = self.AndorDeviceThread.getTemperature()
    #    self.allParameters.child('Spectrometer Camera').child('Camera Temp.').setValue(temp)
    #    # if (self.ShutterDevice.state == ShutterDevice.SAMPLE_STATE):
    #        # expTime = self.AndorDeviceThread.getExposure()
    #        # self.allParameters.child('Spectrometer Camera').child('Exposure').setValue(expTime)

    def updateScanEndPos(self, updateIndex, param, value):
        # print("[updateScanEndPos]")
        pItem = self.allParameters.child('Scan')
        startPos = pItem.child('Start Position').value()
        stepSize = pItem.child('Step Size').value()
        frameNum = pItem.child('Frame Number').value()
        if (updateIndex == 0):
            startPos = value
        elif (updateIndex == 1):
            stepSize = value
        else:
            frameNum = value
        endPos = startPos + stepSize * (frameNum - 1)
        pItem.child('End Position').setValue(endPos)

    @QtCore.pyqtSlot(float)
    def MotorPositionUpdate2(self, pos):
        #print("[MotorPositionUpdate2]")
        self.allParameters.child('Motor').child('Current Location').setValue(pos)

    def MotorPositionUpdate(self):
        #print("[MotorPositionUpdate]")
        pos = self.ZaberDevice.updatePosition()
        self.allParameters.child('Motor').child('Current Location').setValue(pos)

    def CMOShLineValueChange(self, param, value):
        # print("[CMOShLineValueChange]")
        self.CMOShLine.setPos(value)
        self.configParser.set('More Settings', 'laser_position_y', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def CMOSvLineValueChange(self, param, value):
        # print("[CMOSvLineValueChange]")
        self.CMOSvLine.setPos(value)
        self.configParser.set('More Settings', 'laser_position_x', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def synthPowerValueChange(self, param, value):
        self.SynthDevice.setPower(value)
        self.configParser.set('Synth', 'RF_power', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def FSRValueChange(self, param, value):
        self.configParser.set('More Settings', 'FSR', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def SDValueChange(self, param, value):
        self.configParser.set('More Settings', 'SD', str(value))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def spectColumnValueChange(self, param, value):
        self.AndorDeviceThread.setTopPx(int(value))
        self.configParser.set('Andor', 'spect_column', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    def spectRowValueChange(self, param, value):
        self.AndorDeviceThread.setLeftPx(int(value))
        self.configParser.set('Andor', 'spect_row', str(int(value)))
        with open(self.configFilename, 'w') as f:
            self.configParser.write(f)

    @QtCore.pyqtSlot()
    def clearGUIElements(self):
        # print("[clearGUIElements]")
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0
        self.sampleScanDepthData = np.array([])
        self.heatmapScanData = np.array([])

    def startScan(self):
        # First check that a session is running, and that an experiment is selected
        if self.session is None:
            choice = QtGui.QMessageBox.warning(self, 'Starting Scan...',
                                                'No Session open!',
                                                QtGui.QMessageBox.Ok)
            return

        print('Starting a scan in Exp_%d: ' % self.model.activeExperiment)

        # Check if 'Screenshots' directory exists, and if not, create one
        dataPath = os.path.dirname(self.dataFileName) + '\\Screenshots\\'
        if not os.path.exists(dataPath):
            os.makedirs(dataPath)

        calFreq = np.arange(self.allParameters.child('Microwave Source').child('Cal. Freq (min.)').value(), \
            self.allParameters.child('Microwave Source').child('Cal. Freq (max.)').value() + \
            self.allParameters.child('Microwave Source').child('Cal. Freq (step)').value(), \
            self.allParameters.child('Microwave Source').child('Cal. Freq (step)').value())

        flattenedParamList = generateParameterList(self.params, self.allParameters)

        scanSettings = {
            'start': self.allParameters.child('Scan').child('Start Position').value(),
            'step': self.allParameters.child('Scan').child('Step Size').value(),
            'frames': self.allParameters.child('Scan').child('Frame Number').value(),
            'calFreq': calFreq,
            'laserX': self.allParameters.child('More Settings').child('Laser Focus X').value(),
            'laserY': self.allParameters.child('More Settings').child('Laser Focus Y').value(),
            'refExp': self.allParameters.child('Spectrometer Camera').child('Ref. Exposure').value(),
            'sampleExp': self.allParameters.child('Spectrometer Camera').child('Exposure').value(),
            'flattenedParamList': flattenedParamList }
        self.BrillouinScan.assignScanSettings(scanSettings)
        # Scale plot window to scan length (+ calFreq calibration frames per y-z coordinate)
        self.calPoints = calFreq.shape[0]
        self.maxScanPoints = scanSettings['frames'] + self.calPoints
        self.sampleSpecSeriesSize = self.maxScanPoints # Prevent indexing error before scan starts
        self.maxRowPoints = scanSettings['frames'] + self.calPoints
        self.maxColPoints = 1
        self.heatmapPlot.setXRange(0, self.maxRowPoints)
        self.heatmapPlot.setYRange(0, self.maxColPoints)

        self.BrillouinScan.sessionData = self.session
        self.BrillouinScan.saveExpIndex = self.model.activeExperiment

        # Stop periodic polling of hardware state
        #self.hardwareGetTimer.stop()
        self.motorPositionTimer.stop()
        self.BrillouinScan.motorPosUpdateSig.connect(self.MotorPositionUpdate2)

        # Finally start scan in a new thread
        self.BrillouinScan.start()

    def cancelScan(self):
        print('Scan cancelled!')
        self.BrillouinScan.Cancel_Flag = True

    def onFinishScan(self):
        self.maxScanPoints = 400 # Re-scale plot window for free-running mode
        self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, 210))
        self.sampleSpecSeriesSize = 0
        self.maxRowPoints = 20
        self.maxColPoints = 20
        self.calPoints = 0
        self.heatmapPlot.setXRange(0, self.maxRowPoints)
        self.heatmapPlot.setYRange(0, self.maxColPoints)
        # Return microwave source to previous (non-calibration) set point
        self.SynthDevice.setFreq(self.allParameters.child('Microwave Source').child('RF Frequency').value())
        self.SynthDevice.setPower(self.allParameters.child('Microwave Source').child('RF Power').value())
        time.sleep(0.16)
        if (self.allParameters.child('Scan').child('ToggleReference').value() == True):
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
            self.AndorDeviceThread.setRefState(True)
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
            self.AndorDeviceThread.setRefState(False)
        if self.BrillouinScan.Cancel_Flag == False:
            currExp = self.session.experimentList[self.BrillouinScan.saveExpIndex]
            currScanIdx = self.session.experimentList[self.BrillouinScan.saveExpIndex].size() - 1
            SD = self.BrillouinScan.SDcal
            FSR = self.BrillouinScan.FSRcal
            if ~np.isnan(SD):
                self.allParameters.child('More Settings').child('SD').setValue(SD)
            if ~np.isnan(FSR):
                self.allParameters.child('More Settings').child('FSR').setValue(FSR)
        self.BrillouinScan.motorPosUpdateSig.disconnect(self.MotorPositionUpdate2)
        #self.hardwareGetTimer.start(60000)
        self.motorPositionTimer.start(500)
        self.BrillouinScan.Cancel_Flag = False
        print('Scan completed')

    def toggleReference(self, sliderParam, state):
        # print("[toggleReference]")
        # state == True --> Reference
        # state == False --> Sample
        if state:
            self.ShutterDevice.setShutterState(self.ShutterDevice.REFERENCE_STATE)
            self.AndorDeviceThread.setRefState(True)
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Ref. Exposure').value())
        else:
            self.ShutterDevice.setShutterState(self.ShutterDevice.SAMPLE_STATE)
            self.AndorDeviceThread.setRefState(False)
            self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())

    def bgSubtraction(self, sliderParam, state):
        if state:
            self.ShutterDevice.closeBGshutter()
            self.AndorDeviceThread.startBGsubtraction()
            #print("Spectrometer background subtraction ON")
            # Wait until BG subtraction is complete before re-opening shutter
            while self.AndorDeviceThread.checkBGsubtraction():
                time.sleep(0.1)
            self.ShutterDevice.openBGshutter()
        else:
            self.AndorDeviceThread.stopBGsubtraction()
            #print("Spectrometer background subtraction OFF")

    #def switchAutoExp(self, sliderParam, state):
    #    if state:
    #        self.AndorDeviceThread.setAutoExp(True)
    #        print("Spectrometer auto exposure ON")
    #    else:
    #        self.AndorDeviceThread.setAutoExp(False)
    #        self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())
    #        print("Spectrometer auto exposure OFF")
    #        # self.AndorDeviceThread.setExposure(self.allParameters.child('Spectrometer Camera').child('Exposure').value())

    #############################################################################################
    # This next group of methods callback methods to display acquired data #
    #############################################################################################
    
    def UpdateAmbientTemp(self, temperature):
        # print('UpdateAmbientTemp')
        self.allParameters.child('More Settings').child('Ambient Temp.').setValue(temperature)

    # updates the figure containing the Brillouin sequence. newData is a list
    def UpdateSampleBrillouinSeqPlot(self, interPeakDist):
        # print("[UpdateBrillouinSeqPlot]")
        SD = self.allParameters.child('More Settings').child('SD').value()
        FSR = self.allParameters.child('More Settings').child('FSR').value()
        if ~np.isnan(interPeakDist):
            newData = [0.5*(FSR - SD*interPeakDist)]
            newData2 = newData
        else:
            newData = [np.nan]
            newData2 = [np.nan]

        # Update heatmap with Brillouin data
        if len(newData)+len(self.heatmapScanData) > self.maxRowPoints*self.maxColPoints:
            self.heatmapScanData = np.array([])
        self.heatmapScanData = np.append(self.heatmapScanData, newData)
        heatmapSeq = np.pad(self.heatmapScanData, (0, self.maxRowPoints*self.maxColPoints - self.heatmapScanData.shape[0]), 'constant')
        heatmapSeq[np.isnan(heatmapSeq)] = 0 #Remove NaNs
        heatmapArr = np.reshape(heatmapSeq, (-1, self.maxRowPoints))   # self.maxRowPoints columns
        heatmapArr = np.rot90(heatmapArr, 1, (1,0)) # Rotate to match XY axes
        colormapLow = self.allParameters.child('Display').child('Colormap (min.)').value()
        colormapHigh = self.allParameters.child('Display').child('Colormap (max.)').value()
        self.heatmapImage.setImage(heatmapArr, levels=(colormapLow, colormapHigh))

        # Update Brillouin vs. time plot
        if len(newData)+len(self.sampleScanDepthData) > self.maxScanPoints:
            if self.calPoints > 0:
                self.sampleScanDepthData = np.array(newData)
            else:
                t = self.maxScanPoints - len(newData) - len(self.sampleScanDepthData)
                self.sampleScanDepthData = np.roll(self.sampleScanDepthData, t)
                self.sampleScanDepthData[self.maxScanPoints - len(newData):] = newData
        else:
            self.sampleScanDepthData = np.append(self.sampleScanDepthData, newData)

        xdata = np.arange(len(self.sampleScanDepthData))
        xdata = xdata[~np.isnan(self.sampleScanDepthData)]
        ydata = self.sampleScanDepthData[~np.isnan(self.sampleScanDepthData)]
        self.sampleScanDepthItem.setData(x=xdata, y=ydata)

    # Plot fitted Brillouin spectrum
    # curvedata is a tuple of (raw spectrum, fitted spectrum)
    def UpdateSampleSpectrum(self,curveData):
        # print("[UpdateSpectrum]")
        rawSpect = curveData[0]
        fitSpect = curveData[1]

        xdata = np.arange(len(rawSpect))

        self.sampleSpectrumItem.setData(x=xdata, y=np.array(rawSpect))
        if len(fitSpect[~np.isnan(fitSpect)]) == len(rawSpect):
            self.sampleSpectrumItem2.setData(x=np.copy(xdata), y=np.array(fitSpect))
        else:
            self.sampleSpectrumItem2.setData(x=np.copy(xdata), y=np.zeros(len(rawSpect)))

        if self.sampleSpecSeriesData.shape[1]!=len(rawSpect):    #if size of spectrum change, reset the data
            print('[UpdateSpectrum] Resizing spectrograph')
            self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
            self.sampleSpecSeriesSize = 0
        #print('self.sampleSpecSeriesSize=', self.sampleSpecSeriesSize)
        #print('self.maxScanPoints=', self.maxScanPoints)
        if self.sampleSpecSeriesSize < self.maxScanPoints:
            self.sampleSpecSeriesData[self.sampleSpecSeriesSize,:] = rawSpect
            self.sampleSpecSeriesSize += 1
        else:
            if self.calPoints > 0:
                self.sampleSpecSeriesData = np.zeros((self.maxScanPoints, len(rawSpect)))
                self.sampleSpecSeriesData[0,:] = rawSpect
                self.sampleSpecSeriesSize = 1
            else:
                self.sampleSpecSeriesData = np.roll(self.sampleSpecSeriesData, -1, axis=0)
                self.sampleSpecSeriesData[-1, :] = rawSpect

        maximum = self.sampleSpecSeriesData.max()
        sampleSpecSeriesDataScaled = self.sampleSpecSeriesData #* (255.0 / maximum)
        self.sampleSpecSeriesImage.setImage(sampleSpecSeriesDataScaled)

    # Update raw Andor image
    def AndorSampleProcessUpdate(self, image):
        # print("[AndorProcessUpdate]")
        self.SampleImage.setImage(image.transpose((1,0)))

    # Update binned Andor image
    def AndorBinnedProcessUpdate(self, image):
        # print("[AndorProcessUpdate]")
        self.BinnedImage.setImage(image.transpose((1,0)))

    # Update the CMOS camera image
    def MakoProcessUpdate(self, makoData):
        #print("[MakoProcessUpdate]")
        image = makoData
        self.CMOSImage.setImage(image)

    def createNewSession(self):
        dialog = QtGui.QFileDialog()
        options = QtGui.QFileDialog.Options()
        options |= QtGui.QFileDialog.DontUseNativeDialog
        dialog.setOptions(options)
        dialog.setDefaultSuffix('hdf5')
        dialog.setAcceptMode(QtGui.QFileDialog.AcceptSave)
        dialog.setNameFilter("HDF5 Files(*.hdf5)")
        if dialog.exec_():
            filename = str(dialog.selectedFiles()[0])
        else:
            return

        self.dataFileName = filename
        self.sessionName.setText(filename)

        #create a single new session
        self.session = SessionData(os.path.basename(self.dataFileName), filename=filename)

        # create tree model (for display)
        self.model = BrillouinTreeModel()
        self.treeView.setModel(self.model)
        self.treeView.setContextMenuPolicy(QtCore.Qt.CustomContextMenu)
        self.treeView.customContextMenuRequested.connect(self.createTreeMenu)
        self.treeView.selectionModel().selectionChanged.connect(self.treeSelectionChanged)

        self.model.session = self.session

        self.session.updateTreeViewSig.connect(
            lambda updateIndices:self.model.updateTree(updateIndices, self.treeView))
        self.session.addNewExperiment()

        self.treeView.setEnabled(True)


    def closeEvent(self,event):
        print("Program Shutdown")
        if self.dataFile:
            self.dataFile.close()

        #self.hardwareGetTimer.stop()
        self.motorPositionTimer.stop()

        self.stop_event.set()
        while self.AndorDeviceThread.isRunning():
            time.sleep(0.1)
        while self.MakoDeviceThread.isRunning():
            time.sleep(0.1)

        self.MakoDeviceThread.shutdown()
        self.TempSensorDeviceThread.shutdown()
        self.ZaberDevice.shutdown()
        self.ShutterDevice.shutdown()
        self.SynthDevice.shutdown()

        event.accept() #closes the application

    #############################################################################################
    # This next group of methods are for experiment/data management (treeView)                  #
    #############################################################################################
    #TODO: save to file when renaming

    def createTreeMenu(self, position):
        menu = QtGui.QMenu()
        modelIndex = self.treeView.indexAt(position)
        item = self.model.itemFromIndex(modelIndex)

        if item is not None:
            lvl = 0
            k = item.parent()
            while k is not None:
                k = k.parent()
                lvl += 1

            if lvl == 0:
                setActiveAction = menu.addAction("Set as Active")
                action = menu.exec_(self.treeView.mapToGlobal(position))
                if action == setActiveAction:
                    self.model.setActiveExperiment(modelIndex.row())
            elif lvl == 1:
                if (item.parent().child(item.row(), 0).data(role=UserItemRoles.IsDeletedRole).toBool() == False):
                    deleteText = "Delete Scan"
                else:
                    deleteText = "Undelete Scan"
                deleteAction = menu.addAction(deleteText)
                action = menu.exec_(self.treeView.mapToGlobal(position))
                if action == deleteAction:
                    self.model.deleteScan(item)
        else:
            newAction = menu.addAction("New Experiment")
            action = menu.exec_(self.treeView.mapToGlobal(position))
            if action == newAction:
                self.session.addNewExperiment()
    def treeSelectionChanged(self, selected, deselected):
        #print('selected =', selected)
        #print('selected.indexes() =', selected.indexes())
        idx = selected.indexes()[0]
        item = self.model.itemFromIndex(idx)
        if item.parent() is None:
            # print("Experiment selected")
            return
    def mainUI(self):
        self.newSessionBtn.clicked.connect(self.createNewSession)

if __name__ == "__main__":
    app = QtGui.QApplication(sys.argv)
    GUI = App()
    GUI.show()
    sys.exit(app.exec_())