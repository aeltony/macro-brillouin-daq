import threading
from PyQt5 import QtGui,QtCore
from PyQt5.QtCore import pyqtSignal
import queue as Queue
from timeit import default_timer as timer   #debugging
import time
import numpy as np
from ExperimentData import *
import DataFitting

# Scans are sequential measurements comprised of one or many different pieces 
# of hardware. A subset of the data acqusition are taken sequentially while 
# others can be free running in their own threads. The ScanManager controls the 
# sequential data acqusitions and synchronizes with the free running ones using
# ??? (time tags?). Processing of data from individual instruments are done in 
# their corresponding threads asynchronously from both the scan manager and the 
# data acqusition threads. 
# The scan manager also synchronizes the processed data
# from the different processing threads (and does a final processing combining
# different pieces of data?) and sends a signal to the GUI thread for live update.

class ScanManager(QtCore.QThread):
	motorPosUpdateSig = pyqtSignal(float)
	clearGUISig = pyqtSignal()

	#TODO: add a pause event
	def __init__(self, stop_event, motor, shutter, synth):
		super(ScanManager,self).__init__()
        # TODO: change to dictionary
		self.sequentialAcqList = []
		self.sequentialProcessingList = []
		self.freerunningList = []
		self.stop_event = stop_event

		self.motor = motor
		self.shutter = shutter
		self.synth = synth

		self.processor = None

		self.saveScan = True
		self.sessionData = None
		self.saveExpIndex = -1	# this is the expScanIndices argument in the Session.saveToFile() method

		self.scanSettings = None
		self.Cancel_Flag = False


	# TODO: add a lock for accessing these variables
	def assignScanSettings(self, settings):
		self.scanSettings = settings

	# deviceThread is a BrillouinDevice object
	# Make sure only to use thread-safe parts of BrillouinDevice, like
	# setting/clearing events or flags
	def addToSequentialList(self, deviceThread, processingThread):
		self.sequentialAcqList.append(deviceThread)
		self.sequentialProcessingList.append(processingThread)

	def addToFreerunningList(self, deviceThread):
		self.freerunningList.append(deviceThread)

	# processor is a method that takes a list of Brillouin shifts and other arguments as needed
	def addDataProcessor(self, processor):
		self.processor = processor

  	# In a sequential scan, the following sequence must be followed
  	#	dev.doSomethingStart()
  	#	dev.doSomethingWait()
  	#	dev.doSomethingContinue()
  	#
  	#   devThread.continueEvent.set()
    #   devThread.completeEvent.wait()
    #   devThread.completeEvent.clear()
	def run(self):
		self.setPriority(QtCore.QThread.TimeCriticalPriority)

		# make sure saving settings are ok
		if self.saveScan:
			if self.sessionData is None:
				print("No Session provided to save data in; set ScanManager.sessionData first")
				return
			if self.saveExpIndex == -1:
				print("Save parameter is empty; set ScanManager.saveParameter first")
				return

		# Switch to sample arm
		self.shutter.setShutterState((1, 0))
		# Capture initial position of motor
		initialPos = self.motor.updatePosition()
		# Initialize motor to (relative) start position
		self.motor.setMotorAsync('moveRelative', [self.scanSettings['start']])

		print("[ScanManager/run] Start")

		# first turn off free running mode
		for dev in self.sequentialAcqList:
			dev.sendPauseSignal()
		for dev in self.sequentialAcqList:
			dev.waitForPause()
			dev.runMode = 1

		# free running mode off now; safe to force hardware settings
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])

		# Pause all processors and clear any data 
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)
			devProcessor.enqueueData = True
			while not devProcessor.processedData.empty():
				devProcessor.processedData.get()
			
		# Send signal to clear GUI plots
		self.clearGUISig.emit()

		for dev in self.sequentialAcqList:
			dev.unpause()

		step = self.scanSettings['step']
		frames = self.scanSettings['frames']
		calFreq = self.scanSettings['calFreq']
		motorCoords = np.empty([frames]) # Keep track of actual motor positions
		calFreqRead = np.empty([calFreq.shape[0]]) # Keep track of actual microwave freq

		for i in range(frames):
			# Check if scan cancelled
			if self.Cancel_Flag == True:
				print('[ScanManager/run] Cancel_Flag! Terminating scan...')
				# Return to initial (pre-scan) position
				self.motor.moveAbs(initialPos)
				for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
					devProcessor.enqueueData = False
					dev.runMode = 0
				# Send signal to clear GUI plots
				self.clearGUISig.emit()
				self.maxScanPoints = 400 # Re-scale plot window for free-running mode
				# Send motor position signal to update GUI
				motorPos = self.motor.updatePosition()
				self.motorPosUpdateSig.emit(motorPos)
				self.Cancel_Flag = False
				return
			# Signal all devices to start new acquisition
			for dev in self.sequentialAcqList:
				dev.continueEvent.set()
			# Synchronization... wait for all the device threads to complete
			for dev in self.sequentialAcqList:
				dev.completeEvent.wait()
				dev.completeEvent.clear()
			# Send motor position signal to update GUI
			motorPos = self.motor.updatePosition()
			#print('motorPos =', motorPos)
			motorCoords[i] = motorPos
			self.motorPosUpdateSig.emit(motorPos)
			# Move one step forward if not end of line
			if i < frames-1:
				self.motor.moveRelative(step)
			# Otherwise return to intial (pre-scan) position + take calibration data
			else:
				self.motor.moveAbs(initialPos)
				self.shutter.setShutterState((0, 1)) # switch to reference arm
				self.sequentialAcqList[0].forceSetExposure(self.scanSettings['refExp'])
				for idx, f in enumerate(calFreq):
					self.synth.setFreq(f)
					time.sleep(0.01)
					calFreqRead[idx] = self.synth.getFreq()
					# Signal all devices to start new acquisition
					for dev in self.sequentialAcqList:
						dev.continueEvent.set()
					# synchronization... wait for all the device threads to complete
					for dev in self.sequentialAcqList:
						dev.completeEvent.wait()
						dev.completeEvent.clear()
				# return to sample arm
				self.shutter.setShutterState((1, 0))
				self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
		# Send motor position signal to update GUI
		motorPos = self.motor.updatePosition()
		self.motorPosUpdateSig.emit(motorPos)

		# Wait for all processing threads to complete
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)

		print('calFreqRead =', calFreqRead)
		# Process Data
		calFrames = calFreq.shape[0]
		#BS = np.random.random()*(self.colormapHigh - self.colormapLow) + self.colormapLow
		dataset = {'Andor': [], 'Mako': [], 'TempSensor': []}
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			while devProcessor.processedData.qsize() > frames + calFrames:
				devProcessor.processedData.get() # pop out the first few sets of data stored before scan started
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])

		# Create data arrays for sample and reference frames
		RawTempList = np.array(dataset['TempSensor'])[:-calFrames]
		CalTempList = np.array(dataset['TempSensor'])[-calFrames:]
		imageList = [d[0] for d in dataset['Mako']]
		CMOSImage = np.array(imageList)[:-calFrames]
		specImageList = [d[0] for d in dataset['Andor']]
		AndorImage = np.array(specImageList)[:-calFrames]
		CalImage = np.array(specImageList)[-calFrames:]
		maxRowList = [d[1] for d in dataset['Andor']]
		RawSpecList = np.array(maxRowList)[:-calFrames]
		CalSpecList = np.array(maxRowList)[-calFrames:]
		dispImageList = [d[2] for d in dataset['Andor']]
		AndorDisplay = np.array(dispImageList)
		laserPos = np.array([np.float(self.scanSettings['laserX']),np.float(self.scanSettings['laserY'])])

		# Save data
		# lineScan.generateTestData(k)
		lineScan = ScanData(timestamp=datetime.now().strftime('%H:%M:%S'))
		lineScan.CalFreq = calFreqRead
		lineScan.RawTempList = RawTempList
		lineScan.CalTempList = CalTempList
		lineScan.AndorImage = AndorImage
		lineScan.CalImage = CalImage
		lineScan.CMOSImage = CMOSImage
		lineScan.RawSpecList = RawSpecList
		lineScan.CalSpecList = CalSpecList
		lineScan.AndorDisplay = AndorDisplay
		lineScan.LaserPos = laserPos
		lineScan.MotorCoords = motorCoords
		lineScan.Screenshot = self.scanSettings['screenshot']
		lineScan.flattenedParamList = self.scanSettings['flattenedParamList']	#save all GUI paramaters

		#### Fitting Brillouin spectra
		startTime = timer()
		freqList = np.zeros(RawSpecList.shape[0])
		signal = np.zeros(RawSpecList.shape[0])
		fittedSpect = np.zeros(RawSpecList.shape)
		# Find SD / FSR
		pxDist = np.zeros(calFreq.shape)
		for j in range(calFrames):
			interPeakDist, fittedCalSpect = DataFitting.fitSpectrum(np.copy(CalSpecList[j]),1e-6,1e-6)
			if len(interPeakDist)>1:
				pxDist[j] = interPeakDist[1]
				print('pxDist =', pxDist[j])
			else:
				print("[ScanManager/run] Calibration frame #%d failed." %j)
				pxDist[j] = np.nan
		print('pxDist =', pxDist)
		print('calFreqRead =', calFreqRead)
		try:
			SDcal, FSRcal = DataFitting.fitCalCurve(np.copy(pxDist), np.copy(calFreqRead), 1e-6, 1e-6)
			print('Fitted SD =', SDcal)
			print('Fitted FSR =', FSRcal)
		except:
			SDcal = np.nan
			FSRcal = np.nan

		# Use SD/FSR to determine Brillouin shift values
		for k in range(frames):
			sline = np.copy(RawSpecList[k])
			sline = np.transpose(sline)
			interPeakDist, fittedSpect[k] = DataFitting.fitSpectrum(sline,1e-6,1e-6)
			if len(interPeakDist)==2:
				freqList[k] = 0.5*(FSRcal - SDcal*interPeakDist[1])
				signal[k] = interPeakDist[0]
			else:
				freqList[k] = np.nan
				signal[k] = np.nan
		# Saved fitted data
		lineScan.SD = SDcal
		lineScan.FSR = FSRcal
		lineScan.BSList = freqList
		lineScan.FitSpecList = fittedSpect

		endTime = timer()
		print("[ScanManager] Fitting time = %.3f s" % (endTime - startTime))
		print('Brillouin frequency shift list:', freqList)

		self.sessionData.experimentList[self.saveExpIndex].addScan(lineScan)
		scanIdx = self.sessionData.experimentList[self.saveExpIndex].size() - 1
		self.sessionData.saveToFile([(self.saveExpIndex,[scanIdx])])

		# finally return to free running settings before the scan started
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			devProcessor.enqueueData = False
			dev.runMode = 0

		# Send signal to clear GUI plots 
		self.clearGUISig.emit()

		print("[ScanManager/run] finished")