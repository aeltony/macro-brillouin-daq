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
		self.stop_event = stop_event

		self.motor = motor
		self.shutter = shutter
		self.synth = synth

		self.sessionData = None
		self.saveExpIndex = -1	# this is the expScanIndices argument in the Session.saveToFile() method
		self.scanSettings = None
		self.Cancel_Flag = False
		self.SDcal = np.nan
		self.FSRcal = np.nan

	# TODO: add a lock for accessing these variables
	def assignScanSettings(self, settings):
		self.scanSettings = settings

	# deviceThread is a BrillouinDevice object
	# Make sure only to use thread-safe parts of BrillouinDevice, like
	# setting/clearing events or flags
	def addToSequentialList(self, deviceThread, processingThread):
		self.sequentialAcqList.append(deviceThread)
		self.sequentialProcessingList.append(processingThread)

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
		if self.sessionData is None:
			print("No Session provided to save data in; set ScanManager.sessionData first")
			return
		if self.saveExpIndex == -1:
			print("Save parameter is empty; set ScanManager.saveParameter first")
			return

		# Switch to sample arm
		self.shutter.setShutterState((1, 0))
		self.sequentialAcqList[0].forceSetExposure(self.scanSettings['sampleExp'])
		self.sequentialAcqList[0].setRefState(False)
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
				if step > 0:
					self.motor.moveAbs(initialPos)
				# Stop acquiring data
				for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
					devProcessor.enqueueData = False
					dev.runMode = 0
				# Wait for all processing threads to complete + empty the data queues (free up working memory)
				for devProcessor in self.sequentialProcessingList:
					while devProcessor.isIdle == False:
						time.sleep(0.1)
					while not devProcessor.processedData.empty():
						devProcessor.processedData.get()
				# Send signal to clear GUI plots
				self.clearGUISig.emit()
				self.maxScanPoints = 400 # Re-scale plot window for free-running mode
				# Send motor position signal to update GUI
				motorPos = self.motor.updatePosition()
				self.motorPosUpdateSig.emit(motorPos)
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
				if step > 0:
					self.motor.moveRelative(step)
			# Otherwise, take calibration data
			else:
				self.shutter.setShutterState((0, 1)) # switch to reference arm
				self.sequentialAcqList[0].forceSetExposure(self.scanSettings['refExp'])
				self.sequentialAcqList[0].setRefState(True)
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
				self.sequentialAcqList[0].setRefState(False)
		# Return to start location
		if step > 0:
			self.motor.moveAbs(initialPos)
		# Send motor position signal to update GUI
		motorPos = self.motor.updatePosition()
		self.motorPosUpdateSig.emit(motorPos)

		# Wait for all processing threads to complete
		for devProcessor in self.sequentialProcessingList:
			while devProcessor.isIdle == False:
				time.sleep(0.1)

		# Process Data
		calFrames = calFreq.shape[0]
		dataset = {'Andor': [], 'Mako': [], 'TempSensor': []}
		for (dev, devProcessor) in zip(self.sequentialAcqList, self.sequentialProcessingList):
			while devProcessor.processedData.qsize() > frames + calFrames:
				devProcessor.processedData.get() # pop out the first few sets of data stored before scan started
			while not devProcessor.processedData.empty():
				data = devProcessor.processedData.get()	# data[0] is a counter
				dataset[dev.deviceName].append(data[1])

		# Make data arrays
		# lineScan.generateTestData(k)
		lineScan = ScanData(timestamp=datetime.now().strftime('%H:%M:%S'))
		lineScan.CalFreq = calFreqRead
		lineScan.TempList = np.array(dataset['TempSensor'])
		lineScan.CMOSImage = np.array(dataset['Mako'])
		lineScan.AndorImage = np.array([d[0] for d in dataset['Andor']])
		lineScan.SpecList = np.array([d[1] for d in dataset['Andor']])
		calPeakDist = np.array([d[2] for d in dataset['Andor']])[-calFrames:]
		# Free up memory used by dataset
		del dataset
		lineScan.MotorCoords = motorCoords
		lineScan.Screenshot = self.scanSettings['screenshot']
		lineScan.flattenedParamList = self.scanSettings['flattenedParamList']	#save all GUI paramaters

		# Find SD / FSR of final calibration curve
		try:
			self.SDcal, self.FSRcal = DataFitting.fitCalCurve(np.copy(calPeakDist), np.copy(calFreqRead), 1e-6, 1e-6)
			print('[ScanManager] Fitted SD = %.3f GHz/px' % self.SDcal)
			print('[ScanManager] Fitted FSR = %.2f GHz' % self.FSRcal)
		except:
			self.SDcal = np.nan
			self.FSRcal = np.nan
		# Saved fitted SD/FSR
		lineScan.SD = self.SDcal
		lineScan.FSR = self.FSRcal

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