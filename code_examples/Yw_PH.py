#!/usr/bin/env python
from __future__ import print_function
import time, sys, os, datetime # serial, shutil,
from time import sleep
import numpy as np
from Flybratron_functions import PhidSN

############################################################ PARAMS TO SET ############################################################
# Phase coverage: [-0.125, -0.1, -0.075, -0.05, -0.025, 0.0, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15] + phase_use

phase_use			= 0.0			# center phase to use for the phase analysis
num_sets_use        = 1 			# number of sets of trials to run
amplitude_use    	= 1000 			# intended angular velocity to use for the phase analysis
stim_dur_use      	= 0.25			# duration of each stimulus in seconds
sleep_dur_use 		= 0.75			# time between stimuli in seconds

########################################################################################################################################
#### DEFAULT 
# fly line
driver 			= ' XHCS'
responder 		= ' XHCS '
rig 			= 'MS' # mechanical stimulation
slpT            = 0.0
test_mode       = False
trial_clip_V, CL_V, stat_V = 9.99, 9., -9.  

# whether to save bagfiles
saveBag 		= True
# record launch file and bag directory
recLaunchDir= '/home/fponce/catkin_ws/src/Kinefly/launch/record.launch'
bagDir  	= '/home/fponce/bagfiles/ivo/'
#### Prep Phidgets ####
import Phidget22.Devices.VoltageOutput
secondAOCH 	= 1
aout 			= Phidget22.Devices.VoltageOutput.VoltageOutput()
aout.setDeviceSerialNumber(PhidSN)
aout.setChannel(secondAOCH)
aout.openWaitForAttachment(5000)
aout.setEnabled(True)
aout.setVoltage(0.)

#### Flybratron  

print ('*** Sensory modality: Mechanical ***')
from flybratron import Flybratron

dev 			= Flybratron('/dev/ttyACM0')
def_phase   	= 0.
def_amplitude	= 0
waveform 		= 'sin2f' # yaw

dev.param 		= {
					'amplitude' : def_amplitude, 
					'phase'     : def_phase, 
					'waveform'  : waveform,
					}

# brief burst to confirm ratio of driving waveform to accelometer
if 1:
	dev.operating_mode = 'free'
	dev.amplitude =.5
	if rig == 'MS':
		aout.setVoltage(-8.)
	else:
		aout.setVoltage(secondAOCH, -8.)  
	sleep(0.25)
	dev.amplitude =0.
	if rig == 'MS':
		aout.setVoltage(0.)
	else:
		aout.setVoltage(secondAOCH, 0.)  
	if not test_mode:
		dev.operating_mode = 'sync'
	sleep(slpT)

################ trial saving specs ################
scriptNmXpPrt = os.path.basename(__file__)[3:5]

## Set variable names and parameters
year    	= datetime.datetime.today().strftime('%Y')[2:4]
month   	= datetime.datetime.today().strftime('%m')
day     	= datetime.datetime.today().strftime('%d')
datum   	= year + month + day
try:
  trialNum
except NameError:
  print ('assigned trialNum to 1')
  trialNum 	= 1

reCheck 	= True
while reCheck:
    outFileName= datum + driver + responder + str(trialNum).zfill(3) + ' ' + scriptNmXpPrt + rig + '.bag'
    # check if the destination file exists
    if os.path.isfile(bagDir+outFileName.replace(" ", "_")):
		trialNum += 1
		reCheck = True
    else:
        reCheck = False
outFileName 	= datum + driver + responder + str(trialNum).zfill(3)+ ' ' + scriptNmXpPrt + rig + '.bag'


#### Prep to record kinefly video and bag files locally ####
try:
	saveBag
except NameError:
	saveBag = False
	print ('Not saving Kinefly video and data locally')

if saveBag:
	## roslaunch prep (from http://wiki.ros.org/roslaunch/API%20Usage)
	import roslaunch, rospy ##vid
	#rospy.init_node('en_Mapping', anonymous=True) ##vid
	uuid = roslaunch.rlutil.get_or_generate_uuid(None, False) ##vid
	roslaunch.configure_logging(uuid) ##vid
	outFileNameNoSpc = outFileName.replace(" ", "_") ##vid
	pFix = outFileNameNoSpc[:-4] ##vid
	if pFix: ##vid
	    print ('automated naming based on date and script name') ##vid
	else: ##vid
	    pFix = datetime.datetime.today().strftime('%Y-%m-%d-%H-%M-%S') ##vid
	includepfix = 'prefix:=' + pFix ##vid
	sys.argv.append(includepfix) ##vid
    	parent = roslaunch.parent.ROSLaunchParent(uuid, [recLaunchDir]) ##vid
    	rospy.on_shutdown(parent.shutdown) ##vid

#### Start Experiment  
print (' ')
print ('Started trial ' + str(trialNum) + ' at: ' + str(datetime.datetime.now())[11:19])
expStartTime	= time.time()

#### Start recording kinefly video
if saveBag:
	parent.start() ##vid ipython command equivalent to "roslaunch Kinefly record.launch"
	print ('Saving bag file: ' + pFix + '.bag') ##vid
 
 

######################## RUN PHASE ANALYSIS ############################
module_clip_V = -9.8

from Flybratron_functions import yaw_phase_function as ypf
# yaw_phase_function(rig, dev, aout, stat_V, phase_=0.0, amplitude_=2., sleep_dur=1., stim_dur=0.2, num_sets=2, module_clip_V = -9.8)
ypf(rig, dev, aout, stat_V, phase_=phase_use, amplitude_=amplitude_use, sleep_dur=sleep_dur_use, stim_dur=stim_dur_use, num_sets=num_sets_use, module_clip_V=module_clip_V)
print ('Phase set completed')
print (' ')
######################## END PHASE ANALYSIS ############################




#### Turn off and close the Flybratron device
param = {
	'amplitude' : def_amplitude, 
	'phase'     : def_phase, 
	}
dev.param 	= param
sleep(slpT)
dev.close()
	
#### terminate phidgets
aout.setVoltage(0.)
aout.setEnabled(False)
aout.close()

#### Stop Bagfile
if saveBag:
	parent.shutdown() ##vid # ipython command equivalent to Ctrl+C 

elapsed 	= time.time() - expStartTime
print ('Trial time [s]:' + str(elapsed)[0:5])
print (outFileName[0:-4])
