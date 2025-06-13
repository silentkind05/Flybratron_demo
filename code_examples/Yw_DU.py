#!/usr/bin/env python
from __future__ import print_function
import time, sys, os, datetime # serial, shutil,
from time import sleep
import numpy as np
from Flybratron_functions import PhidSN

############################################################ PARAMS TO SET ############################################################

phase_use					= -0.075

shuffle_stat				= False # whether to shuffle the left and right direction of the stimulus for sets
stim_dir_use				= 1 #(1: L-R, 2:R-L)

num_sets_use				= 4
amplitude_use				= 1200 #(angvel)
stim_dur_use    			= np.arange(0.5, 1.0+0.001, 0.25) # or fixed duration
after_fir_stim_dur_use		= 1.0 #np.arange([0.75]) # or fixed duration 
after_sec_stim_dur_use		= 4.0 #np.array([0, 1, 2, 4, 8, 16, 32, 64],dtype = int)
num_sets_use				= 4
set_clip_V_use				= -7.6
module_clip_V_use			= -8.2
stim_V_use                  = amplitude_use / 400.0



########################################################################################################################################

#### DEFAULT 
# fly line
driver 						= ' XHCS'
responder 					= ' XHCS '
rig 			            = 'MS' # mechanical stimulation
slpT            			= 0.05 
test_mode      				= False
trial_clip_V, CL_V, stat_V  = 9.99, 9., -9. # OL voltage is pattern-dependent 

# whether to save bagfiles
saveBag 		= True
recLaunchDir	= '/home/fponce/catkin_ws/src/Kinefly/launch/record.launch'
bagDir  		= '/home/fponce/bagfiles/ivo/'
#### Prep Phidgets ####
import Phidget22.Devices.VoltageOutput
secondAOCH 	= 1
aout 			= Phidget22.Devices.VoltageOutput.VoltageOutput()
aout.setDeviceSerialNumber(PhidSN)
aout.setChannel(secondAOCH)
aout.openWaitForAttachment(5000)
aout.setEnabled(True)
aout.setVoltage(0.)

################    Flybratron     ################

print ('*** Sensory modality: Mechanical ***')
from flybratron import Flybratron

dev 		= Flybratron('/dev/ttyACM0')
def_phase   = 0.
def_amplitude= 0
waveform 	= 'sin2f' # yaw
#waveform 	= 'sin1f' # roll & pitch

dev.param 	= {
	'amplitude' : def_amplitude, 
	'phase'     : def_phase, 
	'waveform'  : waveform,
	}

# brief burst to confirm ratio of driving waveform to accelometer
if 1:
	dev.operating_mode = 'free'
	dev.amplitude =200#.5
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

################ Start Experiment   ################ 
print (' ')
print ('Started trial ' + str(trialNum) + ' at: ' + str(datetime.datetime.now())[11:19])
expStartTime	= time.time()

## start recording kinefly video
if saveBag:
	parent.start() ##vid ipython command equivalent to "roslaunch Kinefly record.launch"
	print ('Saving bag file: ' + pFix + '.bag') ##vid
 
 

############################################################ RUN DUR SET ############################################################
from Flybratron_functions import duration_set_function as dsf

dsf(rig, dev, aout, stat_V, 
    phase_						= phase_use, 
    amplitude					= amplitude_use, 
    stim_dur					= stim_dur_use, 
    after_first_dur				= after_fir_stim_dur_use, 
    after_sec_dur				= after_sec_stim_dur_use, 
    num_sets					= num_sets_use, 
    module_clip_V				= module_clip_V_use, 
    set_clip_V					= set_clip_V_use, 
    stim_dir					= stim_dir_use, 
    shuffle						= shuffle_stat,
    stim_V                      = stim_V_use   
    )


########################################################################################################################################


############ turn off and close the Flybratron device ##############
param = {
	'amplitude' : def_amplitude, 
	'phase'     : def_phase, 
	}
dev.param 	= param
sleep(slpT)
dev.close()
	
### terminate phidgets  ###
aout.setVoltage(0.)
aout.setEnabled(False)
aout.close()

## Stop Bagfile
if saveBag:
	parent.shutdown() ##vid # ipython command equivalent to Ctrl+C 

elapsed 	= time.time() - expStartTime
print ('Trial time [s]:' + str(elapsed)[0:5])
print (outFileName[0:-4])
