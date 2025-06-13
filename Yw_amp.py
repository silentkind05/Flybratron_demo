#!/usr/bin/env python
from __future__ import print_function
import time, sys, os, datetime # serial, shutil,
from time import sleep
import numpy as np
from Flybratron_functions import PhidSN

############################################################ PARAMS TO SET ############################################################
# Before running this script, make sure to set the phase, amplitude, and numSets. 
# rand should be set to True if you want to randomize the order of the amplitudes in each set.
# LR_shuffle_stat should be set to True if you want to shuffle the left and right direction of the stimulus for each set.
# stimV_map_func is a function that maps the amplitude to the voltage used in the experiment.

phase_use       			= -0.05 # change according to the phase analysis

numSets         			= 4
rand            			= False
LR_shuffle_stat				= False 
amp_series                  = np.arange(200, 1700, 200) # intended angular velocity in deg/s
stimV_map_func  			= lambda amp: 2.5 + 0.2 * np.where(amp_series == amp)[0][0] 
stim_dur_use      			= 0.25  # duration of each stimulus in seconds
inter_stim_dur_use 			= 0.75  # time between stimuli in seconds
set_clip_V_use  			= -7.4  # voltage to clip the set of trials at
module_clip_V_use 			= -8.2  # voltage to clip the module at, this is the voltage that will be used to run data analysis


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
 
 

############################################################ RUN AMP SET ############################################################

from Flybratron_functions import amplitude_set_function as asf


all_trials = []
for set_num in range(numSets):
    # create one set of [amp, stimV, set_num]
    trial_set = [[amp, stimV_map_func(amp), set_num]
                 for amp in amp_series]

    # shuffle in place only if rand is True
    if rand:
        np.random.shuffle(trial_set)

    # convert to array and collect
    all_trials.append(np.array(trial_set))

# stack into final stim_array
stim_array = np.vstack(all_trials)

asf(
    rig, dev, aout, stat_V, stim_array,
    phase_			=phase_use,
    stim_dur		=stim_dur_use,
    inter_stim_dur	=inter_stim_dur_use,
    module_clip_V	=module_clip_V_use,
    set_clip_V		=set_clip_V_use,
    LR_shuffle		=LR_shuffle_stat
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
