from __future__ import print_function
from time import sleep
import time, sys, datetime
import numpy as np
PhidSN = 525467 # update here when switching out the phidgets that the stimulus voltage 'stimV'

def init_Flybratron():
	from flybratron import Flybratron

	#### Prep Phidgets ####
	import Phidget22.Devices.VoltageOutput
	secondAOCH 		= 1
	aout_chan 		= Phidget22.Devices.VoltageOutput.VoltageOutput()
	aout_chan.setDeviceSerialNumber(PhidSN)
	aout_chan.setChannel(secondAOCH)
	aout_chan.openWaitForAttachment(5000)
	aout_chan.setEnabled(True)
	aout_chan.setVoltage(0.)
	CLVoltage = 9. # OL voltage is pattern-dependent
	####	####

	def_phase 		= -0.19
	def_amplitude 	= 0.0
	waveform 		= 'sin2f'

	dev 			= Flybratron('/dev/ttyACM0')
	dev.param 		= {
			'amplitude' : def_amplitude, 
			'phase'     : def_phase, 
			'waveform'  : waveform,
			}

def module_crop_V(rig, aout, module_clip_V, stat_V=0., AO_ch=1, pause_dur=0.5):
	# indicate start to crop section
	if rig=='MS':
		aout.setVoltage(module_clip_V)
	else:
		aout.setVoltage(AO_ch, module_clip_V)
	sleep(pause_dur)
	if rig=='MS':
		aout.setVoltage(stat_V)
	else:
		aout.setVoltage(AO_ch, stat_V)
	sleep(pause_dur)

def run_pulse(rig, dev, aout, stat_V, sign, amplitude, phase, stim_V, duration):
	dev.param = {'amplitude': sign * amplitude, 'phase': phase}
	aout.setVoltage(sign * stim_V)
	sleep(duration)
	dev.param = {'amplitude': 0.0, 'phase': phase}
	aout.setVoltage(stat_V)

################################ Function to run PHASE function for YAW responses ################################
def yaw_phase_function(rig, dev, aout, stat_V, phase_= 0.0, amplitude_=2., sleep_dur=1., stim_dur=0.2, num_sets=2, module_clip_V = -9.8):
	phasesSet 	= (np.array(range(-25, 35, 5 ))/200.)[:-1]+phase_  # [-0.125, -0.1, -0.075, -0.05, -0.025, 0.0, 0.025, 0.05, 0.075, 0.1, 0.125, 0.15]

	print ('phase set')
	print ('Running the phase set {}'.format(phasesSet))
	print (' ')

	# indicate start to crop section
	module_crop_V(rig, aout, module_clip_V, stat_V)

	for set_num in np.arange(num_sets):

		if set_num % 2: # if set_num is odd
			phases     = phasesSet[::-1]
		else:
			phases     = phasesSet

		print ('Set {} out of {}'.format(set_num+1, num_sets))
		print (' ')

		for i in np.arange(len(phases)):

			phase_     = phases[i]
			stim_V	   = 10.*phases[i]
   			# First, + amp
			print ('amplitude {}, phase {}'.format(amplitude_, phase_))
			run_pulse(rig, dev, aout, stat_V, 1, amplitude_, phase_, stim_V, stim_dur)
			sleep(sleep_dur)

			# Second, - amp
			print ('and opposite direction')
			run_pulse(rig, dev, aout, stat_V, 1, amplitude_, phase_, stim_V, stim_dur)
			sleep(sleep_dur)
			print (' ')

		# time in between sets of directions
		sleep(2*sleep_dur)

	# indicate end to crop section
	module_crop_V(rig, aout, module_clip_V, stat_V)

################################ Function to run AMPLITUDE set function for YAW responses ################################
def amplitude_set_function(
    rig, 
    dev, 
    aout, 
    stat_V, 
    stim_array,
    phase_         = -0.075, 
    stim_dur       = 0.2, 
    inter_stim_dur = 1.0, 
    module_clip_V  = -8.2, 
    set_clip_V     = -7.8,
	LR_shuffle 	   = False
):

    set_numbers = np.unique(stim_array[:, 2]).astype(int)

    print("Total sets to run: {}".format(len(set_numbers)))
    print(" ")

    # Indicate start of section
    module_crop_V(rig, aout, module_clip_V, stat_V)
    all_amps = stim_array[:, 0].astype(float)
    unique_amps = np.unique(all_amps)
    print("Running amplitude set: {}".format(unique_amps))
    print(" ")

    for set_num in set_numbers:
        print("Set {} out of {}".format(set_num + 1, len(set_numbers)))
        print(" ")
        module_crop_V(rig, aout, set_clip_V, stat_V)
        set_trials = stim_array[stim_array[:, 2] == set_num]
        trial_idx = 0
        num_stim = len(set_trials)
        print("This set has {} stimuli".format(num_stim))
        print(" ")

        # Decide direction for this set (1 = normal L->R, -1 = flipped R->L)
        if LR_shuffle:
            set_sign = np.random.choice([-1, 1])
        else:
            set_sign = 1
        direction = "L" if set_sign == 1 else "R"

        # Extract trials for this set
        for amp, stim_V, _ in set_trials:
            amp = float(amp)
            stim_V = float(stim_V)
            stim_V = round(stim_V, 1)
            trial_idx += 1
            # --- First stimulus ---
            print('Dir {}: {}amp {}, phase {}'.format(direction,"+" if set_sign > 0 else "-", amp, phase_))
            run_pulse(rig, dev, aout, stat_V, set_sign, amp, phase_, stim_V, stim_dur)
            sleep(inter_stim_dur)

            # --- Second stimulus (opposite direction) ---
            print('And opposite direction')
            print(" ")
            run_pulse(rig, dev, aout, stat_V, -set_sign, amp, phase_, stim_V, stim_dur)
            sleep(inter_stim_dur)

        # Inter-set gap
        sleep(2 * inter_stim_dur)

    # Indicate end of section
    module_crop_V(rig, aout, module_clip_V, stat_V)
    
    
    
################################ Function to run DURATION set function for YAW responses ################################
def duration_set_function(rig, dev, aout, stat_V, phase_=0.05, amplitude=1.0, 
                      stim_dur=[0.25], after_first_dur=1.0, after_sec_dur=1.0, num_sets=2, 
                      module_clip_V=-8.2, set_clip_V=-7.6, 
                      stim_dir=1, shuffle=False, stim_V = 5.0):
    
    def get_num_trials(*args):
        lengths = []
        for arg in args:
            if isinstance(arg, (list, np.ndarray)):
                lengths.append(len(arg))
        return max(lengths) if lengths else 1
    def get_trial_value(arr, idx):
        if isinstance(arr, (list, np.ndarray)):
            if idx < len(arr):
                return arr[idx]
            else:
                return arr[-1]
        else:
            return arr
    
    print('Running the duration set: {}'.format(stim_dur))
    print('After first stim duration: {}'.format(after_first_dur))
    print('After second stim duration: {}'.format(after_sec_dur))
    print('')
    
    module_crop_V(rig, aout, module_clip_V, stat_V)

    num_trials = get_num_trials(stim_dur, after_first_dur, after_sec_dur)
    
    for set_num in range(num_sets):
        print('Set {} of {}'.format(set_num + 1, num_sets))
        print('')

        module_crop_V(rig, aout, set_clip_V, stat_V)
        
        if shuffle:
            set_sign = np.random.choice([-1, 1])
            direction = "L" if set_sign == 1 else "R"
        else:
            set_sign = 1 if stim_dir == 1 else -1
            direction = "L" if set_sign == 1 else "R"

        print('Stimulus direction order: {} then {}'.format(direction, "R" if direction == "L" else "L"))
        for i in range(num_trials):
            dur = get_trial_value(stim_dur, i)
            this_after_first = get_trial_value(after_first_dur, i)
            this_after_sec = get_trial_value(after_sec_dur, i)
            
            print('Stim duration: {} sec'.format(dur))
            
            # --- First stimulus ---
            print('Dir 1: {}amp {}, phase {}, wait {}sec'.format("+" if set_sign > 0 else "-", amplitude, phase_, this_after_first))
            run_pulse(rig, dev, aout, stat_V, set_sign, amplitude, phase_, stim_V, dur)
            sleep(this_after_first)
            
            # --- Second stimulus ---
            print('and opposite direction... wait {}sec'.format(this_after_sec))
            run_pulse(rig, dev, aout, stat_V, -set_sign, amplitude, phase_, stim_V, dur)
            sleep(this_after_sec)
            print(' ')

    module_crop_V(rig, aout, module_clip_V, stat_V)

