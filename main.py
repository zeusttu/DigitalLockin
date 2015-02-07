'''
main.py, Main loop for digital Lockin amplifier using NI PXI devices

Copyright: Zeust the Unoobian <2noob2banoob@gmail.com>, 2014

Some files which are part of this program are derivative works of code copyrighted by:
Reinier Heeres <reinier@heeres.eu>, 2008-2010
Those files will individually mention him in their copyright notice.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This programn is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

Written for cross-compatibility with Python 2 and 3, though only tested in Python2

Actual hardware interaction and real measurements only work on Windows machines
with the NI-DAQmx and NI_FGEN drivers (both from National Instruments) installed.
Other features (most notably the simulated mode) also work on other, potentially
non-Windows systems, provided they have a sufficiently recent version of Python
installed along with the necessary Python packages.

Supported commands:
	SELECT
		Select a lock-in instrument
	SET F|FS|A|T|PHASEOFFSET|MEASCH <value>
		Set the value of a variable of the selected lock-in instrument
	GET RPHIBUFFER
		Get multiline representation of all values of R and PHI acquired from
		the selected lock-in instrument since last time they were queried
	GET RPHI|R|PHI|XY|X|Y
		Get the last acquired values of R/PHI/X/Y from the selected lock-in
		This function may still return multiple values (comma-separated) when
		multiple channels are in use
	GET F|FS|A|T|PHASEOFFSET
		Get value of excitation/measurement control variable of selected lock-in instrument
	START
		Start measurements on the selected lock-in instrument
	STOP
		Stop measurements on the selected lock-in instrument
	CLOSE
		Close the selected lock-in instrument
	CLOSE ALL
		Close all lock-in instruments and exit
	PHASENULL
		Set the phase offset of the current lock-in instrument to the current phase

Known issues:
 *  When setting parameters while a measurement is running, the parameters are never really
    set in the hardware. [Now solved for generator parameters (except channel) but not yet for Fs and channels]
 *  No getter for dl_selected
 *  When deleting a lock-in, the identifier of each lock-in with a higher identifier
    than the deleted one decreases by one, which may not be the best behaviour

Potential code readability/architecture enhancements:
 *  This file and digitallockin.py contain two lock-in implementations, the code may be
    cleaned up a lot by removing one of them
 *  Some global variables might be better suited as properties of a DigitalLockin object
 *  A DigitalLockin object currently holds a MeasurementHardwareInterface as a member
    (if it  is not simulated), while in simulation mode it handles everything by itself.
    Maybe it's a better idea to create a SimulatedHardwareInterface class and move all
    simulation-specific code there.
 *  One may argue that using the HardwareInterface as a member is not as elegant as making
    DigitalLockin inherit from it. In principle I would agree; however since it would not
	use the MeasurementHardwareInterface in simulated mode that would require a conditional
	inheritance which is not possible as far as I know.

Fixed issues since the "known issues" list was created:
 *  The instrument buffer is only read once per integration time, without checking if the size
    of this buffer is sufficient to hold that many samples if the integration time is long.
 *  The sizes of the amplitude and phase buffers are not constrained.
    This may lead to crashes if someone starts a measurement but does not retrieve the data.
 *  Only one channel supported by main.py
 *  PHASENULL not yet implemented
 *  Clock mismatch between the acquisition computer and the PXI chassis may lead to a deviation
    between the number of samples in the instrument buffer and the number of samples the
    computer thinks are in that buffer. Eventually this may lead to the buffer containing less
    samples than the computer tries to read from it (and assumes that the samples are there),
    or it may lead to values in the buffer being overwritten before the computer retrieves
    them, while the computer will assume that this does not happen.
 *  No instruction to retrieve detected reference signal amplitude (to which everything is normalised)
'''

import logging
logging.root.setLevel(logging.INFO)
import sys
import signal
import serial
import time
import numpy
import string

import digitallockin as dlm

##### Handle keyboard interrupt
interrupt_received = False
def interrupt_handler(signal, frame):
	global interrupt_received
	if interrupt_received:
		logging.error('Got second human abort, exiting non-gracefully')
		raise RuntimeError('Got second human abort, exiting non-gracefully')
	else:
		interrupt_received = True
		logging.info('got keyboard interrupt or similar signal, preparing to exit...')
signal.signal(signal.SIGINT, interrupt_handler)

##### Constants which are probably (but not necessarily)
##### the same for you as for the author of the code
MEASUREMENT_TIME_MAX = 0.5
R_PHI_BUFFER_LEN_MAX = 1000
CHANNELS_MAX = 3
MAX_BUFFERED_SAMPLES_AFTER_READ = 50000 # about 250 ms at maximum Fs
MIN_BUFFERED_SAMPLES_AFTER_READ = 2000 # about 10 ms at maximum Fs
TIME_CHANGE_IF_TOO_MANY_BUFFERED_SAMPLES = 0.01 # 10 ms
TIME_CHANGE_IF_TOO_FEW_BUFFERED_SAMPLES = 0.001 # 1 ms

##############################################################
#####     Variables specific to our measurement setup    #####
##### Other people using our code may want to edit these #####
##############################################################
available_waveform_generators = ['PXI5412_12', 'PXI5412_14']
available_signal_analysers = ['PXI4462_3', 'PXI4462_4']
comport_to_use = 'COM4' # Part of virtual pair COM3 <-> COM4
#comport_to_use = '/dev/pts/3' # Part of virtual pair /dev/pts/2 <-> /dev/pts/3
comport_timeout = 0.001
integrationtime_default = 0.1

############################
##### Helper functions #####
############################
def _get_lockin(idx):
	'''
	Checks if lock-in with index <idx> exists (first index = 1)
	Returns it if it does, throws error otherwise
	'''
	global dl
	if 0 < idx <= len(dl):
		return dl[idx-1]
	else:
		raise RuntimeError('No lock-in selected')

def _fmt_array_for_com(x):
	'''
	Formats array as string to be sent over com port
	The last dimension is separated by commas and spaces
	All other dimensions are separated by newlines
	'''
	x = numpy.array(x)
	if x.ndim > 1:
		y = ''
		for i in range(len(x)):
			y = '{:s}{:s}'.format(y, _fmt_array_for_com(x[i]))
		return 'OK {:d} lines\n{:s}'.format(len(x), y)
	else:
		if len(x) == 0:
			return 'EMPTY'
		else:
			return string.join([str(z) for z in x], ', ') + '\n'

def _new_lockin():
	'''
	Generates a new lock-in object with the first available waveform generator and the first available signal analyser
	Also appends appropriate default values to all the arrays used for bookkeeping of lock-ins
	'''
	global waveform_generators_used, signal_analysers_used, dl, available_waveform_generators, available_signal_analysers, acquiretimes, measperint, integrationtimes, t_lastmeas, meas_in_cur_int, t_lastintegration, ref_amplitude_buffer, amplitude_buffer, phase_buffer, amplitude_num, phase_num, phase_offset
	try:
		gen_dev_idx = waveform_generators_used.index(False)
	except Exception as e:
		raise RuntimeError('Out of waveform generators:\n{:s}'.format(str(e)))
	try:
		meas_dev_idx = signal_analysers_used.index(False)
	except Exception as e:
		raise RuntimeError('Out of signal analysers:\n{:s}'.format(str(e)))
	waveform_generators_used[gen_dev_idx] = True
	signal_analysers_used[meas_dev_idx] = True
	dl.append(dlm.DigitalLockin(gen_dev=available_waveform_generators[gen_dev_idx], meas_dev=available_signal_analysers[meas_dev_idx]))
	measperint.append(1) #TODO don't assume max_meastime > integrationtime_default
	acquiretimes.append(integrationtime_default) #TODO don't assume max_meastime > integrationtime_default
	integrationtimes.append(integrationtime_default)
	t_lastmeas.append(0.)
	meas_in_cur_int.append(0)
	t_lastintegration.append(0.)
	ref_amplitude_buffer.append(numpy.zeros([R_PHI_BUFFER_LEN_MAX]))
	amplitude_buffer.append(numpy.zeros([R_PHI_BUFFER_LEN_MAX, CHANNELS_MAX]))
	phase_buffer.append(numpy.zeros([R_PHI_BUFFER_LEN_MAX, CHANNELS_MAX]))
	amplitude_num.append(0)
	phase_num.append(0)
	phase_offset.append(0.)

def _new_simulated_lockin():
	'''
	Generates a new lock-in object using simulated instruments
	Also appends appropriate default values to all the arrays used for bookkeeping of lock-ins
	'''
	global dl, acquiretimes, measperint, integrationtimes, t_lastmeas, meas_in_cur_int, t_lastintegration, ref_amplitude_buffer, amplitude_buffer, phase_buffer, amplitude_num, phase_num, phase_offset
	dl.append(dlm.DigitalLockin(simulated=True))
	measperint.append(1) #TODO don't assume max_meastime > integrationtime_default
	acquiretimes.append(integrationtime_default) #TODO don't assume max_meastime > integrationtime_default
	integrationtimes.append(integrationtime_default)
	t_lastmeas.append(0.)
	meas_in_cur_int.append(0)
	t_lastintegration.append(0.)
	ref_amplitude_buffer.append(numpy.zeros([R_PHI_BUFFER_LEN_MAX]))
	amplitude_buffer.append(numpy.zeros([R_PHI_BUFFER_LEN_MAX, CHANNELS_MAX]))
	phase_buffer.append(numpy.zeros([R_PHI_BUFFER_LEN_MAX, CHANNELS_MAX]))
	amplitude_num.append(0)
	phase_num.append(0)
	phase_offset.append(0.)

def _pwrite(p, stw):
	'''Helper function to write string <stw> to port <p> as UTF-8 encoded byte list'''
	if stw[-1] != '\n':
		stw += '\n'
	p.write(bytes(bytearray(stw, encoding='utf-8')))
	#logging.info('Response: {:s}'.format(stw.strip()))

def _inttime_to_meastime(inttime, max_meastime):
	'''Determine how many times to measure per integration period, returns the number of times and the time per measurement'''
	i = int(numpy.ceil(float(inttime) / max_meastime))
	return (i, float(inttime) / i)

################################
##### Functional functions #####
################################

def select_lockin(s):
	'''
	Selects a measurement channel
	If the measurement channel does not exist but is the next one to be created, tries to create it
	'''
	global dl, dl_selected
	if s <= len(dl): #Existing channel
		dl_selected = s
		logging.info('Selected lockin {:d}'.format(s))
	elif s == len(dl) + 1: #New channel
		try:
			_new_lockin()
			#_new_simulated_lockin()
			dl_selected = s
			logging.info('Created and selected lockin {:d}'.format(s))
		except Exception as e:
			#raise RuntimeError('SELECT: Failed to create new channel:\n{:s}'.format(str(e)))
			e.args = ('SELECT: Failed to create new channel:',) + (e.args)
			raise
	else: #Uncreatable channel
		raise RuntimeError('SELECT: Channel {:d} does not exist and is not the next one to be created'.format(s))

def get(idx, var, firsttry=True):
	'''
	Getter for variable with name <var> on lock-in <li>
	Returns value in COMport-compliant string format
	Supported variables:
		RPHIBUFFER  : buffer of floats : excitation amplitude and detected amplitude and phase relative to excitation signal
		RPHI        :  list of floats  : excitation amplitude and detected amplitude and phase relative to excitation signal
		R           :  list of floats  : detected amplitude relative to excitation amplitude
		PHI         :  list of floats  : detected phase relative to excitation phase
		XY          :  list of floats  : detected X and Y components relative to excitation signal
		X           :  list of floats  : detected X component relative to excitation signal
		Y           :  list of floats  : detected Y component relative to excitation signal
		F           :       float      : excitation frequency
		FS          :       float      : sample frequency
		A           :       float      : excitation amplitude
		T           :       float      : integration time
		PHASEOFFSET :       float      : phase offset (set by PHASENULL)
	For a buffer of floats, a multi-line representation of the buffer is returned
	Values corresponding to the same integration interval but different channels are printed on the same line, separated by commas
	Values corresponding to subsequent integration intervals are printed on subsequent lines
	If R and PHI are both requested and there are multiple channels, both variables are still printed on
	the same line, first grouped by variable and then by channel. For example if there are 3 channels:
	R(ch1), R(ch2), R(ch3), PHI(ch1), PHI(ch2), PHI(ch3)
	'''
	try:
		global ref_amplitude_buffer, amplitude_buffer, phase_buffer, integrationtimes, phase_offset
		li = _get_lockin(idx)
		if var == 'rphibuffer':
			if amplitude_num[idx-1] == 0 or phase_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read RPHIBUFFER but it is not available, will try again next iteration')
				return None
			dnum = amplitude_num[idx-1] - phase_num[idx-1]
			rref = ref_amplitude_buffer[idx-1][max(0,dnum):amplitude_num[idx-1]]
			r = amplitude_buffer[idx-1][max(0,dnum):amplitude_num[idx-1],:dl[idx-1].get_num_meas_ch()]
			r = numpy.append(numpy.array([rref]).T, r, axis=1)
			phi = phase_buffer[idx-1][max(0,-dnum):phase_num[idx-1],:dl[idx-1].get_num_meas_ch()]
			bufstr = _fmt_array_for_com(numpy.append(r, phi, 1))
			amplitude_num[idx-1] = 0
			phase_num[idx-1] = 0
			return 'OK ' + bufstr
		elif var == 'rphi':
			return 'OK ' + _fmt_array_for_com(numpy.append(*li.continuous_get_r_phi()))
			''' Non-continuous lock-in
			if amplitude_num[idx-1] == 0 or phase_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read RPHI but it is not available, will try again next iteration')
				return None
			rref = ref_amplitude_buffer[idx-1][amplitude_num[idx-1]-1]
			r = amplitude_buffer[idx-1][amplitude_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			r = numpy.append(numpy.array([rref]).T, r, axis=1)
			phi = phase_buffer[idx-1][phase_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			valstr = _fmt_array_for_com(numpy.append(r, phi))
			amplitude_num[idx-1] = 0
			phase_num[idx-1] = 0
			return 'OK ' + valstr
			'''
		elif var == 'r':
			if amplitude_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read R but it is not available, will try again next iteration')
				return None
			logging.info('Returning {:d} values of R'.format(amplitude_num[idx-1]))
			rvalstr = _fmt_array_for_com(amplitude_buffer[idx-1][amplitude_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()])
			amplitude_num[idx-1] = 0
			return 'OK ' + rvalstr
		elif var == 'phi':
			if phase_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read PHI but it is not available, will try again next iteration')
				return None
			logging.info('Returning {:d} values of phi'.format(phase_num[idx-1]))
			phivalstr = _fmt_array_for_com(phase_buffer[idx-1][phase_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()])
			phase_num[idx-1] = 0
			return 'OK ' + phivalstr
		elif var == 'xy':
			if amplitude_num[idx-1] == 0 or phase_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read XY but it is not available, will try again next iteration')
				return None
			r = amplitude_buffer[idx-1][amplitude_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			phi = phase_buffer[idx-1][phase_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			valstr = _fmt_array_for_com(numpy.append(r*numpy.cos(phi), r*numpy.sin(phi)))
			amplitude_num[idx-1] = 0
			phase_num[idx-1] = 0
			return 'OK ' + valstr
		elif var == 'x':
			if amplitude_num[idx-1] == 0 or phase_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read X but it is not available, will try again next iteration')
				return None
			r = amplitude_buffer[idx-1][amplitude_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			phi = phase_buffer[idx-1][phase_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			xstr = _fmt_array_for_com(r * numpy.cos(phi))
			amplitude_num[idx-1] = 0
			phase_num[idx-1] = 0
			return 'OK ' + xstr
		elif var == 'y':
			if amplitude_num[idx-1] == 0 or phase_num[idx-1] == 0:
				if firsttry:
					logging.warning('GET: Tried to read Y but it is not available, will try again next iteration')
				return None
			r = amplitude_buffer[idx-1][amplitude_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			phi = phase_buffer[idx-1][phase_num[idx-1]-1,:dl[idx-1].get_num_meas_ch()]
			ystr = _fmt_array_for_com(r * numpy.sin(phi))
			amplitude_num[idx-1] = 0
			phase_num[idx-1] = 0
			return 'OK ' + ystr
		elif var == 'f':
			return 'OK {:f}\n'.format(li.get_f())
		elif var == 'fs':
			return 'OK {:f}\n'.format(li.get_fs())
		elif var == 'a':
			return 'OK {:f}\n'.format(li.get_gen_amplitude())
		elif var == 't':
			return 'OK {:f}\n'.format(integrationtimes[idx-1])
		elif var == 'phaseoffset':
			return 'OK {:f}\n'.format(phase_offset[idx-1])
		else:
			raise RuntimeError('GET: invalid variable {:s}'.format(var))
	except Exception as e:
		e.args = ('GET({:s}) failed'.format(var),) + e.args
		raise

def set(idx, var, val):
	'''
	Sets variable with name <var> to value <val> on lock-in <li> (accepts <val> as string)
	Supported variables:
		F           :  float  : generated signal frequency [Hz]
		FS          :  float  : sample frequency [Hz]
		A           :  float  : generated signal amplitude [V pk-pk]
		T           :  float  : integration time [s]
		PHASEOFFSET :  float  : phase which is considered zero [radians]
		MEASCH : list(string) : measurement channels (excluding the one measuring the generated signal)
	Examples:
		set('F', '1000.0')
		set('MEASCH', 'ai1,ai2,ai3')
	Note: setting the integration time while a measurement is running might lead to timing issues and/or skipped samples, so don't do this
	'''
	global integrationtimes, measperint, acquiretimes, phase_offset
	li = _get_lockin(idx)
	if var == 'f':
		li.set(F=float(val))
	elif var == 'fs':
		li.set(Fs=float(val))
	elif var == 'a':
		li.set(A=float(val))
	elif var == 't':
		li.set_flt_time_constant(float(val))
		integrationtimes[idx-1] = float(val)
		(measperint[idx-1], acquiretimes[idx-1]) = _inttime_to_meastime(val, MEASUREMENT_TIME_MAX)
		logging.info('Tint={:.2f}, dt={:.2f}, ratio={:d}'.format(integrationtimes[idx-1], acquiretimes[idx-1], measperint[idx-1]))
	elif var == 'phaseoffset':
		phase_offset[idx-1] = float(val)
	elif var == 'measch':
		ch = val.split(',')
		for i in range(len(ch)):
			ch[i] = ch[i].strip()
			try:
				ch[i] = int(ch[i])
			except Exception:
				None
		logging.info('channels: {:s}'.format(str(ch)))
		li.set_channels(meas_ch=ch)
	elif var == 'alpha':
		li.set_flt_alpha(float(val))
	else:
		raise RuntimeError('SET: invalid variable {:s} (tried to assign value {:s})'.format(var, val))

def phasenull(ch='1'):
	'''Set the phase offset to compensate for the last measured phase of the <ch>th measurement channel'''
	try:
		ch = int(ch) - 1
	except ValueError as e:
		logging.warning('Cannot apply phase-nulling to channel number {:s}, will default to channel 1'.format(ch))
		ch = 0
	global phase_offset
	if bool(dl_selected):
		if bool(phase_num[dl_selected-1]):
			phase_offset_increase = phase_buffer[dl_selected-1][phase_num[dl_selected-1]-1, ch]
			logging.info('Phase offset for lock-in {:d} was {:f} rad, increases by {:f} rad based on channel {:d}'.format(dl_selected, phase_offset[dl_selected-1], phase_offset_increase, ch+1))
			phase_offset[dl_selected-1] += phase_offset_increase
		else:
			raise RuntimeError('Could not phase-null because no phase information is available')
	else:
		raise RuntimeError('Could not phase-null because no lock-in is selected')

def start_lockin(idx):
	'''Starts measurement on lock-in amplifier with index <idx> (first index = 1)'''
	li = _get_lockin(idx)
	li.start_measurement()
	t_lastmeas[idx-1] = time.time()
	meas_in_cur_int[idx-1] = 0
	t_lastintegration[idx-1] = t_lastmeas[idx-1]

def stop_lockin(li):
	'''Stops measurement on lock-in amplifier <li>'''
	li.stop_measurement()

def close_lockin(idx):
	'''Closes lock-in device with index <idx> (first index = 1)'''
	global waveform_generators_used, available_waveform_generators, signal_analysers_used, available_signal_analysers, dl, integrationtimes, t_lastmeas, meas_in_cur_int, t_lastintegration, ref_amplitude_buffer, amplitude_buffer, phase_buffer, amplitude_num, phase_num, phase_offset, dl_selected
	li = _get_lockin(idx)
	li.close()
	waveform_generators_used[available_waveform_generators.index(li.gen_dev_str)] = False
	signal_analysers_used[available_signal_analysers.index(li.meas_dev_str)] = False
	del dl[idx-1], integrationtimes[idx-1], measperint[idx-1], acquiretimes[idx-1], t_lastmeas[idx-1], meas_in_cur_int[idx-1], t_lastintegration[idx-1], ref_amplitude_buffer[idx-1], amplitude_buffer[idx-1], phase_buffer[idx-1], amplitude_num[idx-1], phase_num[idx-1], phase_offset[idx-1]
	if dl_selected == idx:
		dl_selected = 0

##########################
##### Loop functions #####
##########################
def command_loop(cmdnow):
	'''
	Interprets command that has been written to the COM-port and executes the command.
	Checks if cmdnow is a complete string.
	cmdnow (str) = command from com port (pcom.readline()).
	'''
	global cmd, dl_selected, interrupt_received
	if len(cmdnow) > 0:
		cmd = '{:s}{:s}'.format(cmd, cmdnow)
	if len(cmd) > 0 and cmd[-1] == '\n':
		resetcmd = True
		if len(cmdnow) > 0:
			logging.info('command: {:s}'.format(cmd[:-1]))
		try:
			if cmd[:5].upper() == '*IDN?':
				_pwrite(pcom, 'DigitalLockin virtual/software-based lock-in amplifier\n')
			elif cmd[:6].upper() == 'SELECT':
				select_lockin(int(cmd[7:-1]))
				_pwrite(pcom, 'OK\n')
			elif cmd[:3].upper() == 'GET':
				gotstr = get(dl_selected, cmd[4:-1].lower())
				if gotstr is not None:
					_pwrite(pcom, gotstr)
				else:
					resetcmd = False
			elif cmd[:3].upper() == 'SET':
				setstr = cmd[4:-1].split(' ')
				if len(setstr) == 2:
					set(dl_selected, setstr[0].lower(), setstr[1])
					_pwrite(pcom, 'OK\n')
				else:
					raise RuntimeError('SET: Incorrect number of arguments ({:d}): {:s}'.format(len(setstr), cmd[4:-1]))
			elif cmd[:5].upper() == 'START':
				start_lockin(dl_selected)
				_pwrite(pcom, 'OK\n')
			elif cmd[:4].upper() == 'STOP':
				stop_lockin(_get_lockin(dl_selected))
				_pwrite(pcom, 'OK\n')
			elif cmd[:5].upper() == 'CLOSE':
				if cmd[6:9].upper() == 'ALL':
					interrupt_received = True
					_pwrite(pcom, 'OK\n')
				elif dl_selected != 0:
					close_lockin(dl_selected)
					dl_selected = 0
					_pwrite(pcom, 'OK\n')
			elif cmd[:9].upper() == 'PHASENULL':
				if len(cmd) > 11:
					phasenull(cmd[10:-1])
				else:
					phasenull()
				_pwrite(pcom, 'OK\n')
			else:
				raise RuntimeError('Unknown command: {:s}'.format(cmd[:-1]))
		except Exception:
			#e.args = ('Error during command {:s}'.format(cmd.strip()),) + e.args
			#logging.error(str(e))
			logging.exception('Error during command {:s}:'.format(cmd.strip()))
			_pwrite(pcom, 'ERROR\n')
		finally:
			if resetcmd:
				cmd = ''

def measure_loop():
	'''
	For each lock-in, if the measurement time has passed, retrieve the data.
	If a full integration time has passed, process the data and store in ref_amplitude_buffer, amplitude_buffer and phase_buffer.
	'''
	t = time.time() #Only function that's cross-python compatible
	for i in range(len(dl)):
		if dl[i].is_measuring() and t > t_lastmeas[i] + acquiretimes[i]:
			# Measure samples
			t_lastmeas[i] += dl[i].retrieve_seconds(acquiretimes[i], bool(meas_in_cur_int[i]))
			
			# Compensate for potential clock mismatch between PXI chassis and PC
			buffer_samples = dl[i].num_measured_samples_in_instrument_buffer()
			if buffer_samples > MAX_BUFFERED_SAMPLES_AFTER_READ:
				t_lastmeas[i] -= TIME_CHANGE_IF_TOO_MANY_BUFFERED_SAMPLES
				logging.info('Made next measurement earlier ({:d} samples left in buffer)'.format(buffer_samples))
			elif buffer_samples < MIN_BUFFERED_SAMPLES_AFTER_READ:
				t_lastmeas[i] += TIME_CHANGE_IF_TOO_FEW_BUFFERED_SAMPLES
				logging.info('Postponed next measurement ({:d} samples left in buffer)'.format(buffer_samples))
			
			# Calculate R and PHI if a full integration period has passed
			meas_in_cur_int[i] += 1
			if meas_in_cur_int[i] == measperint[i]:
				meas_in_cur_int[i] = 0
				if amplitude_num[i] == R_PHI_BUFFER_LEN_MAX:
					logging.warning('Amplitude buffer reached capacity, discarding whole buffer')
					amplitude_num[i] = 0
				if phase_num[i] == R_PHI_BUFFER_LEN_MAX:
					logging.warning('Phase buffer reached capacity, discarding whole buffer')
					phase_num[i] = 0
				(ref_amplitude_buffer[i][amplitude_num[i]], amplitude_buffer[i][amplitude_num[i],:dl[i].get_num_meas_ch()], phase_now) = dl[i].process_data()
				phase_buffer[i][phase_num[i],:dl[i].get_num_meas_ch()] = phase_now - phase_offset[i]
				amplitude_num[i] += 1
				phase_num[i] += 1
				dl[i].printmainresults(compactfmt=True)

def measure_loop_continuous():
	'''
	For each lock-in, retrieve the data.
	'''
	for i in range(len(dl)):
		if dl[i].is_measuring():
			dl[i].continuous_retrieve_and_filter()

########################
##### Main program #####
########################

# Initialization
pcom = serial.Serial(comport_to_use, timeout=comport_timeout)
waveform_generators_used = [False] * len(available_waveform_generators)
signal_analysers_used = [False] * len(available_signal_analysers)
dl = [] # Digital lockin object array
integrationtimes = []
acquiretimes = []
measperint = []
t_lastmeas = []
meas_in_cur_int = []
t_lastintegration = []
amplitude_buffer = []
ref_amplitude_buffer = []
phase_buffer = []
amplitude_num = []
phase_num = []
phase_offset = []
dl_selected = 0 # Default = none.
cmd = ''
logging.info('Initialization done')

# Main loop
while not interrupt_received:
	try:
		# Reads com port and parses to cmd, and executes chosen command.
		if len(cmd) and cmd[-1] == '\n':
			command_loop('')
		else:
			command_loop(pcom.readline())
	except serial.SerialException as e:
		if str(e) != 'read failed: (4, \'Interrupted system call\')':
			logging.error('Could not read command:\n{:s}'.format(str(e)))
	measure_loop_continuous()
	time.sleep(0.01) # Sleep for 10 ms to allow UI interaction

# Closing
for dli in dl:
	dli.close()
_pwrite(pcom, 'EXIT\n')
pcom.close()
logging.info('Now exiting.')

