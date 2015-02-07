'''
digitallockinhwinterface.py, hardware interface for digital lockin applications using NI PXI devices

Copyright: Zeust the Unoobian <2noob2banoob@gmail.com>, 2014

This file is part of DigitalLockin.

DigitalLockin is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

DigitalLockin is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with DigitalLockin.  If not, see <http://www.gnu.org/licenses/>.
'''

import numpy
import string

import nidaq_dllsupport as daq
import nifgen_dllsupport as fgen

#Constants
_DRIVER_NI_DAQ = 1
_DRIVER_NI_FGEN = 2
MIN_SAMPLE_FREQUENCY_MEAS_DAQ = 100 #TODO find actual value instead of arbitrary estimate
MAX_SAMPLE_FREQUENCY_MEAS_DAQ = 204800
MIN_GEN_AMPLITUDE_FGEN = 0.005635
MAX_GEN_AMPLITUDE_FGEN = 12
MAX_GEN_OFFSET_FGEN = 3
_FGEN_OUTPUTMODE = fgen.OUTPUTMODE_FUNC
_REF_CLK_SRC = 'PXI_Clk10'
_REF_CLK_FREQ = 10000000

# Checks the existence of one or more NI-DAQmx channels
# If only one channel is specified, it is allowed to be in a list but doesn't have to
# If multiple channels are specified they should be in a list
# If all specified channels exist this function returns a list of all channels
# Otherwise it returns None
def _daq_check_channels(ch, allch, dev=''):
	if isinstance(ch, list) == False:
		ch = [ch]
	for i in range(len(ch)):
		if isinstance(ch[i], (int,long)):
			if ch[i] < len(allch):
				ch[i] = allch[ch[i]].split('/')[1]
			else:
				return None
		else:
			try:
				allch.index(ch[i])
			except ValueError:
				try:
					allch.index('%s/%s' % (dev,ch[i]))
					parts=ch[i].split('/')
					if len(parts) == 2 and parts[0] == dev:
						ch[i] = parts[1]
				except ValueError:
					return None
	return ch
	

class MeasurementHardwareInterface:
	'''
	Class for the implemented digital Lock-In procedure
	Only supports NI-FGEN generation channels and NI-DAQmx measurement channels
	Supports only one generation channel
	Multiple measurement channels are supported but they must be on the same device for synchronisation reasons
	Only standard waveforms are supported for the generated signal
	'''
	
	###################################
	##### Constructor and similar #####
	###################################
	
	#Constructor
	def __init__(self, gen_dev='PXI5412_12', meas_dev='PXI4462_3', gen_ch='0', gen_meas_ch='ai0', meas_ch='ai1', gen_outmode=fgen.OUTPUTMODE_FUNC,
			waveform=fgen.WAVEFORM_SINE, Fs=MAX_SAMPLE_FREQUENCY_MEAS_DAQ, Fsignal=1000, gen_amplitude=1, gen_offset=0, gen_output_impedance=50):
		'''Constructor, also refreshes channel lists, selects channels and configures waveform to be generated'''
		daq_devs = daq.get_device_names()
		self._daq_tasks = []
		self._gen_fgen_session = None
		# Initiate generator session
		try:
			daq_devs.index(gen_dev)
			raise RuntimeError('Specified generator device \'%s\' was identified as NI-DAQmx device. NI-DAQmx generator devices are not supported at this point, please use a NI-FGEN device instead.', gen_dev)
		except ValueError:
			self._gen_driver = _DRIVER_NI_FGEN
			self._gen_fgen_session = fgen.init(gen_dev, True, True, None)
			if self._gen_fgen_session is None:
				raise RuntimeError('Failed to initialize NI-FGEN generator session')
			fgen.configure_reference_clock(self._gen_fgen_session, _REF_CLK_SRC, _REF_CLK_FREQ)
		# Initiate measurement session
		try:
			daq_devs.index(meas_dev)
			self._meas_driver = _DRIVER_NI_DAQ
			self._meas_dev = meas_dev
			daq.reset_device(self._meas_dev)
			self._daqread_task = None
		except ValueError:
			raise RuntimeError('Specified measurement device \'%s\' was not identified as NI-DAQmx device. Only NI-DAQmx measurement devices are supported at this point.', meas_dev)
		# Check existence of measurement channels and set them
		self.refresh_channels()
		self.set_meas_channels(meas_ch)
		self.set_gen_channel(gen_ch)
		self.set_gen_meas_channel(gen_meas_ch)
		# Set frequencies and amplitudes without configuring them with the driver yet
		self.set_meas_sample_frequency(Fs)
		self._set_gen_signal_frequency_passive(Fsignal)
		self._set_gen_amplitude_passive(gen_amplitude)
		self._set_gen_offset_passive(gen_offset)
		self._set_gen_waveform_passive(waveform)
		self.set_gen_output_impedance(gen_output_impedance)
		#Configure generator with parameters which have just been set
		self._configure_gen_parameters()
		self._gen_output_enabled = False
		self._gen_parameters_need_reconfigure = False
	
	def close(self):
		'''
		Terminate all NI-FGEN sessions and NI-DAQmx tasks associated with this hardware interface object
		not calling this function may lead to NI-FGEN sessions and NI-DAQmx tasks remaining open
		'''
		if self._gen_output_enabled == True:
			self.stop_generation()
		if self._gen_fgen_session is not None:
			fgen.reset(self._gen_fgen_session)
			fgen.close(self._gen_fgen_session)
			self._gen_fgen_session = None
			self._gen_driver = None
		while len(self._daq_tasks) > 0:
			daq.kill_task(self._daq_tasks.pop())
	
	##############################
	##### Channel functions ######
	##############################
	
	def refresh_channels(self):
		'''Refresh internal lists of available channels for measurement and generation'''
		if self._meas_driver == _DRIVER_NI_DAQ:
			self._daq_chans_all_i = daq.get_physical_input_channels(self._meas_dev)
		if self._gen_driver == _DRIVER_NI_DAQ:
			self._chans_all_o = daq.get_physical_output_channels(self._gen_dev)
		elif self._gen_driver == _DRIVER_NI_FGEN:
			self._chans_all_o = ['0'] #TODO find a non-hardcoded way to detect FGEN channels
	
	# Getters for channel lists (full identifiers including device id)
	def get_all_daq_input_channels(self):
		'''Get list of available input channels on selected measurement device'''
		return self._chans_all_i
	
	def get_all_daq_ouput_channels(self):
		'''Get list of available output channels on selected generation device'''
		return self._chans_all_o
	
	# Channel selectors, driver-specific, private
	def _set_daq_meas_channels(self, ch):
		meas_ch = _daq_check_channels(ch, self._daq_chans_all_i, self._meas_dev)
		if meas_ch is None:
			raise RuntimeError('Failed to set measurement channels, tried {:s}'.format(str(ch)))
		else:
			self._meas_ch = meas_ch
	
	def _set_daq_gen_meas_channel(self, ch):
		gen_meas_ch = _daq_check_channels(ch, self._daq_chans_all_i, self._meas_dev)[0]
		if gen_meas_ch is None:
			raise RuntimeError('Failed to set measurement channel for generated signal')
		else:
			self._gen_meas_ch = gen_meas_ch
	
	def _set_daq_gen_channel(self, ch):
		gen_ch = _daq_check_channels(ch, self._daq_chans_all_o, self._gen_dev)[0]
		if gen_ch is None:
			raise RuntimeError('Failed to set generation channel')
		else:
			self._gen_ch = gen_ch
	
	def _set_fgen_gen_channel(self, ch):
		# TODO this function does not yet check existence of channel 
		# In fact, I do not even know if that is possible with FGEN
		self._gen_ch = ch
	
	# Channel selectors, public driver-independent wrapper functions
	def set_meas_channels(self, ch):
		'''
		Set measurement channels, not including the channel for measuring the generated signal
		If multiple channels are specified they must be put in a list
		If only one channel is specified it can be in a list but doesn't have to
		'''
		try:
			len(ch)
		except TypeError:
			ch = [ch]
		if self._meas_driver == _DRIVER_NI_DAQ:
			self._set_daq_meas_channels(ch)
	
	def set_gen_meas_channel(self, ch):
		'''Set channel for measuring generated signal'''
		if self._meas_driver == _DRIVER_NI_DAQ:
			self._set_daq_gen_meas_channel(ch)
	
	def set_gen_channel(self, ch):
		'''Set generation channel'''
		if self._gen_driver == _DRIVER_NI_DAQ:
			self._set_daq_gen_channel(ch)
		elif self._gen_driver == _DRIVER_NI_FGEN:
			self._set_fgen_gen_channel(ch)
	
	# Getters for selected channels (short identifiers excluding device id)
	def get_measurement_channels(self):
		'''
		Get measurement channels, not including the channel for measuring the generated signal
		Always returns a list even if the number of channels is one
		'''
		return self._meas_ch
	
	def get_generation_channel(self):
		'''Get generation channel'''
		return self._gen_ch
	
	def get_generated_signal_measurement_channel(self):
		'''Get channel for measuring generated signal'''
		return self._gen_meas_ch
	
	#########################################
	##### Generator parameter functions #####
	#########################################
	
	# Local, private setters for parameters
	# These only set the parameters inside this object
	# The measurement and generation devices are not automatically configured to use the new parameters
	def _set_gen_signal_frequency_passive(self, f):
		fsignalmax = self._meas_fs / 2 #Nyquist
		fwarn = fsignalmax / 5
		if f > fsignalmax:
			self._gen_signal_frequency = fsignalmax
			logging.warning('Tried to set generated signal frequency to %f Hz, set to Nyquist limit %f Hz instead', f, fsignalmax)
		else:
			self._gen_signal_frequency = f
			if f > fwarn:
				logging.warning('Set generated signal frequency to %f Hz which is close to Nyquist limit %f Hz', f, fsignalmax)
	
	def _set_gen_amplitude_passive(self, a):
		if self._gen_driver == _DRIVER_NI_FGEN:
			amin = MIN_GEN_AMPLITUDE_FGEN
			amax = MAX_GEN_AMPLITUDE_FGEN
		if a < amin:
			self._gen_amplitude = amin
			logging.warning('Tried to set generation amplitude to %f, set to minimum %f instead', a, amin)
		elif a > amax:
			self._gen_amplitude = amax
			logging.warning('Tried to set generation amplitude to %f, set to maximum %f instead', a, amax)
		else:
			self._gen_amplitude = a
	
	def _set_gen_offset_passive(self, os):
		if self._gen_driver == _DRIVER_NI_FGEN:
			omax = MAX_GEN_OFFSET_FGEN
		if abs(os) > omax:
			self._gen_offset = omax * numpy.sign(os)
			logging.warning('Tried to set generation signal offset to %f, set to maximum %f instead', os, self._gen_offset)
		else:
			self._gen_offset = os
	
	def _set_gen_waveform_passive(self, w):
		#TODO check for allowed values
		self._gen_waveform = w
	
	# Configuration function
	def _configure_gen_parameters(self):
		if self._gen_driver == _DRIVER_NI_FGEN:
			fgen.configure_output_mode(self._gen_fgen_session, _FGEN_OUTPUTMODE)
			fgen.configure_standard_waveform(self._gen_fgen_session, self._gen_ch, self._gen_waveform, self._gen_amplitude, self._gen_offset, self._gen_signal_frequency, 0)
	
	#Public setters
	def set_gen_signal_frequency(self, f):
		'''
		Set the generated signal frequency
		If the signal generator is currently generating, this only takes effect after the generation stops and starts again
		'''
		self._set_gen_signal_frequency_passive(f)
		if self._gen_driver == _DRIVER_NI_FGEN:
			if not self._gen_output_enabled:
				fgen.set_frequency(self._gen_fgen_session, self._gen_ch, self._gen_signal_frequency)
			else:
				raise RuntimeWarning('set_gen_signal_frequency: Cannot change frequency while running, will change after restart')
				self._gen_parameters_need_reconfigure = True
	
	def set_gen_signal_amplitude(self, a):
		'''
		Set the generated signal amplitude
		If the signal generator is currently generating, this only takes effect after the generation stops and starts again
		'''
		self._set_gen_amplitude_passive(a)
		if self._gen_driver == _DRIVER_NI_FGEN:
			if not self._gen_output_enabled:
				fgen.set_amplitude(self._gen_fgen_session, self._gen_ch, self._gen_amplitude)
			else:
				raise RuntimeWarning('set_gen_signal_amplitude: Cannot change amplitude while running, will change after restart')
				self._gen_parameters_need_reconfigure = True
	
	def set_gen_output_impedance(self, Z):
		'''Set waveform generator output impedance'''
		self._gen_output_impedance = Z #TODO should there be limits for this?
	
	#Getters
	def get_gen_signal_frequency(self):
		'''Get the frequency of the generated signal'''
		return self._gen_signal_frequency
	
	def get_gen_signal_amplitude(self):
		'''Get the amplitude of the generated signal'''
		return self._gen_amplitude
	
	def get_gen_signal_offset(self):
		'''Get the DC offset of the generated signal'''
		return self._gen_offset
	
	###########################################
	##### Measurement parameter functions #####
	###########################################
	
	def set_meas_sample_frequency(self, fs):
		'''Set the measurement sample frequency'''
		if self._meas_driver == _DRIVER_NI_DAQ:
			fmin = MIN_SAMPLE_FREQUENCY_MEAS_DAQ
			fmax = MAX_SAMPLE_FREQUENCY_MEAS_DAQ
		if fs < fmin:
			self._meas_fs = fmin
			logging.warning('Tried to set measurement sample frequency to %f Hz, set to minimum %f Hz instead', fs, fmin)
		elif fs > fmax:
			self._meas_fs = fmax
			logging.warning('Tried to set measurement sample frequency to %f Hz, set to maximum %f Hz instead', fs, fmax)
		else:
			self._meas_fs = fs
	
	def get_meas_sample_frequency(self):
		'''Get the sample frequency of the signal analyser'''
		return self._meas_fs
	
	####################################
	##### Output control functions #####
	####################################
	
	def start_generation(self):
		'''Start waveform generation, configure output impedance and enable output'''
		if self._gen_driver == _DRIVER_NI_FGEN:
			fgen.set_output_impedance(self._gen_fgen_session, self._gen_ch, self._gen_output_impedance)
			fgen.initiate_generation(self._gen_fgen_session)
			fgen.output_enable(self._gen_fgen_session)
		self._gen_output_enabled = True
	
	def stop_generation(self):
		'''Stop waveform generation and disable output'''
		if self._gen_driver == _DRIVER_NI_FGEN:
			fgen.output_disable(self._gen_fgen_session)
			fgen.abort_generation(self._gen_fgen_session)
		self._gen_output_enabled = False
		if self._gen_parameters_need_reconfigure:
			self._configure_gen_parameters()
			self._gen_parameters_need_reconfigure = False
	
	################################
	##### Measurement function #####
	################################
	
	#The actual function to do the measurement
	def do_measurement(self, nsamples, vmin=-10, vmax=10, timeout=1, config='PSEUDODIFF'):
		'''
		Measure signals
		Returns a multidimensional array.
		data[0][:] contains the measured generated signal, data[1:][:] contains the other measured signals
		The timeout you specify is increased the time the measurement should take so you don't have to calculate this time yourself
		'''
		# Some convenience variables
		ch = [self._gen_meas_ch] + self._meas_ch
		for i in range(len(ch)):
			ch[i] = '%s/%s' % (self._meas_dev, ch[i])
		ch_str = string.join(ch, ',')
		timeout = timeout + float(nsamples) / self._meas_fs
		# Read data
		if self._meas_driver == _DRIVER_NI_DAQ:
			data = daq.read(ch_str, nsamples, self._meas_fs, vmin, vmax, timeout, config, _REF_CLK_SRC, _REF_CLK_FREQ)
		# Split data into multidimensional array, separated by channels, and return
		datasplit=[[]]*(len(self._meas_ch)+1) # Empty list
		for i in range(len(datasplit)):
			datasplit[i] = data[i*nsamples:(i+1)*nsamples]
		return datasplit
	
	#Convenience functions to measure for a specified amount of signal periods or seconds
	def measure_periods(self, nperiods, vmin=-10, vmax=10, timeout=10, config='PSEUDODIFF'):
		'''Version of do_measurement() where you specify the number of signal periods (not necessarily integer) instead of the number of samples'''
		nsamples = int(round(nperiods * self._meas_fs / float(self._gen_signal_frequency)))
		return self.do_measurement(nsamples, vmin, vmax, timeout, config)
	
	def measure_seconds(self, nseconds, vmin=-10, vmax=10, timeout=10, config='PSEUDODIFF'):
		'''Version of do_measurement() where you specify the time in seconds (not necessarily integer) instead of the number of samples'''
		nsamples = int(round(nseconds*self._meas_fs))
		return self.do_measurement(nsamples, vmin, vmax, timeout, config)

	#Functions to measure in several steps
	def start_measurement(self, vmin=-10, vmax=10, config='PSEUDODIFF', bufsize=204800):
		'''Instruct the hardware to start measuring but don't acquire any samples to the computer just yet'''
		# Some convenience variables
		ch = [self._gen_meas_ch] + self._meas_ch
		for i in range(len(ch)):
			ch[i] = '%s/%s' % (self._meas_dev, ch[i])
		ch_str = string.join(ch, ',')
		# Start acquisition
		if self._meas_driver == _DRIVER_NI_DAQ:
			self._daqread_task = daq.read_init(ch_str, vmin, vmax, config)
			if self._daqread_task is not None:
				self._daq_tasks.append(self._daqread_task)
				if daq.set_refclk(self._daqread_task, _REF_CLK_SRC, _REF_CLK_FREQ):
					daq.read_start(self._daqread_task, -bufsize, self._meas_fs)
				else:
					raise RuntimeWarning('Failed to configure reference clock for NI-DAQmx measurement task')
			else:
				raise RuntimeError('Could not get NI-DAQmx task handle for measurement')
	
	def retrieve_samples(self, nsamples=1, timeout=1.0, assumebuffered=False):
		'''
		Retrieve the specified number of samples from the hardware
		The timeout you specify is increased the time the measurement should take so you don't have to calculate this time yourself
		This function reshapes the measured data to a channels x samples array
		'''
		if not assumebuffered:
			timeout += float(nsamples) / self._meas_fs
		if self._meas_driver == _DRIVER_NI_DAQ:
			data = daq.read_get_some_samples(self._daqread_task, nsamples, timeout, len(self._meas_ch)+1)
			if len(data) == nsamples * (len(self._meas_ch)+1):
				return data.reshape([len(self._meas_ch)+1, nsamples])
			elif nsamples == -1:
				return data.reshape([len(self._meas_ch)+1, len(data) / (len(self._meas_ch)+1)])
			else:
				raise RuntimeWarning('retrieve_samples: expected {:d} samples but got {:d}'.format(nsamples * (len(self._meas_ch)+1), len(data)))
				return data.reshape([len(self._meas_ch)+1, len(data) / (len(self._meas_ch)+1)])
	
	def retrieve_periods(self, nperiods=1, timeout=1.0, assumebuffered=False):
		'''
		Retrieve a number of samples corresponding to the specified number of signal periods
		The timeout you specify is increased the time the measurement should take so you don't have to calculate this time yourself
		'''
		nsamples = int(round(nperiods * self._meas_fs / float(self._gen_signal_frequency)))
		return self.retrieve_samples(nsamples, timeout, assumebuffered)
	
	def retrieve_seconds(self, nseconds=1, timeout=0.1, assumebuffered=True):
		'''
		Retrieve a number of samples corresponding to the specified time in seconds, rounded to an integer amount of signal periods
		The timeout you specify is increased the time the measurement should take so you don't have to calculate this time yourself
		'''
		nperiods = round(nseconds*self._gen_signal_frequency)
		return self.retrieve_periods(nperiods, timeout, assumebuffered)
	
	def end_measurement(self):
		'''Kill the measurement task'''
		if self._meas_driver == _DRIVER_NI_DAQ:
			daq.kill_task(self._daqread_task)
			self._daq_tasks.remove(self._daqread_task)
			self._daqread_task = None
	
	###################################
	##### Miscellaneous functions #####
	###################################
	
	def measured_samples_in_instrument_buffer(self):
		'''Find out how many samples are left in the instrument's sample buffer. Returns None on failure.'''
		if self._meas_driver == _DRIVER_NI_DAQ:
			return daq.num_samples_in_instrument_buffer(self._daqread_task)
