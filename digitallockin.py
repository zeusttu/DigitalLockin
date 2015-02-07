'''
digitallockin.py, Digital Lockin amplifier using NI PXI devices

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

import matplotlib.pyplot as plt
import numpy
import sys
import random
import logging
import time #for benchmark

try:
	import digitallockinhwinterface as hwi
	CAN_MEASURE = True
	MAX_SAMPLE_FREQUENCY = hwi.MAX_SAMPLE_FREQUENCY_MEAS_DAQ
	logging.info('Loaded hardware interface module.')
except Exception:
	CAN_MEASURE = False
	MAX_SAMPLE_FREQUENCY = 200000
	logging.exception('Could not load hardware interface. You can perform simulated lock-in sequences on virtual noise sources but not actual measurements on physical devices. Details:')

# Simple Euler integration
# A sample of the output is the sum of all input samples up to the same sample number
# Uses a unit timestep instead of taking the timestep as a parameter so the result is not properly scaled
def _integrate(x):
	if x[0].size != 1:
		y=numpy.zeros([len(x),len(x[0])])
		for i in range(len(x)):
			y[i][:] = _integrate(x[i][:])
	else:
		y = numpy.zeros(x.size)
		y[0] = x[0]
		for i in xrange(1,x.size):
			y[i] = y[i-1] + x[i]
	return y

# Filtering function
# Every filter is realised as a cascade of first-order alpha filters
# The initial value of each filter is the average of its input
# The alpha values of subsequent first-order filters are supplied as a list
# For a first-order filter you can also supply just the element rather than a one-dimensional list
# The alpha-cascade implementation has some limitations, because it does not support complex poles
def _filter(x, alpha=numpy.pi/200000):
	if isinstance(x[0], list) or isinstance(x[0], numpy.ndarray):
		y = numpy.zeros([len(x), len(x[0])])
		for i in range(len(x)):
			y[i][:] = _filter(x[i][:], alpha)
		return y
	if not isinstance(alpha, list):
		alpha = [alpha]
	if len(alpha) == 0:
		return x
	#yinit = numpy.average(x)
	yinit = 0
	a = alpha[-1]
	am = 1 - a
	y = numpy.zeros(len(x))
	y[0] = am * yinit + a * x[0]
	for i in xrange(1, len(x)):
		y[i] = am * y[i-1] + a * x[i]
	return _filter(y, alpha[:-1])

#Generate a zero-mean noise signal with specified standard deviation and number of samples
def _gaussiannoise(sigma, numsamples):
	y = numpy.zeros(numsamples)
	for i in xrange(numsamples):
		y[i] = random.gauss(0, sigma)
	y -= numpy.average(y)
	return y

class DigitalLockin:
	'''
	Digital Lock-in Amplifier class
	This class does NOT do the lock-in in real-time
	Can initialise hardware, run measurements, process data and make some plots

	This class is also capable of running a simulated measurement with Gaussian noise
	In this case it always generates a sine wave of amplitude 1
	The amplitude argument to the constructor is then interpreted as the noise sigma
	It also generates only one output channel regardless of the number of specified channels
	'''

	################################################
	##### Initialization and closing functions #####
	################################################
	
	def __init__(self, gen_dev='PXI5412_12', meas_dev='PXI4462_3', gen_ch='0', gen_meas_ch='ai0', meas_ch='ai1', Fs=MAX_SAMPLE_FREQUENCY, Fsignal=MAX_SAMPLE_FREQUENCY*101./20201, gen_amplitude=1, gen_output_impedance=50, simulated=False):
		'''
		The constructor

		Arguments:
			gen_dev              : string  [Default: 'PXI5412_12']
				The waveform generator device ID/location
				This argument is ignored in simulation mode
			meas_dev             : string  [Default: 'PXI4462_3']
				The signal analyser device ID/location
				This argument is ignored in simulation mode
			gen_ch               : string  [Default: '0']
				The generation channel name
				This argument is ignored in simulation mode
			gen_meas_ch          : string  [Default: 'ai0']
				The channel for measuring the generated input signal
				This argument is ignored in simulation mode
			meas_ch              : string  [Default: 'ai1']
				The channel(s) for measuring output signals of your device under measurement
				Example values:
					'ai1'            (Single channel, value notation)
					['ai1']          (Single channel, list notation)
					['ai1', 'ai2']   (Multiple channels)
				This argument is ignored in simulation mode
			Fs                   : float   [Default: 204800]
				Sample frequency of the signal analyser in Hz
			Fsignal              : float   [Default: 1000]
				Frequency of the generated input sine wave in Hz
			gen_amplitude        : float   [Default: 1]
				Amplitude of the generated signal in V
				When in simulation mode, this is the noise amplitude instead
			gen_output_impedance : float   [Default: 50]
				Output impedance of the waveform generator in Ohm
				This argument is ignored in simulation mode
			simulated            : boolean [Default: False]
				True for simulation mode, False for measurement mode
		'''
		self._simulated = simulated
		self._is_measuring = False
		self.gen_dev_str = gen_dev # Not gonna make a getter and setter for a variable which isn't internally used
		self.meas_dev_str = meas_dev # Not gonna make a getter and setter for a variable which isn't internally used
		self.free_data()
		self.set_flt_time_constant()
		if simulated:
			self._simulation_noise_amplitude = gen_amplitude
			self._hw = None
			self._fs = Fs
			self._f = Fsignal
		elif CAN_MEASURE:
			self._hw = hwi.MeasurementHardwareInterface(gen_dev=gen_dev, meas_dev=meas_dev, gen_ch=gen_ch, meas_ch=meas_ch, Fs=Fs, Fsignal=Fsignal, gen_amplitude=gen_amplitude, gen_output_impedance=gen_output_impedance)
			self._simulation_noise_amplitude = None
			self._fs = self._hw.get_meas_sample_frequency()
			self._f = self._hw.get_gen_signal_frequency()
			self._x1 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
			self._y1 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
			self._x2 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
			self._y2 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
		else:
			raise RuntimeError('Cannot load hardware interface module so cannot initialise in measurement mode')

	def close_hardware(self):
		'''Close hardware tasks and sessions and release their handles'''
		if self._hw is not None:
			self._hw.close()

	def free_rawdata(self):
		'''
		Erase raw data from memory and free the memory it used
		! This cannot be undone !
		'''
		self._rawdata = None

	def free_intermediate_calc_results(self):
		'''Erase intermediate data processing results and free the memory it used'''
		self._sinx = None
		self._cosx = None
		self._sini = None
		self._cosi = None
		self._amplitudes = None
		self._phases = None

	def free_t(self):
		'''Erase the time vector that is stored for convenienced and free the memory it used'''
		self._t = None

	def free_calcdata(self):
		'''Erase the time vector, intermediate data processing results and data processing end results, and free the memory they used'''
		self.free_t()
		self.free_intermediate_calc_results()
		self._normamplitudes = None
		self._normphases = None
		self._gen_meas_amplitude = None

	def free_data(self):
		'''
		Erase all measured data and processing results and free all used memory
		! This cannot be undone !
		'''
		self.free_rawdata()
		self.free_calcdata()
		self._x1 = 0.
		self._y1 = 0.
		self._x2 = 0.
		self._y2 = 0.
		self._phi = 0.

	def close(self):
		'''
		Release all resources used by this object
		This includes used memory as well as hardware tasks and sessions
		! This cannot be undone !
		'''
		self.close_hardware()
		self.free_data()

	############################
	##### Universal setter #####
	############################
	
	def set(self, F=None, A=None, Fs=None):
		'''Universal setter for signal frequency <F>, sample frequency <Fs> and signal amplitude <A>'''
		if F is not None:
			if self._simulated:
				self._f = F
			else:
				try:
					self._hw.set_gen_signal_frequency(F)
				except RuntimeWarning as w:
					logging.warning(str(w))
				self._f = self._hw.get_gen_signal_frequency()
		if A is not None:
			if not self._simulated:
				try:
					self._hw.set_gen_signal_amplitude(A)
				except RuntimeWarning as w:
					logging.warning(str(w))
		if Fs is not None:
			if self._simulated:
				self._fs = Fs
			else:
				self._hw.set_meas_sample_frequency(Fs)
				self._fs = self._hw.get_meas_sample_frequency()
	
	def set_channels(self, meas_ch=None, gen_meas_ch=None, gen_ch=None):
		'''Set measurement channels <meas_ch>, generated signal measurement channel <gen_meas_ch> and/or generator channel <gen_ch>'''
		if meas_ch is not None:
			self._hw.set_meas_channels(meas_ch)
			self._x1 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
			self._y1 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
			self._x2= numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
			self._y2 = numpy.zeros(len(self._hw.get_measurement_channels()) + 1)
		if gen_meas_ch is not None:
			self._hw.set_gen_meas_channel(gen_meas_ch)
		if gen_ch is not None:
			self._hw.set_gen_channel(gen_ch)
	
	###################
	##### Getters #####
	###################
	
	def get_f(self):
		'''Get generated signal frequency'''
		return self._f

	def get_fs(self):
		'''Get sample frequency'''
		return self._fs

	def get_gen_amplitude(self):
		'''Get generated signal amplitude'''
		if self._simulated:
			return 1
		else:
			return self._hw.get_gen_signal_amplitude()
	
	def get_num_meas_ch(self):
		'''Get number of measurement channels (excluding the one for the generated signal)'''
		return len(self._hw.get_measurement_channels())

	#################################
	##### Measurement functions #####
	#################################
	
	def run_measurement(self, periods):
		'''
		In measurement mode:
			Start the signal generators,
			run a measurement for <periods> signal periods and save the raw data,
			then stop the signal generators again
		In simulation mode:
			Save a perfect sine as input signal
			Add gaussian noise to that sine and save it as output signal
		'''
		if self._simulated:
			num_samples = int(round(periods * self._fs / self._f))
			t = numpy.array(range(num_samples)) * 1. / self._fs
			sig_in = numpy.sin(2 * numpy.pi * self._f * t)
			sig_out = sig_in + _gaussiannoise(self._simulation_noise_amplitude, num_samples)
			self._rawdata = numpy.array([sig_in, sig_out])
		else:
			self._hw.start_generation()
			self._rawdata = self._hw.measure_periods(periods)
			self._hw.stop_generation()

	def start_measurement(self, bufsize=409600):
		'''
		Start the waveform generators and inform the signal analysers to collect samples
		Do not retrieve any measured samples to the pc yet
		In simulation mode this sets a boolean to allow acquiring samples
		'''
		if self._is_measuring:
			raise RuntimeWarning('Already measuring')
		else:
			self._is_measuring = True
			if not self._simulated:
				self._hw.start_generation()
				self._hw.start_measurement(bufsize=bufsize)

	def retrieve_samples(self, samples, append=False):
		'''Retrieve <samples> samples if the device is currently measuring'''
		if self._is_measuring:
			if self._simulated:
				t = numpy.array(range(samples)) / float(self._fs)
				sig_in = numpy.sin(2 * numpy.pi * self._f * t)
				sig_out = sig_in + _gaussiannoise(self._simulation_noise_amplitude, samples)
				rawdata = numpy.array([sig_in, sig_out])
			else:
				rawdata = self._hw.retrieve_samples(samples)
			if append:
				self._rawdata = numpy.append(self._rawdata, rawdata, 1)
			else:
				self._rawdata = rawdata
		else:
			raise RuntimeWarning('Tried to retrieve %d samples from non-measuring device', samples)

	def retrieve_periods(self, periods, append=False):
		'''Retrieve <periods> signal periods worth of samples if the device is currently measuring'''
		self.retrieve_samples(int(round(float(self._fs) / self._f * periods)), append)

	def retrieve_seconds(self, seconds, append=False):
		'''
		Retrieve samples for <seconds> seconds if the device is currently measuring
		Automatically rounds to an integer number of signal periods so you don't have to worry about artefacts caused by a non-integer amount of periods
		'''
		self.retrieve_periods(round(self._f * seconds), append)
		return round(round(self._f * seconds) * self._fs / self._f) / self._fs

	def stop_measurement(self):
		'''Stop the collection of samples and the waveform generators'''
		if self._is_measuring:
			if not self._simulated:
				self._hw.end_measurement()
				self._hw.stop_generation()
			self._is_measuring = False
		else:
			raise RuntimeWarning('Tried stopping device from measuring but it already wasn\'t')

	def is_measuring(self):
		return self._is_measuring
	
	def num_measured_samples_in_instrument_buffer(self):
		'''
		Find out how many samples are left in the instrument's sample buffer.
		Returns None on failure and 0 during simulation or when not measuring.
		'''
		if self._is_measuring and not self._simulated:
			return self._hw.measured_samples_in_instrument_buffer()
		else:
			return 0

	##############################################################
	##### Measure and filter function for continuous lock-in #####
	##############################################################
	
	def continuous_retrieve_and_filter(self):
		'''
		Retrieve all samples in instrument buffer if the device is currently measuring
		Apply synchronous detection (i.e. multiply with a sine and cosine)
		Filter the result with a second-order filter (two cascaded normalising alpha filters)
		Higher-order filters may be implementable in the future, but it is not
		trivial because the optimized numpy implementation requires that the samples
		be processed in bulk and only the last sample of the filter output is calculated
		
		Each normalising alpha filter works like:
		
		         /-----------\      /---\
		-------->| 1 - alpha |----->| + |---------------+----->
		         \-----------/      \---/               |
		                              ^                 |
		                              |    /-------\    |
		                              \----| alpha |<---/
		                                   \-------/
		'''
		if self._is_measuring and not self._simulated:
			# Calculate -ln(alpha) and -ln(1-alpha)
			mlnalpha = - numpy.log(1 - 1. / self._flt_tau / self._fs)
			mlnialpha = numpy.log(self._flt_tau * self._fs)
			
			# Acquire samples
			rawdata = self._hw.retrieve_samples(-1, .1, True)
			channels = rawdata.shape[0]
			samples = rawdata.shape[1]
			
			# Calculate phases for synchronous detection
			multfac = 2 * numpy.pi * self._f / self._fs
			phi = numpy.arange(self._phi, self._phi + multfac * (samples-0.5), multfac)
			# The following phase expression may drift over time due to rounding errors
			# But that'll only affect the detected common mode phase which is arbitrary and rejected anyway
			self._phi = numpy.mod(self._phi + 2 * numpy.pi * self._f / self._fs * samples, 2 * numpy.pi)
			
			# Calculate multiplication factors for filters
			# Sample to output of first filter
			flt1weight = numpy.exp(numpy.arange(-(samples-1) * mlnalpha - mlnialpha, mlnalpha/2 - mlnialpha, mlnalpha))
			# initvalmulfac1*Sample to output of second filter
			flt2weightr = numpy.repeat((numpy.exp(-mlnialpha) * numpy.arange(samples, 0.5, -1)).reshape([1,samples]), channels, axis=0)
			# Initial value to output of the same alpha filter
			initvalmulfac1 = numpy.exp(- mlnalpha * samples)
			# Initial value to output of next alpha filter
			initvalmulfac2 = samples * numpy.exp(- mlnalpha * samples - mlnialpha)
			
			# Perform synchronous detection and filtering
			f1sin = rawdata * numpy.repeat((numpy.sin(phi) * flt1weight).reshape([1,samples]), channels, axis=0)
			f1cos = rawdata * numpy.repeat((numpy.cos(phi) * flt1weight).reshape([1,samples]), channels, axis=0)
			f2sin = f1sin * flt2weightr
			f2cos = f1cos * flt2weightr
			self._x2 = initvalmulfac1 * self._x2 + initvalmulfac2 * self._x1 + f2sin.sum(axis=1)
			self._y2 = initvalmulfac1 * self._y2 + initvalmulfac2 * self._y1 + f2cos.sum(axis=1)
			self._x1 = initvalmulfac1 * self._x1 + f1sin.sum(axis=1)
			self._y1 = initvalmulfac1 * self._y1 + f1cos.sum(axis=1)
		elif self._is_measuring:
			raise RuntimeError('Continuous running mode not supported for simulation')
		else:
			raise RuntimeWarning('Tried to retrieve samples from non-measuring device')
	
	def continuous_get_r_phi(self):
		#r = numpy.sqrt(numpy.append(self._x1, self._x2 / (2*self._flt_tau * self._fs)**2)**2 + numpy.append(self._y1, self._y2 / (2*self._flt_tau * self._fs)**2)**2)
		r = numpy.sqrt(numpy.append(self._x1, self._x2)**2 + numpy.append(self._y1, self._y2)**2)
		#r = numpy.sqrt(self._x2**2 + self._y2**2)
		#r[1:] /= r[0]
		r[1:int(len(r)/2)] /= r[0]
		r[int(len(r)/2)+1:] /= r[int(len(r)/2)]
		#r[0] /= self._flt_tau * self._fs / 2
		r[0::int(len(r)/2)] *= 2
		#phi = numpy.arctan2(self._y2, self._x2)
		phi = numpy.arctan2(numpy.append(self._y1, self._y2), numpy.append(self._x1, self._x2))
		phi = numpy.mod(phi[1:] - phi[0] + numpy.pi, 2 * numpy.pi) - numpy.pi
		phi[int(len(r)/2):] -= phi[int(len(r)/2) - 1]
		return (r, phi)
	
	def set_flt_alpha(self, alpha=0.9):
		self._flt_tau = 1. / self._fs / (1 -alpha)
	
	def set_flt_time_constant(self, tau=0.1):
		self._flt_tau = tau
	
	#####################################
	##### Data processing functions #####
	#####################################
	
	def process_data(self):
		'''Perform lock-in analysis and save results in memory'''
		try:
			if self._rawdata is None:
				raise RuntimeError()
		except Exception:
			logging.error('No raw data found')
			return
		phi = 2*numpy.pi*self._f * numpy.array(range(len(self._rawdata[0]))) / float(self._fs)
		sini = (numpy.sin(phi) * self._rawdata).sum(axis=1)
		cosi = (numpy.cos(phi) * self._rawdata).sum(axis=1)
		amplitudes = numpy.sqrt(sini**2 + cosi**2)
		phases = numpy.arctan2(cosi, sini)
		self._normamplitudes = amplitudes[1:] / amplitudes[0]
		normphases = phases[1:] - phases[0]
		self._normphases = numpy.mod(normphases + numpy.pi, 2 * numpy.pi) - numpy.pi
		self._gen_meas_amplitude = 2 * amplitudes[0] / float(len(phi))
		return (self._gen_meas_amplitude, self._normamplitudes, self._normphases)

	def process_data_moreinfo(self, fltord=0, RC=1/numpy.pi):
		'''
		Perform lock-in analysis and save results in memory
		Saves lots of intermediate calculated values
		Also has the option to apply a cascade of <fltord> identical RC-filters with RC-time <RC> before integration
		'''
		try:
			if self._rawdata is None:
				raise RuntimeError()
		except Exception:
			logging.error('No raw data found')
			return
		self._t = numpy.array(range(len(self._rawdata[0])))*1./self._fs
		tsin = numpy.sin(2*numpy.pi*self._f*self._t)
		tcos = numpy.cos(2*numpy.pi*self._f*self._t)
		self._sinx = tsin * self._rawdata
		self._cosx = tcos * self._rawdata
		if fltord == 0:
			self._sinf = self._sinx
			self._cosf = self._cosx
		else:
			alpha = 1 / (RC * self._fs)
			self._sinf = _filter(self._sinx, [alpha]*fltord)
			self._cosf = _filter(self._cosx, [alpha]*fltord)
		self._sini = _integrate(self._sinf)
		self._cosi = _integrate(self._cosf)
		self._amplitudes = numpy.sqrt(self._sini**2 + self._cosi**2)
		self._phases = numpy.arctan2(self._cosi, self._sini)
		self._normamplitudes_all = self._amplitudes[1:][:] / self._amplitudes[0][:]
		self._normphases_all = numpy.mod(self._phases[1:][:] - self._phases[0][:] + numpy.pi, 2*numpy.pi) - numpy.pi
		sampsperperiod = self._fs / self._f
		self._t_smp = self._t[sampsperperiod-1::sampsperperiod]
		self._normamplitudes_smp = [[]]*len(self._normamplitudes_all)
		self._normphases_smp = [[]]*len(self._normphases_all)
		self._normamplitudes = numpy.zeros(len(self._normamplitudes_all))
		self._normphases = numpy.zeros(len(self._normphases_all))
		for i in range(len(self._normamplitudes)):
			self._normamplitudes_smp[i] = self._normamplitudes_all[i][sampsperperiod-1::sampsperperiod]
			self._normphases_smp[i] = self._normphases_all[i][sampsperperiod-1::sampsperperiod]
			self._normamplitudes[i] = self._normamplitudes_all[i][-1]
			self._normphases[i] = self._normphases_all[i][-1]
		self._normamplitudes_smp = numpy.array(self._normamplitudes_smp)
		self._normphases_smp = numpy.array(self._normphases_smp)
		self._gen_meas_amplitude = 2 * self._amplitudes[0][-1] / float(len(self._t))
		return (self._gen_meas_amplitude, self._normamplitudes, self._normphases)

	#############################
	##### Display functions #####
	#############################
	
	def plot_results(self):
		'''
		Plot how the results of the lock-in measurement evolve as measurement extends across more signal periods
		Very useful when testing the influence of various lock-in parameters, in particular the number of periods during which is measured
		Less useful when doing a lot of lock-in measurements and only comparing end results
		Only works after process_data_moreinfo()
		'''
		try:
			if self._normamplitudes_all is None or self._normphases_all is None:
				raise RuntimeError()
		except Exception:
			logging.error('No intermediate processing data found')
		if self._t is None:
			t = numpy.array(range(len(self._rawdata[0])))*1./self._fs
		else:
			t = self._t
		f, sf = plt.subplots(3,1)
		try:
			#print('%dx%d' % (len(self._rawdata[1:][:]), len(self._rawdata[0][:])))
			rawout = self._rawdata[1:][:]
			#rawout.reverse()
			sf[0].plot(t, numpy.array(rawout).T)
			if len(self._normamplitudes_smp.T) > 100:
				sf[1].semilogy(self._t_smp, self._normamplitudes_smp.T)
				sf[2].plot(self._t_smp, self._normphases_smp.T)
			else:
				sf[1].semilogy(self._t_smp, self._normamplitudes_smp.T, 'o', t, self._normamplitudes_all.T)
				sf[2].plot(self._t_smp, self._normphases_smp.T, 'o', t, self._normphases_all.T)
				if self._normamplitudes_all.max() > 10 * self._normamplitudes_smp.max():
					sf[1].set_ylim([0, 2*max(max(self._normamplitudes_smp))])
			plt.show()
			time.sleep(0.1)
		except Exception as e:
			print('plotting error:\n%s' % str(e))

	def printmainresults(self, compactfmt=True):
		'''Print a terminal message with only the end value of the amplitude and phase of the integrated signal'''
		try:
			if self._normamplitudes is None or self._normphases is None or self._gen_meas_amplitude is None:
				raise RuntimeError()
			else:
				if compactfmt:
					strres = 'Ref({:.2e})'.format(self._gen_meas_amplitude)
					for i in range(len(self._normamplitudes)):
						strres += ',  ch{:d}({:.3e} {:.0f}deg)'.format(i+1, self._normamplitudes[i], 180/numpy.pi * self._normphases[i])
					logging.info(strres)
				else:
					logging.info('Reference amplitude: {:f} V'.format(self._gen_meas_amplitude))
					for i in range(len(self._normamplitudes)):
						#logging.info('channel {:d}: {:e} e^i {:f}'.format(i, self._amplitudes[i,-1], self._phases[i,-1]))
						logging.info('channel {:d}: {:e} e^i {:f}'.format(i+1, self._normamplitudes[i], self._normphases[i]))
		except Exception:
			logging.exception('No processed data found')

	def get_main_results(self):
		'''Getter for the end value of the reference signal amplitude and the normalised amplitudes and phases of other integrated signals'''
		try:
			return (self._gen_meas_amplitude, self._normamplitudes, self._normphases)
		except Exception:
			raise RuntimeWarning('No results!')

#####################################################################################
##### Meta functions (not part of the class, just for running some quick tests) #####
#####################################################################################

def runonce(periods=1, freq=10, fs=1000, ch='ai1'):
	dl = DigitalLockin(Fsignal=freq, Fs=fs, meas_ch=ch)
	bm_t1 = time.clock()
	dl.run_measurement(periods)
	dl.close_hardware()
	bm_t2 = time.clock()
	#dl.process_data_moreinfo()
	dl.process_data()
	bm_t3 = time.clock()
	dl.printmainresults()
	#sys.stdout.flush()
	#dl.plot_results()
	dl.close()
	print 'Measurement took %f seconds, analysis took %f' % (bm_t2-bm_t1, bm_t3-bm_t2)
	return dl._normamplitudes

# Compare outcome for different filter orders
def check_filters(periods=1, freq=10, fs=1000, ch='ai1', RC=1/numpy.pi, a=0.1):
	dl = DigitalLockin(Fsignal=freq, Fs=fs, meas_ch=ch, simulated=True, gen_amplitude=a)
	dl.run_measurement(periods)
	dl.close_hardware()
	numords = 5
	ordsin = []
	ordcos = []
	for i in range(numords):
		dl.process_data(RC=RC, fltord=i)
		print('Filter order %d' % i)
		dl.printmainresults()
		ordsin.append(dl._sinf[1])
		ordcos.append(dl._cosf[1])
	t = dl._t
	inp_raw = dl._rawdata[0][:]
	raw = dl._rawdata[1][:]
	dl.close()
	f, subf = plt.subplots(5,1,sharex=True)
	subf[0].plot(t, raw, 'b', t, inp_raw, 'r')
	subf[1].plot(t,numpy.array(ordsin).T)
	subf[2].plot(t,numpy.array(ordcos).T)
	spp = fs / freq
	raw_nodc = raw - numpy.average(raw)
	raw_oneperiodapart = - raw_nodc[spp/2:] * raw_nodc[:-spp/2]
	subf[3].plot(t, raw**2, 'b', t, inp_raw**2, 'r')
	subf[4].plot(t[spp/2:], raw_oneperiodapart, 'b', t, inp_raw**2, 'r')
	print 'Input autocorrelation: %e' % numpy.sqrt(2 * numpy.average(inp_raw**2))
	print 'Autocorrelation: %e' % numpy.sqrt(2 * numpy.average(raw**2))
	print 'Half-period-shifted autocorrelation: %e %e %e' % (numpy.sqrt(2 * abs(numpy.average(raw_oneperiodapart))), numpy.sqrt(2 * abs(numpy.average(_filter(raw_oneperiodapart, 1/RC/fs)))), numpy.sqrt(2 * abs(numpy.average(_filter(raw_oneperiodapart, [1/RC/fs]*2)))))
	sys.stdout.flush()
	plt.show()

def filterchk2():
	dt = numpy.pi / 30
	t = numpy.arange(0,6*numpy.pi,numpy.pi/30)
	s = numpy.sin(t)
	c = numpy.cos(t)
	sf = _filter(s, numpy.pi*dt)
	cf = _filter(c, numpy.pi*dt)
	f, subf = plt.subplots(2,1)
	print '%d, %d,%d, %d,%d, %f,%f, %f,%f' % (len(t), len(s), len(c), len(sf), len(cf), numpy.average(s), numpy.average(c), numpy.average(sf), numpy.average(cf))
	subf[0].plot(t,s,'r',t,sf,'b')
	subf[1].plot(t,c,'r',t,cf,'b')
	plt.show()
