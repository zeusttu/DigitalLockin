'''
nidaq_dllsupport.py, python wrapper for NIDAQ DLL

Copyright:
	Zeust the Unoobian <2noob2banoob@gmail.com>, 2014
	Reinier Heeres <reinier@heeres.eu>, 2008-2010

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

This file is a derivative work of nidaq.py which is part of QTLab
<https://github.com/heeres/qtlab>, distributed under GPL v2 and found at:
https://github.com/heeres/qtlab/blob/master/source/lib/dll_support/nidaq.py
The used version of nidaq.py stems from Oct 22, 2010

Changes made by Zeust the Unoobian:
  * Added additional debugging information to CHK calls so errors can
    be traced to a specific part of the code.
  * Made CHK also report warnings
  * create_counter_task() and other functions I do not use have not
    yet been modified with additional debugging info
  * Added synchronisation-related functions
  * Modified read() to use a reference clock by default
  * Split functionality of read() into several functions and made the read()
    function call those sequentially
  * Removed one-sample option from read() because that was not working
    (at least with the hardware I have tested with) and not needed for
	the Lock-in functionality either
  * Additional changes that I forgot to log here (I'm sorry)
'''

import ctypes
import types
import numpy
import logging
import time

nidaq = ctypes.windll.nicaiu

int32 = ctypes.c_long
uInt32 = ctypes.c_ulong
uInt64 = ctypes.c_ulonglong
float64 = ctypes.c_double
TaskHandle = uInt32

DAQmx_Val_Cfg_Default = int32(-1)

DAQmx_Val_RSE               = 10083
DAQmx_Val_NRSE              = 10078
DAQmx_Val_Diff              = 10106
DAQmx_Val_PseudoDiff        = 12529

_config_map = {
	'DEFAULT': DAQmx_Val_Cfg_Default,
	'RSE': DAQmx_Val_RSE,
	'NRSE': DAQmx_Val_NRSE,
	'DIFF': DAQmx_Val_Diff,
	'PSEUDODIFF': DAQmx_Val_PseudoDiff,
}

DAQmx_Val_Volts             = 10348
DAQmx_Val_Rising            = 10280
DAQmx_Val_FiniteSamps       = 10178
DAQmx_Val_ContSamps         = 10123
DAQmx_Val_GroupByChannel    = 0
DAQmx_Val_GroupByScanNumber = 1
DAQmx_Val_ChanPerLine       = 0
DAQmx_Val_ChanForAllLines   = 1

DAQmx_Val_CountUp           = 10128
DAQmx_Val_CountDown         = 10124
DAQmx_Val_ExtControlled     = 10326


DAQmx_Val_AIConvertClock=12484
DAQmx_Val_10MHzRefClock=12536
DAQmx_Val_20MHzTimebaseClock=12486
DAQmx_Val_SampleClock=12487
DAQmx_Val_AdvanceTrigger=12488
DAQmx_Val_ReferenceTrigger=12490
DAQmx_Val_StartTrigger=12491
DAQmx_Val_AdvCmpltEvent=12492
DAQmx_Val_AIHoldCmpltEvent=12493
DAQmx_Val_CounterOutputEvent=12494
DAQmx_Val_WDTExpiredEvent=12512
DAQmx_Val_SampleCompleteEvent=12530
DAQmx_Val_ChangeDetectionEvent=12511

def CHK(err, function=''):
	'''Error checking routine'''

	if err == 0:
		return True

	if function is None:
		function = ''

	buf_size = 1000
	buf = ctypes.create_string_buffer('\000' * buf_size)
	nidaq.DAQmxGetErrorString(err, ctypes.byref(buf), buf_size)
	errstr = 'Nidaq call ''{:s}'' failed with {:s} {:d}: {:s}'.format(function, '{:s}', err, repr(buf.value))
	if err < 0:
		logging.error(errstr.format('error'))
		return False
	else:
		logging.warning(errstr.format('warning'))
		return True

def buf_to_list(buf):
	name = ''
	namelist = []
	for ch in buf:
		if ch in '\000 \t\n':
			name = name.rstrip(',')
			if len(name) > 0:
				namelist.append(name)
				name = ''
			if ch == '\000':
				break
		else:
			name += ch

	return namelist

def get_device_names():
	'''Return a list of available NIDAQ devices.'''

	bufsize = 1024
	buf = ctypes.create_string_buffer('\000' * bufsize)
	nidaq.DAQmxGetSysDevNames(ctypes.byref(buf), bufsize)
	return buf_to_list(buf)

def reset_device(dev):
	'''Reset device "dev"'''
	return CHK(nidaq.DAQmxResetDevice(dev), 'reset_device()')

def get_physical_input_channels(dev):
	'''Return a list of physical input channels on a device.'''

	bufsize = 1024
	buf = ctypes.create_string_buffer('\000' * bufsize)
	CHK(nidaq.DAQmxGetDevAIPhysicalChans(dev, ctypes.byref(buf), bufsize), 'get_physical_input_channels')
	return buf_to_list(buf)

def get_physical_output_channels(dev):
	'''Return a list of physical output channels on a device.'''

	bufsize = 1024
	buf = ctypes.create_string_buffer('\000' * bufsize)
	nidaq.DAQmxGetDevAOPhysicalChans(dev, ctypes.byref(buf), bufsize)
	return buf_to_list(buf)

def get_digital_output_channels(dev):
	'''Return a list of physical output channels on a device.'''

	bufsize = 1024
	buf = ctypes.create_string_buffer('\000' * bufsize)
	nidaq.DAQmxGetDevDOLines(dev, ctypes.byref(buf), bufsize)
	return buf_to_list(buf)

def get_physical_counter_channels(dev):
	'''Return a list of physical counter channels on a device.'''

	bufsize = 1024
	buf = ctypes.create_string_buffer('\000' * bufsize)
	nidaq.DAQmxGetDevCIPhysicalChans(dev, ctypes.byref(buf), bufsize)
	return buf_to_list(buf)

def kill_task(taskHandle):
	'''
	Stops and clears a NI-DAQmx task.
	'''
	try:
		CHK(nidaq.DAQmxStopTask(taskHandle), 'kill_task().stop')
	except Exception:
		#logging.error('%s' % str(e))
		logging.exception('kill_task: could not stop task:')
	try:
		CHK(nidaq.DAQmxClearTask(taskHandle), 'kill_task().clear')
	except Exception:
		#logging.error('%s' % str(e))
		logging.exception('kill_task: could not clear task:')

def read(devchan, samples=1, freq=10000.0, minv=-10.0, maxv=10.0,
			timeout=10.0, config=DAQmx_Val_Cfg_Default, clksrc='PXI_Clk10', clkfreq=10e6):
	'''
	Read up to max_samples from a channel. Seems to have trouble reading
	1 sample!

	Input:
		devchan (string): device/channel specifier, such as Dev1/ai0
		samples (int): the number of samples to read
		freq (float): the sampling frequency
		minv (float): the minimum voltage
		maxv (float): the maximum voltage
		timeout (float): the time in seconds to wait for completion
		config (string or int): the configuration of the channel

	Output:
		A numpy.array with the data on success, None on error
	'''

	numchannels = len(devchan.split(','))
	
	taskHandle = read_init(devchan, minv, maxv, config)
	if taskHandle is None:
		logging.error('read_init() returned no task handle')
		return None
	if clksrc is not None and clkfreq > 0:
		if set_refclk(taskHandle, clksrc, clkfreq) == False:
			logging.error('Failed to configure reference clock')
	else:
		logging.warning('No reference clock specified')
	if read_start(taskHandle, samples, freq) == False:
		logging.error('read_start() failed')
	else:
		return read_finish(taskHandle, samples, timeout, numchannels)

def read_init(devchan, minv=-10.0, maxv=10.0, config=DAQmx_Val_Cfg_Default):
	'''
	Initiate a task for reading samples. Also opens a voltage measurement channel.

	Input:
		devchan (string): device/channel specifier, such as Dev1/ai0
		minv (float): the minimum voltage
		maxv (float): the maximum voltage
		config (string or int): the configuration of the channel

	Output:
		A task handle (int) on success, None on error
	'''

	#convert config
	if type(config) is types.StringType:
		if config in _config_map:
			config = _config_map[config]
		else:
			logging.error('read_init(): invalid config string %s', config)
			return None
	if type(config) is not types.IntType:
		logging.error('read_init(): non-integer config %s of type %s', config, type(config))
		return None

	taskHandle = TaskHandle(0)
	try:
		CHK(nidaq.DAQmxCreateTask("", ctypes.byref(taskHandle)), 'read().DAQmxCreateTask()')
		CHK(nidaq.DAQmxCreateAIVoltageChan(taskHandle, devchan, "",
				config, float64(minv), float64(maxv), DAQmx_Val_Volts, None),
			'read().DAQmxCreateAIVoltageChan(taskHandle=%d, phys_chan=%s, name_to_assign_to_channel="", config=%d, min=%f, max=%f, units=%d (Volts), customScaleName=None)' % (taskHandle.value,devchan
,config,minv,maxv,DAQmx_Val_Volts))
		return taskHandle
	except Exception:
		#logging.error('%s', str(e))
		logging.exception('read_init: could not start task:')
		if taskHandle.value != 0:
			kill_task(taskHandle)
		return None

def read_start(taskHandle, samples=1, freq=10000.0):
	'''
	Start measuring samples from a channel. Do not acquire measured samples yet.

	Input:
		taskHandle (int): Handle of task generated by read_init()
		samples (int): the number of samples to read
		               0 or negative values indicate continuous sampling
					   in that case the value should be minus the desired buffer size
		freq (float): the sampling frequency

	Output:
		True on success, False on failure
	'''
	
	try:
		if samples > 0:
			CHK(nidaq.DAQmxCfgSampClkTiming(taskHandle, "", float64(freq),
				DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
				uInt64(samples)), 'read().DAQmxCfgSampClkTiming()');
		else:
			CHK(nidaq.DAQmxCfgSampClkTiming(taskHandle, "", float64(freq),
				DAQmx_Val_Rising, DAQmx_Val_ContSamps,
				uInt64(abs(samples))), 'read().DAQmxCfgSampClkTiming()');
		CHK(nidaq.DAQmxStartTask(taskHandle), 'read().DAQmxStartTask()')
		return True
	except Exception as e:
		logging.error('{:s}\nIf you have no idea what may cause this error, you may be requesting more samples than the system supports.'.format(str(e)))
		kill_task(taskHandle)
		return False

def read_get_some_samples(taskHandle, samples=1, timeout=10.0, numchannels=1):
	'''
	Read up to max_samples measured samples from a channel.

	Input:
		taskHandle (int): Handle of task generated by read_init()
		samples (int): the number of samples to read
		timeout (float): the time in seconds to wait for completion

	Output:
		A numpy.array with the data on success, None on error
	'''
	#Allocate data array and initiate variable with number of read samples
	if samples < 0:
		sz = numchannels*100000
	else:
		sz = numchannels*samples
	data = numpy.zeros(sz, dtype=numpy.float64)
	read = int32(0)

	try:
		CHK(nidaq.DAQmxReadAnalogF64(taskHandle, samples, float64(timeout),
				DAQmx_Val_GroupByChannel, data.ctypes.data,
				sz, ctypes.byref(read), None),
			'read().DAQmxReadAnalogF64(task={:d}, samples={:d}, timeout={:.3f}, grouping={:d}, data=pointer, size={:d}, num_read=pointer, reserved=None)'.format(
				taskHandle.value, samples, timeout, DAQmx_Val_GroupByChannel, sz))
		if read.value > 50000:
			raise RuntimeError('Realtime operation isnt working')
	except Exception:
		#logging.error('%s', str(e))
		logging.exception('Failed to get samples:')
		kill_task(taskHandle)

	if read.value > 0:
		return data[:numchannels*read.value]
	else:
		return None

def read_finish(taskHandle, samples=1, timeout=10.0, numchannels=1):
	'''
	Read up to max_samples measured samples from a channel.
	Then stop data acquisition and kill the task pointed to by taskHandle.

	Input:
		taskHandle (int): Handle of task generated by read_init()
		samples (int): the number of samples to read
		timeout (float): the time in seconds to wait for completion

	Output:
		A numpy.array with the data on success, None on error
	'''
	
	#Allocate data array and initiate variable with number of read samples
	data = numpy.zeros(numchannels*samples, dtype=numpy.float64)
	read = int32()

	try:
		if samples > 0:
			CHK(nidaq.DAQmxReadAnalogF64(taskHandle, samples, float64(timeout),
				DAQmx_Val_GroupByChannel, data.ctypes.data,
				numchannels*samples, ctypes.byref(read), None), 'read().DAQmxReadAnalogF64()')
	except Exception as e:
		#logging.error('%s', str(e))
		logging.exception('read_finish: could not read last samples:')
	finally:
		kill_task(taskHandle)

	if read > 0:
		return data[:numchannels*read.value]
	else:
		return None

def write(devchan, data, freq=10000.0, minv=-10.0, maxv=10.0,
				timeout=10.0):
	'''
	Write values to channel

	Input:
		devchan (string): device/channel specifier, such as Dev1/ao0
		data (int/float/numpy.array): data to write
		freq (float): the the minimum voltage
		maxv (float): the maximum voltage
		timeout (float): the time in seconds to wait for completion

	Output:
		Number of values written
	'''

	if type(data) in (types.IntType, types.FloatType):
		data = numpy.array([data], dtype=numpy.float64)
	elif isinstance(data, numpy.ndarray):
		if data.dtype is not numpy.float64:
			data = numpy.array(data, dtype=numpy.float64)
	elif len(data) > 0:
		data = numpy.array(data, dtype=numpy.float64)
	samples = len(data)

	taskHandle = TaskHandle(0)
	written = int32()
	try:
		CHK(nidaq.DAQmxCreateTask("", ctypes.byref(taskHandle)), 'write().DAQmxCreateTask()')
		CHK(nidaq.DAQmxCreateAOVoltageChan(taskHandle, devchan, "",
			float64(minv), float64(maxv), DAQmx_Val_Volts, None), 'write().DAQmxCreateAOVoltageChan()')

		if len(data) == 1:
			CHK(nidaq.DAQmxWriteAnalogScalarF64(taskHandle, 1, float64(timeout),
				float64(data[0]), None), 'write().DAQmxWriteAnalogScalarF64()')
			written = int32(1)
		else:
			CHK(nidaq.DAQmxCfgSampClkTiming(taskHandle, "", float64(freq),
				DAQmx_Val_Rising, DAQmx_Val_FiniteSamps, uInt64(samples)), 'write().DAQmxCfgSampClkTiming()')
			CHK(nidaq.DAQmxWriteAnalogF64(taskHandle, samples, 0, float64(timeout),
				DAQmx_Val_GroupByChannel, data.ctypes.data,
				ctypes.byref(written), None), 'write().DAQmxWriteAnalogF64()')
			CHK(nidaq.DAQmxStartTask(taskHandle), 'write().DAQmxStartTask()')
	except Exception as e:
		logging.error('NI DAQ call write() failed internally (correct channel configuration selected?): %s', str(e))
	finally:
		if taskHandle.value != 0:
			nidaq.DAQmxStopTask(taskHandle)
			nidaq.DAQmxClearTask(taskHandle)

	return written.value

def read_counter(devchan="/Dev1/ctr0", samples=1, freq=1.0, timeout=1.0, src=""):
	'''
	Read counter 'devchan'.
	Specify source pin with 'src'.
	'''

	taskHandle = TaskHandle(0)
	try:
		CHK(nidaq.DAQmxCreateTask("", ctypes.byref(taskHandle)), 'read_counter().DAQmxCreateTask()')
		initial_count = int32(0)
		CHK(nidaq.DAQmxCreateCICountEdgesChan(taskHandle, devchan, "",
				DAQmx_Val_Rising, initial_count, DAQmx_Val_CountUp), 'read_counter().DAQmxCreateCICountEdgesChan()')
		if src is not None and src != "":
			CHK(nidaq.DAQmxSetCICountEdgesTerm(taskHandle, devchan, src), 'read_counter().DAQmxSetCICountEdgesTerm()')

		nread = int32()
		data = numpy.zeros(samples, dtype=numpy.float64)
		if samples > 1:
			CHK(nidaq.DAQmxCfgSampClkTiming(taskHandle, "", float64(freq),
				DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
				uInt64(samples)), 'read_counter().DAQmxCfgSampClkTiming()');
			CHK(nidaq.DAQmxStartTask(taskHandle), 'read_counter().DAQmxStartTask(1)')
			CHK(nidaq.DAQmxReadAnalogF64(taskHandle, int32(samples), float64(timeout),
			   DAQmx_Val_GroupByChannel, data.ctypes.data,
			   samples, ctypes.byref(read), None), 'read_counter().DAQmxReadAnalogF64()')
		else:
			CHK(nidaq.DAQmxStartTask(taskHandle), 'read_counter().DAQmxStartTask(2)')
			time.sleep(1.0 / freq)
			nread = int32(0)
			CHK(nidaq.DAQmxReadCounterF64(taskHandle, int32(samples), float64(timeout),
				data.ctypes.data, int32(samples), ctypes.byref(nread), None), 'read_counter().DAQmxReadCounterF64()')
			nread = int32(1)

	except Exception as e:
		logging.error('NI DAQ call read_counter() failed internally: %s', str(e))

	finally:
		if taskHandle.value != 0:
			nidaq.DAQmxStopTask(taskHandle)
			nidaq.DAQmxClearTask(taskHandle)

	if nread.value == 1:
		return int(data[0])
	else:
		return data

def create_counter_task(devchan, samples=1, freq=1, timeout=1, src=""):
	taskHandle = TaskHandle(0)

	try:
		CHK(nidaq.DAQmxCreateTask("", ctypes.byref(taskHandle)))
		initial_count = int32(0)
		CHK(nidaq.DAQmxCreateCICountEdgesChan(taskHandle, devchan, "",
				DAQmx_Val_Rising, initial_count, DAQmx_Val_CountUp))
		if src is not None and src != "":
			CHK(nidaq.DAQmxSetCICountEdgesTerm(taskHandle, devchan, src))

		if samples > 1:
			CHK(nidaq.DAQmxCfgSampClkTiming(taskHandle, "", float64(freq),
				DAQmx_Val_Rising, DAQmx_Val_FiniteSamps,
				uInt64(samples)));

	except Exception as e:
		logging.error('NI DAQ call failed: %s', str(e))
		if taskHandle.value != 0:
			nidaq.DAQmxStopTask(taskHandle)
			nidaq.DAQmxClearTask(taskHandle)

	return taskHandle

def read_counters(devchans=["/Dev1/ctr0","/Dev1/ctr1"], samples=1, freq=1.0, timeout=1.0, src=None):
	tasks = []
	devsrc = None
	ret = []
	for i, dev in enumerate(devchans):
		if src is not None:
			devsrc = src[i]
		result = create_counter_task(dev, samples, freq, timeout, devsrc)
		if result != -1:
			tasks.append(result)

	try:
		for task in tasks:
			CHK(nidaq.DAQmxStartTask(task))

		time.sleep(float(samples) / freq)

		for task in tasks:
			data = numpy.zeros(samples, dtype=numpy.float64)
			if samples > 1:
				CHK(nidaq.DAQmxReadAnalogF64(task, int32(samples), float64(timeout),
						DAQmx_Val_GroupByChannel, data.ctypes.data,
						samples, ctypes.byref(read), None))
				ret.append(data)
			else:
				nread = int32(0)
				CHK(nidaq.DAQmxReadCounterF64(task, int32(samples), float64(timeout),
					data.ctypes.data, int32(samples), ctypes.byref(nread), None))
				nread = int32(1)
				ret.append(data[0])

	except Exception as e:
		logging.error('NI DAQ call failed: %s', str(e))

	finally:
		for task in tasks:
			if task.value != 0:
				nidaq.DAQmxStopTask(task)
				nidaq.DAQmxClearTask(task)

	return ret

def write_dig_port8(channel, val, timeout=1.0):
	'''
	Set digital output of channels.
	The value is sent to the specified channels, LSB to MSB.
	'''

	taskHandle = TaskHandle(0)
	try:
		CHK(nidaq.DAQmxCreateTask("", ctypes.byref(taskHandle)))
		CHK(nidaq.DAQmxCreateDOChan(taskHandle, channel, '', DAQmx_Val_ChanForAllLines))

		nwritten = int32(0)

#        val = numpy.array((val,), dtype=numpy.int16)
#       This requires shifting bits if writing to part of a port
#        CHK(nidaq.DAQmxWriteDigitalU16(taskHandle, int32(1), int32(1),
#            float64(1.0), int32(DAQmx_Val_GroupByChannel), val.ctypes.data, ctypes.byref(nwritten), None))

		vals = numpy.array([(val >> i) & 1 for i in range(8)], dtype=numpy.int8)
		nbytes = int32(0)
		CHK(nidaq.DAQmxGetWriteDigitalLinesBytesPerChan(taskHandle, ctypes.byref(nbytes)))
		CHK(nidaq.DAQmxWriteDigitalLines(taskHandle, int32(1), int32(1),
			float64(1.0), int32(DAQmx_Val_GroupByChannel), vals.ctypes.data, ctypes.byref(nwritten), None))

		CHK(nidaq.DAQmxStartTask(taskHandle))

	except Exception as e:
		logging.error('NI DAQ call failed: %s', str(e))

	finally:
		if taskHandle.value != 0:
			nidaq.DAQmxStopTask(taskHandle)
			nidaq.DAQmxClearTask(taskHandle)

nidaq.DAQmxExportSignal.argtypes = [ctypes.c_uint32, ctypes.c_int32, ctypes.POINTER(ctypes.c_char)]
def export_control_signal(taskHandle, signalID=DAQmx_Val_10MHzRefClock, outputTerminal='RefClock'):
	'''
	Routes a control signal to the specified terminal. The output terminal can reside on the device that generates the control signal or on a different device. Use this function to share clocks and triggers between multiple tasks and devices. The routes created by this function are task-based routes.
	'''
	return CHK(nidaq.DAQmxExportSignal(taskHandle, signalID, outputTerminal), 'export_control_signal()')

nidaq.DAQmxSetRefClkSrc.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_char)]
nidaq.DAQmxSetRefClkRate.argtypes = [ctypes.c_uint32, ctypes.c_double]
def set_refclk(taskHandle, clksrc='PXI_Clk10', freq=10e6):
	'''
	Specifies the terminal and assumed frequency of the signal to use as the Reference Clock.
	clksrc: Clock source (e.g. PXI_Clk10)
	freq: the assumed frequency of the incoming clock signal
	'''
	return CHK(nidaq.DAQmxSetRefClkSrc(taskHandle, clksrc), 'set_refclk().src') \
		and CHK(nidaq.DAQmxSetRefClkRate(taskHandle, freq), 'set_refclk().f')
	
nidaq.DAQmxCfgDigEdgeStartTrig.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_char), ctypes.c_int32]
def set_digedge_start_trigger(taskHandle, source='PXI_Trig0', edge=DAQmx_Val_Rising):
	'''Configures the task to start acquiring or generating samples on a rising or falling edge of a digital signal.'''
	return CHK(nidaq.DAQmxCfgDigEdgeStartTrig(taskHandle, source, edge), 'set_digedge_start_trigger')

nidaq.DAQmxGetReadAvailSampPerChan.argtypes = [ctypes.c_uint32, ctypes.POINTER(ctypes.c_uint32)]
def num_samples_in_instrument_buffer(taskHandle):
	'''Find out how many samples are left in the instrument's sample buffer. Returns None on failure.'''
	data = uInt32(0)
	if CHK(nidaq.DAQmxGetReadAvailSampPerChan(taskHandle, data), 'num_samples_in_instrument_buffer({:s})'.format(str(taskHandle))):
		return int(data.value)
	else:
		return None
