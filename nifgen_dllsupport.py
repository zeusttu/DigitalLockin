'''
nifgen.py, python wrapper for niFgen DLL (64-bit)

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

What is implemented:
  * All session initialization and closing functions
  * Configuring standard waveforms
  * Starting/stopping output of configured waveforms
  * Generating trigger for synchronisation with acquisition
  * Trigger on input reference clock which is much slower than sample clock [TODO]
What is not implemented:
  * Configuring arbitrary waveforms and sweeps
  * Configuring and enabling/disabling filters
  * Configuring operation mode (only CONTINUOUS supported)
  * Configuring which channels to use
  * Interchangeability info
  * Calibration functions
  * Most utility functions (mainly health checks and low-level stuff)
  * Trigger override
  * 32-bit support
'''

#Import python libraries
import ctypes
import logging

#Open DLL(64-bit)
nifgen_dll = ctypes.windll.niFgen_64

#Define some datatypes
TYPE_STRING = ctypes.POINTER(ctypes.c_char)
TYPE_BOOL = ctypes.c_uint16
TYPE_SESSIONID = ctypes.c_uint32
TYPE_FLOAT64 = ctypes.c_double
TYPE_VI_ATTR = ctypes.c_uint32

#Define attribute identifiers
IVI_ATTR_BASE = 1000000
IVI_SPECIFIC_PUBLIC_ATTR_BASE = IVI_ATTR_BASE + 150000
ATTR_STARTED_EVENT_OUTPUT_BEHAVIOR = IVI_SPECIFIC_PUBLIC_ATTR_BASE + 331
ATTR_STARTED_EVENT_LEVEL_ACTIVE_LEVEL = IVI_SPECIFIC_PUBLIC_ATTR_BASE + 316

#Define other constants
OUTPUTMODE_FUNC = 0
OUTPUTMODE_ARB = 1
OUTPUTMODE_SEQ = 2
OUTPUTMODE_FREQ_LIST = 101
OUTPUTMODE_SCRIPT = 102
WAVEFORM_SINE = 1
WAVEFORM_SQUARE = 2
WAVEFORM_TRIANGLE = 3
WAVEFORM_RAMP_UP = 4
WAVEFORM_RAMP_DOWN = 5
WAVEFORM_DC = 6
WAVEFORM_NOISE = 101
WAVEFORM_USER = 102
METAOUTPUT_BASE = 1000
METAOUTPUT_MARKER_EVENT = METAOUTPUT_BASE + 1
METAOUTPUT_SYNC_OUT = METAOUTPUT_BASE + 2
METAOUTPUT_ONBOARD_REFERENCE_CLOCK = METAOUTPUT_BASE + 19
METAOUTPUT_START_TRIGGER = METAOUTPUT_BASE + 4
METAOUTPUT_SAMPLE_CLOCK_TIMEBASE = METAOUTPUT_BASE + 6
METAOUTPUT_SYNCHRONIZATION = METAOUTPUT_BASE + 7
METAOUTPUT_SAMPLE_CLOCK = 101
METAOUTPUT_REFERENCE_CLOCK = 102
METAOUTPUT_SCRIPT_TRIGGER = 103
METAOUTPUT_READY_FOR_START_EVENT = 105
METAOUTPUT_STARTED_EVENT = 106
METAOUTPUT_DONE_EVENT = 107
METAOUTPUT_DATA_MARKER_EVENT = 108
TRIGGERMODE_SINGLE = 1
TRIGGERMODE_CONTINUOUS = 2
TRIGGERMODE_STEPPED = 3
TRIGGERMODE_BURST = 4
METAOUTPUT_SOFTWARE_TRIG = 2
CONTROL_SIGNAL_BHV_PULSE = 101
CONTROL_SIGNAL_BHV_LEVEL = 102
CONTROL_SIGNAL_BHV_TOGGLE = 103
CONTROL_SIGNAL_LEVEL_ACTIVE_HIGH = 101
CONTROL_SIGNAL_LEVEL_ACTIVE_LOW = 102

#Define argument types for used functions
#Error handling functions
nifgen_dll.niFgen_error_message.argtypes = [TYPE_SESSIONID, ctypes.c_int32, TYPE_STRING]
nifgen_dll.niFgen_error_query.argtypes = [TYPE_SESSIONID, ctypes.POINTER(ctypes.c_int32), TYPE_STRING]
nifgen_dll.niFgen_ClearError.argtypes = [TYPE_SESSIONID]
#Initialization and closing functions
nifgen_dll.niFgen_init.argtypes = [TYPE_STRING, TYPE_BOOL, TYPE_BOOL, ctypes.POINTER(TYPE_SESSIONID)]
nifgen_dll.niFgen_InitWithOptions.argtypes = [TYPE_STRING, TYPE_BOOL, TYPE_BOOL, TYPE_STRING, ctypes.POINTER(TYPE_SESSIONID)]
nifgen_dll.niFgen_reset.argtypes = [TYPE_SESSIONID]
nifgen_dll.niFgen_Commit.argtypes = [TYPE_SESSIONID]
nifgen_dll.niFgen_close.argtypes = [TYPE_SESSIONID]
#Configuration functions
nifgen_dll.niFgen_ConfigureOutputMode.argtypes = [TYPE_SESSIONID, ctypes.c_int32]
nifgen_dll.niFgen_ConfigureStandardWaveform.argtypes = [TYPE_SESSIONID, TYPE_STRING, ctypes.c_int32, TYPE_FLOAT64, TYPE_FLOAT64, TYPE_FLOAT64, TYPE_FLOAT64]
nifgen_dll.niFgen_ConfigureFrequency.argtypes = [TYPE_SESSIONID, TYPE_STRING, TYPE_FLOAT64]
nifgen_dll.niFgen_ConfigureAmplitude.argtypes = [TYPE_SESSIONID, TYPE_STRING, TYPE_FLOAT64]
nifgen_dll.niFgen_ConfigureReferenceClock.argtypes = [TYPE_SESSIONID, TYPE_STRING, TYPE_FLOAT64]
nifgen_dll.niFgen_ConfigureSoftwareEdgeStartTrigger.argtypes = [TYPE_SESSIONID]
nifgen_dll.niFgen_ConfigureTriggerMode.argtypes = [TYPE_SESSIONID, TYPE_STRING, ctypes.c_int32]
nifgen_dll.niFgen_DisableStartTrigger.argtypes = [TYPE_SESSIONID]
nifgen_dll.niFgen_ExportSignal.argtypes = [TYPE_SESSIONID, ctypes.c_int32, TYPE_STRING, TYPE_STRING]
#Signal generation enable/disable functions
nifgen_dll.niFgen_InitiateGeneration.argtypes = [TYPE_SESSIONID]
nifgen_dll.niFgen_AbortGeneration.argtypes = [TYPE_SESSIONID]
nifgen_dll.niFgen_SendSoftwareEdgeTrigger.argtypes = [TYPE_SESSIONID, ctypes.c_int32, TYPE_STRING]
#Output functions
nifgen_dll.niFgen_ConfigureOutputEnabled.argtypes = [TYPE_SESSIONID, TYPE_STRING, TYPE_BOOL]
nifgen_dll.niFgen_ConfigureOutputImpedance.argtypes = [TYPE_SESSIONID, TYPE_STRING, TYPE_FLOAT64]
#Low-level attribute functions (only used ones have been implemented)
nifgen_dll.niFgen_SetAttributeViInt32.argtypes = [TYPE_SESSIONID, TYPE_STRING, TYPE_VI_ATTR, ctypes.c_int32]

#Error checking routine
def CHK(err, fcn='', session=0, will_query=False):
	if err == 0:
		return True
	if fcn is None:
		fcn = ''
	buf_size = 256
	buf = ctypes.create_string_buffer('\000' * buf_size)
	nifgen_dll.niFgen_error_message(session, err, buf) #TODO should I use niFgen_error_message(), niFgen_GetError() or both?
	err_str = 'NI-Fgen call \'%s\' failed with error %d: %s' % (fcn, err, repr(buf.value))
	if will_query:
		err2 = ctypes.c_int32()
		nifgen_dll.niFgen_error_query(session, ctypes.byref(err2), buf)
		err_str = '%s\nAdditional info (error %d):\n%s' % (err_str, err2.value, repr(buf.value))
	if err < 0:
		raise RuntimeError(err_str)
		return False
	elif err > 0:
		raise RuntimeWarning(err_str)
		niFgen_ClearError(session)
		return True

#Check for error but do not throw potentially fatal exception
def CHKNF(err, fcn='', session=0, will_query=False):
	try:
		return CHK(err, fcn, session, will_query)
	except Exception as e:
		if err < 0:
			logging.exception('FGEN:')
		else:
			logging.warning('FGEN: %s', str(e))
		return False

		
################################################
##### Initialization and closing functions #####
################################################

#Initialize session
def init(dev='PXI5412_12', checkid=True, resetdev=True, options=None):
	#session = ctypes.c_uint32(0)
	session = TYPE_SESSIONID()
	if options is None:
		if CHK(nifgen_dll.niFgen_init(dev,checkid,resetdev,session), 'init'):
			return session
		else: return None
	elif CHK(nifgen_dll.niFgen_InitWithOptions(dev,checkid,resetdev,options,session), 'init'):
		return session
	else:
		return None

#Reset session
def reset(session):
	return CHK(nifgen_dll.niFgen_reset(session), 'reset', session)

#"Commit" session (the documentation does not make it entirely clear to me what this does)
def commit(session):
	return CHK(nifgen_dll.niFgen_Commit(session), 'commit', session)

#Close session
def close(session):
	return CHK(nifgen_dll.niFgen_close(session), 'close', session)

	
############################################
##### Waveform configuration functions #####
############################################

#Configure output mode
def configure_output_mode(session, outputmode=OUTPUTMODE_FUNC):
	return CHK(nifgen_dll.niFgen_ConfigureOutputMode(session, outputmode), 'configure_output_mode', session)

#Configure all parameters of standard waveform
def configure_standard_waveform(session, channelname='0', waveform=WAVEFORM_SINE, amplitude=1., offset=0., frequency=1000., phase=0.):
	return CHK(nifgen_dll.niFgen_ConfigureStandardWaveform(session, channelname, waveform, amplitude, offset, frequency, phase), 'configure_standard_waveform', session)

#Reconfigure frequency of standard waveform
def set_frequency(session, channelname='0', frequency=1000):
	return CHK(nifgen_dll.niFgen_ConfigureFrequency(session, channelname, frequency), 'set_frequency', session)

#Reconfigure amplitude of standard waveform
def set_amplitude(session, channelname='0', amplitude=1):
	logging.info('set_amplitude(session={:s}, channel={:s}, amplitude={:.2f})'.format(str(session), str(channelname), amplitude))
	return CHK(nifgen_dll.niFgen_ConfigureAmplitude(session, channelname, amplitude), 'set_amplitude', session)

#Configure trigger source ("reference clock")
#There is also another triggering method which uses the trigger directly as the sample clock
#But that has not been implemented here because it's only useful when the trigger frequency equals the desired sample rate
#In our case the trigger frequency is an order of magnitude lower so the "reference clock" method is more appropriate
def configure_reference_clock(session, clocksource='PXI_Clk10', clockfrequency=10e6):
	return CHK(nifgen_dll.niFgen_ConfigureReferenceClock(session, clocksource, clockfrequency), 'configure_reference_clock()', session)

#Configure software generated start trigger
def enable_software_start_trigger(session):
	return CHK(nifgen_dll.niFgen_ConfigureSoftwareEdgeStartTrigger(session), 'enable_software_start_trigger()', session)

#Disable start trigger
def disable_start_trigger(session):
	return CHK(nifgen_dll.niFgen_DisableStartTrigger(session), 'disable_start_trigger()', session)

#Set trigger mode
def configure_trigger_mode(session, channelname='0', triggermode=TRIGGERMODE_SINGLE):
	return CHKNF(nifgen_dll.niFgen_ConfigureTriggerMode(session, channelname, triggermode), 'configure_trigger_mode()', session, True)

#Configure output of trigger signal
#The selected signal will keep the selected terminal occupied until it is set to be output on no terminal ('')
#This configuration may survive between sessions
def configure_export_signal(session, signal=METAOUTPUT_START_TRIGGER, signalidentifier='', terminal='PXI_Trig0'):
	return CHKNF(nifgen_dll.niFgen_ExportSignal(session, signal, signalidentifier, terminal), 'configure_export_signal()', session)

#Deconfigure output of one or more trigger signals (you must specify which)
#When deconfiguring multiple signals, they must be put in a list (not a tuple, numpy.array, matrix or whatever)
def deconfigure_export_signal(session, signal=METAOUTPUT_START_TRIGGER, signalidentifier=''):
	so_far_successful = True
	if isinstance(signal, list):
		for i in range(len(signal)):
			if isinstance(signalidentifier, list):
				so_far_successful &= deconfigure_export_signal(session, signal[i], signalidentifier[i])
			else:
				so_far_successful &= deconfigure_export_signal(session, signal[i], signalidentifier)
		return so_far_successful
	elif isinstance(signalidentifier, list):
		for i in range(len(signalidentifier)):
			so_far_successful &= deconfigure_export_signal(signal, signalidentifier[i])
		return so_far_successful
	else:
		return configure_export_signal(session, signal, signalidentifier, '')


##########################################
##### Start/stop waveform generation #####
##########################################

#Start generation of signal (does not enable output yet)
def initiate_generation(session):
	return CHK(nifgen_dll.niFgen_InitiateGeneration(session), 'initiate_generation', session)

#Abort generation of signal
def abort_generation(session):
	return CHK(nifgen_dll.niFgen_AbortGeneration(session), 'abort_generation', session)

#Send software-generated trigger
def send_trigger(session, triggertype=METAOUTPUT_START_TRIGGER, triggerID=''):
	return CHK(nifgen_dll.niFgen_SendSoftwareEdgeTrigger(session, triggertype, triggerID), 'send_trigger()', session)


#################################
##### Enable/disable output #####
#################################

#Enable output
def output_enable(session, channelname='0'):
	return CHK(nifgen_dll.niFgen_ConfigureOutputEnabled(session, channelname, True), 'output_enable', session)

#Disable output
def output_disable(session, channelname='0'):
	return CHK(nifgen_dll.niFgen_ConfigureOutputEnabled(session, channelname, False), 'output_disable', session)

#Define anticipated output impedance
def set_output_impedance(session, channelname='0', R=50.):
	return CHK(nifgen_dll.niFgen_ConfigureOutputImpedance(session,channelname,R), 'set_output_impedance', session)
