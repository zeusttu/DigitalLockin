# DigitalLockin
Implementation of a digital Lock-in amplifier as a virtual instrument operating over a virtual COM-port

Written for cross-compatibility with Python 2 and 3, though only tested in Python2

Actual hardware interaction and real measurements only work on Windows machines
with the NI-DAQmx and NI_FGEN drivers (both from National Instruments) installed.
Other features (most notably the simulated mode) also work on other, potentially
non-Windows systems, provided they have a sufficiently recent version of Python
installed along with the necessary Python packages.

## Supported commands
### **SELECT**
Select a lock-in instrument
### **SET** F|FS|A|T|PHASEOFFSET|MEASCH <value>
Set the value of a variable of the selected lock-in instrument
### **GET** RPHIBUFFER
Get multiline representation of all values of R and PHI acquired from
the selected lock-in instrument since last time they were queried
### **GET** RPHI|R|PHI|XY|X|Y
Get the last acquired values of R/PHI/X/Y from the selected lock-in
This function may still return multiple values (comma-separated) when
multiple channels are in use
### **GET** F|FS|A|T|PHASEOFFSET
Get value of excitation/measurement control variable of selected lock-in instrument
### **START**
Start measurements on the selected lock-in instrument
### **STOP**
Stop measurements on the selected lock-in instrument
### **CLOSE**
Close the selected lock-in instrument
### **CLOSE** ALL
Close all lock-in instruments and exit
### **PHASENULL**
Set the phase offset of the current lock-in instrument to the current phase

## Known issues
*  When setting parameters while a measurement is running, the parameters are never really
    set in the hardware. [Now solved for generator parameters (except channel) but not yet for Fs and channels]
*  No getter for dl_selected
*  When deleting a lock-in, the identifier of each lock-in with a higher identifier
    than the deleted one decreases by one, which may not be the best behaviour

## Potential code readability/architecture enhancements
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

## Fixed issues since the "known issues" list was created
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

## Useful utilities
### Matlab-qd plugin (included)
Plugin to use the lock-in with the Matlab-qd measurement acquisition software

### Null-modem emulator (com0com)
Utility for generating virtual COM-port pairs on Windows, this is necessary for connecting the virtual instrument to read-out software like Matlab-qd.
The original version with unsigned drivers can be found at [Sourceforge](http://sourceforge.net/projects/com0com/).
A 64-bit version with signed drivers can be found at [Google Code](https://code.google.com/p/powersdr-iq/downloads/detail?name=setup_com0com_W7_x64_signed.exe&can=2&q=).

