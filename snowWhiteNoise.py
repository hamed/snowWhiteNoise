#!/usr/bin/env python

# -*- coding: utf-8 -*-

# Created on Sun Aug 25 01:03:08 2013
# @file white_noise.py
# @author  Hamed Seyed-allaei <hamed@ipm.ir>
# @version 1.0
#
# @section LICENSE
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License as
# published by the Free Software Foundation; either version 2 of
# the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# General Public License for more details at
# http://www.gnu.org/copyleft/gpl.html

''' 
snowWhiteNoise.py
=================

Capture and analyze data sent by Arduino due
You should run this script while arduino is connected.


Principals
----------

You want to find the frequency response of a BlackBox. 
You feed it with a signal, and record the signal from it.

InPut -> BlackBox -> OutPut 

This code calculate the power spectra of input and output
and then calculate the gain.

A finer diagram is like this:

 TRNG -> DAC -> The BlackBox -\/\/\-> ADC
   |                                   |
   V                                   V
   PC                                  PC

where True Random Number Generator (TRNG), 
Digital to Analog Converter (DAC) and 
Analog to Digital Converter are inside arduino.

Both TRNG and ADC data are sent to PC via native USB port of arduino.

Data produced by TRNG are tagged by setting their most significant bit to 1.

You can rung this script like this:

    python white_noise.py --serial-port /dev/ttyACM1

in this, /dev/ttyACM1 is an example, it might be different on your computer. 
or 

    python white_noise.py --serial-port /dev/ttyACM1 --format png

this one will save figures as png file. You can try other formats as well, 
like eps and svg. The files will be named like this:
    time-in-second-from-epoch_nSamples_samplingRate.png

'''



from __future__ import division, absolute_import, print_function

import serial   # To read from SerialUSB
import array    # To store received data.
import matplotlib.pyplot as plt
import matplotlib.pylab as plb
import numpy as np
from optparse import OptionParser   # to parse command lines
import time   # to produce file names.


''' Chnage this 3 variables according to your needs.'''

nSamples = 2**22    # Number of samples to be taken,
                    # should be a power of two and should be 512 or more.

samplingRate = 200000   # Sampling rate per second.
                        # It's better if 42000000/samplingRate is an integer,
                        # because I have OCD :-)

NFFT = 2**11    # Must be a power of two
                # Must be smaller than nSamples
                # Big values give you lower frequencies,
                # Small values give you accurate results.

''' Look at the rest of the code as a Black Box!'''


a = array.array('H')  # The data will be stored in this array.
                      # 'H' means unsigned shot int, 2 bytes.


parser = OptionParser()
parser.add_option("-s", "--serial-port", dest="port", type="string",
                  help="Listen to PORT serial port. Like -s /dev/ttyACM0",
                  metavar="PORT")
parser.add_option("-f", "--format", dest="format", type="string",
                  help="File format for the outputs: png, pdf, eps, ps, svg",
                  metavar="FORMAT")

(options, args) = parser.parse_args()
print(options)

# Setup the serial port.
# baudrate? parity? and etc? I have no idea, I just copy pasted these lines :D
ser = serial.Serial(port=options.port,
                    baudrate=4000000,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    bytesize=serial.EIGHTBITS)

# Send Number of Samples and Sampling rate to the board.DON'T TOUCH this line.
ser.write(str(nSamples) + '\n' + str(samplingRate) + '\n')

# Read data:
# We need to transfer 4 bytes for each sample,
# 2 bytes the original random number, 2 bytes for data read from adc, 
# so each chunk of 512 bytes include 128 samples.
print("starting data acquisition.")
if ser.isOpen():
    for i in range(nSamples//128):
        a.fromstring(ser.read(512))

# We can close the port now.
ser.close()

# Process the data.
print("we got all the data. now we process them.")

b = np.array(a)  # Make a numpy array.



# We need to tag data so we know if it is from TRNG or ADC.
# We do it by putting a one in the most significant bit of data from TRNG.
# Here we use that tag to separate data written to DAC and data read from ADC.
# Then we remove it.
dac = b[b > 2**13] - 2**15  # Original data. we remove their tag.
adc = b[b < 2**13]  # Data read from ADC, they don't have any tag.
dac = 4*(dac - 2033)/6 + 2033   # DAC output is between 1/6 to 5/6 of 3.3V,
                                # It is safer if you check it for your device.
# Print some informations
print("acd mean:",  adc.mean())
print("acd std:",   adc.std())

print("acd max:",   max(adc))
print("acd min:",   min(adc))

print("dac mean:",  dac.mean())
print("dac std:",   dac.std())

print("dac max:",   max(dac))
print("dac min:",   min(dac))


# Base file name
filename=str(int(time.time())) + "_" + str(nSamples) + "_" + str(samplingRate)

# Power spectra, f is frequency
pdac, f = plt.psd(dac, NFFT=NFFT, Fs=samplingRate, detrend=plb.detrend_mean)
padc, f = plt.psd(adc, NFFT=NFFT, Fs=samplingRate, detrend=plb.detrend_mean)
plt.legend(["InPut","OutPut"],loc=0)


if options.format == None:
    plt.show()  # Showing graph
else:
    plt.savefig(filename+"_power."+options.format)  # Saving graph
plt.clf() # Cleaning current plot.

gain = 10 * np.log10(padc/pdac)     # Calculating Gain in dB

# -3 dB is an important number, isn't it?
y_ticks = np.array([-40, -36, -33, -30, -26, - 23,  -20, -16, -13, -10, -6, -3,
                    -1, 0, 1, 3, 6, 10, 13, 16, 20, 23, 26, 30, 33, 36, 40])

plt.semilogx(f, gain)
plt.yticks(y_ticks[(y_ticks<(gain.max()+1))&(y_ticks>(gain.min()-1))])
plt.grid(b=True);
plt.grid(b=True, which='minor');
plt.xlabel('Frequency (Hz)')
plt.ylabel('Gain (dB)')

if options.format == None:
    plt.show()
else:
    plt.savefig(filename+"_gain."+options.format)
plt.clf()

# Samples
plt.plot(3.3 * dac[-1000:-1] / 4096, 'x')
plt.plot(3.3 * adc[-1000:-1] / 4096, '.')
plt.title('Last 1000 samples')
plt.xlabel('Sample')
plt.ylabel('Voltage (V)')
plt.legend(["InPut","OutPut"], numpoints=1,loc=0)

if options.format == None:
    plt.show()
else:
    plt.savefig(filename+"_samples."+options.format)
