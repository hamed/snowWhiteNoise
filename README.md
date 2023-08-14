
Snow White Noise and the Black Box
==================================
[![DOI](https://zenodo.org/badge/21629/hamed/snowWhiteNoise.svg)](https://zenodo.org/badge/latestdoi/21629/hamed/snowWhiteNoise)

Hamed Seyed-allaei <hamed@ipm.ir>

If you have got a __Black Box__ and you want to know its frequency response, 
__Snow White Noise__ is for you. 
If you want to __learn__ or __teach__ filters, FFT, power spectrum and white noise analysis, 
__Snow White Noise__ is for you.

snowWhiteNoise sends white noise to your black box and then records output
of your black box and make a Gain-Frequency graph of you device. 
You can also have the raw data and do your own analysis on them.

I wrote this codes to test my home made preamp. I have got the inspiration while attending 
Hand-on research in complex systems.
<http://www.handsonresearch.org/>

This code uses DMA so it is fast and can handle sampling rates up to 200kHz.
I followed @pklaus code to handle buffers and use DMA and ADC.

The Black Box
-------------

The black box can be any thing:

* A home made neural recording device,
* A brand new Hi-Fi system,
* An Optoelectronic Coupler, 
* Any passive or active high-pass, low-pass, bandpass, bandstop filter,
* Or anything that accepts an input signal and gives you back an output signal.


Ingredients
-----------

1.  [Arduino Due](https://amzn.to/3YC6iJg) and its softwares
2.  A Linux based PC 
3.  Python(2.7), Numpy, Matplotlib and pySerial
4.  and of curse, A Black Box, to experiment with, 
5.  and necessary hardwares to protect your Black Box and Arduino against over voltage and over loads.

__Take number 5 seriously__, if now, you may damage your Arduino or your brand new Hi-Fi.


Limitations
-----------

* 0V-3.3V is in put of Arduinos ADC pins,
* 1/6 to 5/6 of 3.3V is the out put of Arduinos DAC pins,
* 200KHz is the maximum frequency of this setting.


Instructions
============

1. Upload the Arduino Code, snowWhiteNoise.ino to your Arduino Due.
2. Connect your Arduino via Native USB port to PC.
3. Connect the input of your BlackBox to DAC1 and GND pin of Arduino board via necessary circuits.
4. Connect the output of your BlackBox to A7 and GND pin of Arduino board via necessary circuits.
5. Edit the number of samples and sampling rate in snowWhiteNoise.py, if needed.
6. Run snowWhiteNoise.py on your PC.
`python snowWhiteNoise.py --serial-port /dev/ttyACM0`
If every thing is right, you will see a graph. close it to see the next graph.
7. Make any change that you like in your Black Box and/or snowWhiteNoise.py and go to step 6. No need to reset Arduino.

`/dev/ttyACM0` in step 6 might be different on your device.


Parameters
----------

### Number of Samples
You can set the number of samples and sampling rate *inside python script*
	nSamples = 2**20    # Number of samples to be taken,
- This number should be atleast 1024 to give you some result worth looking at.
- It must be `2^n`, where `n` is an integer number. 
- If you change this, you may need to change NFFT as well.
This is how Fast Fourier Transform (FFT) works.


### Sampling Rate

The theoretical limit is 1MHz (The maximum frequency of ADC in Arduino Due), 
but the practical limit probably is less, because you can do only 84 operation 
for each sample at best, and, Arduino needs to send 4 bytes to computer 
for every sample. So according to my tests, 200KHz is the practical upper limit.
This is more than what I needed for my own work. But if you need more, you have to 
optimize the code as well. 

To just give you a filling, 

* Your ears are responsive up to 20KHz,
* Many acquisition devices works at 30KHz,
* The sound card of your PC is at best 196KHz.

To change sampling rate, find this line:
	samplingRate = 200000   # Sampling rate per second.
It's better if 42000000/samplingRate is an integer. ( I have OCD :-)

### FFT window size

A third parameter you might need to change is NFFT. To calculate the spectral 
density, the data are divided into segments of NFFT data points. Then FFT is 
calculated over each segments and the results are averaged. So smaller NFFT 
will give you less noisier results at the cost of loosing low frequencies. 
	NFFT = 2**13    # Must be 2^n where n is an integer.
Be ware that NFFT must be smaller than Number of Samples,


Principals
==========

You want to find the frequency response of a BlackBox. 
You feed it with a signal, and record the signal from it.

	InPut -> BlackBox -> OutPut

Then, for a given frequency, gain in dB is:

	gain = 10 * log10(OutPut/InPut)

An obvious choice for the InPut signal is sine wave, and like many other 
obvious choices in life, it is *boring*!
We can do better than that, we can use white noise instead. It will give you 
better result for the same amount of data points, in a wider range of frequencies. 
The good news is that [Arduino Due](https://amzn.to/3YC6iJg) has an on-chip True Random Number Generator, 
so producing a white noise is pretty strain forward.

The diagram of our setup is like this:

	TRNG -> DAC -> The BlackBox -> ADC
	  |                             |
	  V                             V
	  PC                            PC

Where True Random Number Generator (TRNG), Digital to Analog Converter (DAC) 
and Analog to Digital Converter are inside Arduino.

Both TRNG and ADC send data to PC via native USB port of Arduino. Data produced 
by TRNG are tagged by setting their most significant bit to 1.

You can rung this script like this:
	python snowWhiteNoise.py --serial-port /dev/ttyACM0
in this, `/dev/ttyACM0` is an example, it might be different on your computer. 
Run the following to save graphs: 
	python white_noise.py --serial-port /dev/ttyACM1 --format png
this one will save figures as png file. You can try other formats as well, 
like *eps*, *pdf* and *svg*. The files will be named like this:
	time-in-second-from-epoch_nSamples_samplingRate.png


Timings
=======

To keep the timing accurate, I used Arduinos internal Timer Counter. 
The timer can generate signals upto 42MHz, and the timing are pretty 
accurate according to my oscilloscope.

Timer Counter
-------------

We set 2 separate timers for each DAC and ADC devices: Channel 0 and 1 of TC0.
This timers are in the same block and synchronized. But their output signal is 
different (TIOA0 and TIOA0). 
* TIOA1 controls DAC.
* TIOA0 controls ADC.

The relation between the clocks are

	a                                                                              a
	a   ______    ______    ______    ______    ______    ______    _              a
	a__|      |__|      |__|      |__|      |__|      |__|      |__|     TIOA1     a
	a     _________                     _________                                  a
	a    |         |_________          |         |          _________              a
	a____|                   |_________|         |_________|         |   DAC1      a
	a          _         _         _         _         _         _                 a
	a_________| |_______| |_______| |_______| |_______| |_______| |___   ADC clock a
	a          ____      ____      ____      ____      ____      ____              a
	a_________|    |____|    |____|    |____|    |____|    |____|    |   TIOA0     a
	a                                                                              a


DACC send out the signal after 25 clocks, so we start DAC 25 clocks earlier than ADC.
So we can be sure that  we are  measuring the center of the signal.

