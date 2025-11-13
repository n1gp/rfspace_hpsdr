rfspace_usb2hpsdr is an RFSPACE USB SDR to HPSDR software translation server.

It currently builds and runs on Linux, and supports the SDR-IP and SDR-14.

A real HPSDR radio's (i.e. Hermes) frequency range is from 10KHz to 55MHz.
But the SDR-14 and SDR-IQ are only up to 30MHz.

The main purpose of this program is to provide a mechanism that
allows RFSPACE USB SDR owners the ability to use them on HPSDR specific
software programs.

Since the real HPSDR (i.e. Hermes) rcvr can do up to eight rcvr
slices, there is a concept of 'COPY' rcvrs in this server. This
would allow one to use HPSDR programs that expected more rcvrs
than were attached. Currently if a program request more rcvrs
than are actually attached the rfspace_usb2hpsdr server will make copies
of the last 'real' rcvr. This allows one to only have one SDR device
attached and run PowerSDR mRX or Thetis, which may expect up to four rcvr
slices.

Refer to: http://openhpsdr.org/softwareinfo.php  for a list of
HPSDR supported applications.

These are two very useful OpenHPSDR programs:
https://github.com/ramdor/Thetis
https://github.com/dl1ycf/pihpsdr    

I have tested this on both Thetis and PiHPSDR using and SDR-IP and
an SDR-14. The SDR-IQ support is currently untested.

Useful documentation:
http://openhpsdr.org/support/Ozy/USB_protocol_V1.48.pdf
http://svn.tapr.org/repos_sdr_hpsdr/trunk/Metis/Documentation/Metis-%20How%20it%20works_V1.30.pdf

Feel free to fix / add / modify this program.
I'd appreciate keeping me updated, n1gp@hotmail.com


1. Dependencies

	sudo apt-get install libliquid-dev libftdi1-dev libfftw3-dev

2. Installation

	$cd rfspace_usb2hpsdr
	$ make

3. Usage

	$ ./rfspace_usb2hpsdr 
	  0.000000 Starting usb_read_task_func()
	  0.000795 Using RFSPACE SDR-14 SN ER000053
	<liquid.resamp_crcf, rate=0.979202, m=13, as=60.000, fc=0.45, npfb=64>
	  0.181959 Waiting on Discovery...

4. Known problems

- I notice that when I first plug in the USB cable from my SDR-14
  to the PC, the rfspace_usb2hpsdr program hangs at:
     Starting usb_read_task_func()

  If this occurs, either use CTRL-C and restart it, or CTRL-Z and kill
  and retry the program.

