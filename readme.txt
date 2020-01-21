Dutchtronix AVR Oscilloscope Clock hardware 3-1.

October 6, 2010

Source code release for firmware V4.0

This archive contains the source code for the Dutchtronix AVR Oscilloscope Clock firmware V4.0. You can either load these files as a project in AVR Studio (I used Version 4.18, Build 700 and WinAVR-20100110) or build the code directly from the makefile in the "default" directory.

The default configuration builds firmware for the Dutchtronix AVR Oscilloscope Clock. To build the firmware for the Sparkfun O-Clock V1.1, you have to make one change:

	file ClkConfig.h
	change "#define ds1307 0" to "#define ds1307 1"
	change "#define pcf8563 1" to "#define pcf8563 0"

Any questions: email jan@dutchtronix.com

