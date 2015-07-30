# WWVB-DSP-clock-thing
WWVB DSP on a microcontroller

This is an older and (potentially non-working) version of a DSP program for a TI MSP430 microcontroller (specifically targeted at the G-series for the Launchpad).  The build was last tested on mspgcc on MacOS 10.8.

The basic idea is to plug the RF amp directly into a pin on the MSP and sample *really* fast with the ADC.  Even though the 10-bit ADC is barely adequate for the job, noise, processing gain, and a comb filter should give a useable signal.  

A main event loop runs indefinitely to process blocks of data from the ADC.  Each pass through the loop performs a write to the CPU status register to sleep the CPU, and the CPU is awoken by the ADC interrupt routine when a new block of data is available for processing (the stack is manipulated so the CPUOFF bit is not set when the interrupt routine returns).  Blocks of data from the ADC Data Transfer Controller, and double-buffering is used.

The MSP is  meant to be driven by a stable (steerable) oscillator, and the DSP program runs a costas loop to steer the time-base into phase lock with the WWVB signal.  In this first implementation, the clock source comes from the MSP's internal NCO, which is steered by the costas loop, but the final implementation is meant to use a crystal oscillator (preferably in an oven) for longer integration times (which gives better noise immunity). 

A special note about the time-base and ADC: In this implementation, the time-base does *not* need to run at a nice multiple of the WWVB RF carrier frequency (60 kHz).  One way to trigger the ADC is from the Timer A peripheral's capture-compare register in a mode where the timer counts to the compare value, and then resets.  Each reset triggers the ADC.  This has the nice property of not using very much CPU time, but requires an integer ratio between the WWVB carrier and the oscillator time base.  

Instead of that, this code uses a mode where each compare value match triggers the ADC and generates an interrupt (this must be at 8 times the carrier frequency, or 480 kHz for technical reasons described in the code).  The interrupt routine then adds a value to the capture register equal to one trigger period.  The purpose of doing this way is to allow the trigger period to be dithered on each cycle, effectively creating a fractional multiplier.

A special service routine was written in in-line assembly for the Timer A peripheral's capture-compare register.  The service routine takes 18 CPU cycles (total, including the time to accept the interrupt and return from the routine), which allows the interrupt to fire at about 1 MHz.  In practical application, the interrupt fires at 0.480 MHz and the balance of the time is used for the DSP program.

Another benefit of running the timer like this is that the other one or two (chip dependent) capture-compare registers and timer-A overflow interrupt are free to be used for other lower-priority purposes.  For example, a pulse-per-second waveform could be generated for locking an NTP server or a PWM waveform could be generated for analog output (perhaps to steer an oscillator).  

Some pins needed for the primary function of this project are shared with the hardware UART pins, so the file softUART-t2.c is an implementation of a softUART for use in this project.  It was successfully tested by jumpering its output to the Rx pin on the MSP Launchpad board.

