# WWVB-DSP-clock-thing
WWVB DSP on a microcontroller

This is a (potentially) old and non-working version of a DSP program for a TI MSP430 microcontroller (specifically targeted at the G-series for the Launchpad).  The basic idea is to plug the RF amp directly into a pin on the MSP and sample *really* fast.  Even though the 10-bit ADC is barely adequate for the job, noise, processing gain, and a comb filter should give a useable signal.  The MSP is also meant to be driven by a stable (steerable) oscillator, and the DSP program has a provision for adjusting the oscillator in a costas loop.  In this first implementation, the clock source comes from the MSP's internal NCO, which is steered by the costas loop. 

The interrupt service routines for the timers are written in assembly because they need to fire at around 480 kHz:  WWVB broadcasts at 60 kHz, and we want to sample four times per carrier cycle (=240 kHz) for the costas loop. The ADC only fires on one edge of the timer output signal, so the timer CCR needs to expire at 480 kHz.  Each time the timer expires, the interrupt routine sets the CCR compare value ahead by the appropriate number of counts. 
