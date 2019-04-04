# CLK-Syncronized-ADC-Sample-Buffer
Code created to respond to RS232 ASCII commands including: Link, Exit, Take Power Measurements, Update Relays, Send current Status.
Upon execution the device looks for interrupts (using level detection) from (CLK) running at 25hz from which the ADC syncronizes on rising edge and then takes x configurable samples, stores them to RAM and then sends back to terminal upon request.
