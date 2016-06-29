#ifndef PULSE_H
#define PULSE_H

void single_pulse(uint8_t capBank);
void paired_pulse(uint8_t ISI); // ISI = InterStimulus Interval in ms
void repetitive_pulse(uint8_t stimulation_frequency);
void delay_in_ms(double ms);
#endif
