// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 7, 10/27/26
// * Sound file: sine wave through DAC1 (PA4) using DMA and TIM6, from Project 4
// *****************************************************************

#ifndef SOUND_H
#define SOUND_H

#include "stm32f4xx.h"

// Sets up PA4 (DAC), PB6 (amp shutdown), DMA1 Stream 5, DAC1 and TIM6, starting at 440 Hz
void Sound_Init(void);

// Retunes the tone; the frequency is given in hundredths of a Hz (440 Hz = 44000)
void Sound_SetFrequency(uint32_t freqHundredths);

// Turns the audio amplifier on (1) or off (0) using the active-low shutdown pin PB6
void Sound_Enable(uint8_t on);

#endif
