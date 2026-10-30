// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 7, 10/27/26
// * Sound file: sine wave through DAC1 (PA4) using DMA and TIM6, from Project 4
// *****************************************************************

#include "Sound.h"

#define DAC_PIN       4
#define AUDIO_SD_PIN  6
#define SINE_SAMPLES  100
#define TIM6_CLOCK    90000000UL  // APB1 timer clock at 180 MHz (45 MHz x 2); Project 4 used 16 MHz

// 100-sample sine LUT centered at 2048, spanning 1048-3048 (from Dr. Martin)
static const uint16_t sine_lut[SINE_SAMPLES] = {
    2048, 2110, 2173, 2235, 2296, 2357, 2416, 2474, 2530, 2584,
    2636, 2685, 2732, 2777, 2818, 2857, 2892, 2924, 2953, 2978,
    2999, 3016, 3030, 3040, 3046, 3048, 3046, 3040, 3030, 3016,
    2999, 2978, 2953, 2924, 2892, 2857, 2818, 2777, 2732, 2685,
    2636, 2584, 2530, 2474, 2416, 2357, 2296, 2235, 2173, 2110,
    2048, 1985, 1922, 1860, 1799, 1738, 1679, 1621, 1565, 1511,
    1459, 1410, 1363, 1318, 1277, 1238, 1203, 1171, 1142, 1117,
    1096, 1079, 1065, 1055, 1049, 1048, 1049, 1055, 1065, 1079,
    1096, 1117, 1142, 1171, 1203, 1238, 1277, 1318, 1363, 1410,
    1459, 1511, 1565, 1621, 1679, 1738, 1799, 1860, 1922, 1985
};

// Converts a frequency in hundredths of a Hz to TIM6's ARR; the DAC needs 100 samples per cycle
static uint32_t freqToArr(uint32_t freqHundredths) {
    return ((TIM6_CLOCK + (freqHundredths / 2U)) / freqHundredths) - 1U;
}

// Sets PA4 to analog for the DAC and PB6 to a digital output for the amp shutdown
static void Sound_GPIO_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;

    GPIOA->MODER |= (0x3U << (DAC_PIN * 2));
    GPIOB->MODER &= ~(0x3U << (AUDIO_SD_PIN * 2));
    GPIOB->MODER |=  (0x1U << (AUDIO_SD_PIN * 2));
}

// DMA copies samples from the sine LUT to the DAC register in a loop with no CPU work
static void Sound_DMA_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;

    DMA1_Stream5->CR &= ~DMA_SxCR_EN;
    while (DMA1_Stream5->CR & DMA_SxCR_EN) { }

    DMA1_Stream5->CR |= (7U << DMA_SxCR_CHSEL_Pos);                                  // Channel 7 (DAC1)
    DMA1_Stream5->CR |= (0x1U << DMA_SxCR_MSIZE_Pos) | (0x1U << DMA_SxCR_PSIZE_Pos); // 16-bit
    DMA1_Stream5->CR |= DMA_SxCR_MINC;                                               // advance through the LUT
    DMA1_Stream5->CR |= (0x1U << DMA_SxCR_DIR_Pos);                                  // memory -> peripheral
    DMA1_Stream5->CR |= DMA_SxCR_CIRC;                                               // loop the LUT forever

    DMA1_Stream5->NDTR = SINE_SAMPLES;
    DMA1_Stream5->PAR  = (uint32_t)&(DAC->DHR12R1);
    DMA1_Stream5->M0AR = (uint32_t)sine_lut;

    DMA1_Stream5->CR |= DMA_SxCR_EN;
}

// Enables DAC channel 1, triggered by TIM6's TRGO, requesting DMA on every trigger
static void Sound_DAC_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    DAC->CR &= ~DAC_CR_TSEL1;   // TSEL1 = 000b -> TIM6 TRGO
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_DMAEN1;
    DAC->CR |= DAC_CR_EN1;
}

// Starts TIM6 as the DAC's pacing timer at 100 samples per cycle
static void Sound_TIM6_Init(uint32_t freqHundredths) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 0; // full 90 MHz timer clock
    TIM6->ARR = freqToArr(freqHundredths);
    TIM6->CR2 &= ~TIM_CR2_MMS;
    TIM6->CR2 |= (0x2U << TIM_CR2_MMS_Pos); // update event -> TRGO
    TIM6->CR1 |= TIM_CR1_CEN;
}

// Sets up the whole sound path, starting at 440 Hz with the amp off
void Sound_Init(void) {
    Sound_GPIO_Init();
    Sound_Enable(0);
    Sound_DMA_Init();
    Sound_DAC_Init();
    Sound_TIM6_Init(44000U);
}

// Retunes TIM6's ARR to a new frequency and forces an immediate reload for a clean transition
void Sound_SetFrequency(uint32_t freqHundredths) {
    if (freqHundredths == 0U) return; // avoid dividing by zero; the amp is shut down for OFF instead
    TIM6->ARR = freqToArr(freqHundredths);
    TIM6->EGR = TIM_EGR_UG;
}

// PB6 is the amp's active-low shutdown pin: high = amp on, low = amp off
void Sound_Enable(uint8_t on) {
    if (on) {
        GPIOB->BSRR = (1U << AUDIO_SD_PIN);
    } else {
        GPIOB->BSRR = (1U << (AUDIO_SD_PIN + 16U));
    }
}
