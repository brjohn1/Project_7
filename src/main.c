// *****************************************************************
// * Author: Brian Johnson
// * CPEG222 Project 7, 10/27/26
// * NucleoF446ZE + CPEG222 Shield - CMSIS bare metal
// *
// * Theremin: SysTick starts a reading every 200 ms, the HC-SR04 measures
// * the hand distance (EXTI1 + TIM7), and the distance picks a note that is
// * played through the DAC/DMA, shown on the OLED and SSD, and shown as a
// * color on the NeoPixels. Slide switch S1 enables the audio.
// *****************************************************************


/*
 * PROJECT 7 - THEREMIN
 *
 * SysTick - 1 ms interrupts; measurement flag every 200 ms
 * TIM7    - Microsecond timing for ultrasonic echo
 * EXTI1   - Detects rising/falling edges on PG1
 * TIM5    - 400 Hz interrupts for SSD refresh
 *
 * HC-SR04 TRIG - PE3
 * HC-SR04 ECHO - PG1
 * Switch S1    - PC8
 *
 * Main program converts distance into note, frequency,
 * and RGB color, then updates the output peripherals.
 */
#include "stm32f4xx.h"
#include "OLED.h"
#include "SysClock180MHz.h"
#include "HCSR04.h"
#include "ssd.h"
#include "Sound.h"
#include "NeoPixel.h"
#include <stdint.h>
#include <stdio.h>

#define S1_PORT            GPIOC
#define S1_PIN             8U      // S1 = PC8
#define MEASURE_PERIOD_MS  200U    // 5 readings per second
#define ECHO_TIMEOUT_MS    40U     // give up if no echo comes back in 40 ms
#define US_PER_CM          58.3f   // round trip: 2 cm of travel takes about 58.3 us at 343 m/s
#define CM_PER_INCH        2.54f

// One row of the notes table: name, frequency in hundredths of a Hz, and RGB color
typedef struct {
    const char *name;
    uint32_t    freqHundredths;
    uint8_t     r, g, b;
} Note;

#define NOTE_OFF 12  // index of the OFF entry (distance of 15 inches or more)

// Notes by inch: index 0 is C4 (under 4 in), then one note per inch up to B4 (14 to 15 in), then OFF
static const Note notes[13] = {
    {"C4",  26163,   0,   0, 255},
    {"C#4", 27718,   0,  93, 255},
    {"D4",  29366,   0, 185, 255},
    {"D#4", 31113,   0, 255, 232},
    {"E4",  32963,   0, 255, 139},
    {"F4",  34923,   0, 255,  46},
    {"F#4", 36999,  46, 255,   0},
    {"G4",  39200, 139, 255,   0},
    {"G#4", 41530, 232, 255,   0},
    {"A4",  44000, 255, 185,   0},
    {"A#4", 46616, 255,  93,   0},
    {"B4",  49388, 255,   0,   0},
    {"OFF",     0,   0,   0,   0}
};

// Variables shared between the interrupts and the main loop
volatile uint32_t msTicks = 0;      // counts milliseconds, updated by SysTick
volatile uint8_t  measureFlag = 0;  // set by SysTick every 200 ms to start a reading
volatile uint8_t  echoDone = 0;     // set by EXTI1 when a full ECHO pulse was captured
volatile uint8_t  echoArmed = 0;    // set on a rising edge so a falling edge is only trusted after it
volatile uint16_t echoStart = 0;    // TIM7 count at the rising edge
volatile uint16_t pulseWidth = 0;   // ECHO pulse width in us

static uint8_t currentNote = 0xFF;  // note shown right now; 0xFF means nothing shown yet

// Turns on the floating point unit so float math runs in hardware instead of faulting
static void FPU_Enable(void) {
    SCB->CPACR |= (0xFU << 20);
    __DSB();
    __ISB();
}

// Sets slide switch S1 (PC8) up as a plain digital input
static void S1_Init(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
    S1_PORT->MODER &= ~(0x3U << (S1_PIN * 2U));
}

// Routes PG1 to EXTI line 1 and interrupts on both the rising and falling edge of ECHO
static void EXTI_Init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    SYSCFG->EXTICR[0] &= ~(0xFU << 4);
    SYSCFG->EXTICR[0] |=  (0x6U << 4);  // 0b0110 selects port G for line 1
    EXTI->IMR  |= (1U << 1);
    EXTI->RTSR |= (1U << 1);
    EXTI->FTSR |= (1U << 1);
    NVIC_SetPriority(EXTI1_IRQn, 0);    // highest priority so edge timestamps are never delayed
    NVIC_EnableIRQ(EXTI1_IRQn);
}

// Configures SysTick to interrupt every 1 ms (SysTick alone cannot count 200 ms, its limit is about 93 ms)
static void SysTick_Init(void) {
    SysTick->LOAD = (SystemCoreClock / 1000U) - 1U;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, 1);
}

// Configures TIM5 to interrupt at 400 Hz (every 2.5 ms) so each of the 4 SSD digits refreshes at 100 Hz
static void TIM5_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;

    TIM5->PSC = 8999;  // 90 MHz / 9000 = 10 kHz -> 0.1 ms per tick (Project 4 used 1599 at 16 MHz)
    TIM5->ARR = 24;    // 25 ticks = 2.5 ms period -> 400 Hz
    TIM5->EGR = TIM_EGR_UG;
    TIM5->SR  = ~TIM_SR_UIF;
    TIM5->DIER |= TIM_DIER_UIE;

    NVIC_SetPriority(TIM5_IRQn, 2);
    NVIC_EnableIRQ(TIM5_IRQn);

    TIM5->CR1 = TIM_CR1_CEN;
}

// Every 1 ms: counts time, and every 200 ms raises the flag that starts a new reading
void SysTick_Handler(void) {
    static uint32_t periodCount = 0;
    msTicks++;
    if (++periodCount >= MEASURE_PERIOD_MS) {
        periodCount = 0;
        measureFlag = 1;
    }
}

// Every 2.5 ms: refreshes one SSD digit
void TIM5_IRQHandler(void) {
    if (TIM5->SR & TIM_SR_UIF) {
        TIM5->SR = ~TIM_SR_UIF;
        SSD_Refresh();
    }
}

// EXTI1 detects both ECHO edges: the rising edge saves the TIM7 time, the falling edge computes the pulse width
void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1U << 1)) {
        EXTI->PR = (1U << 1);
        if (HCSR04_ECHO_PORT->IDR & (1U << HCSR04_ECHO_PIN)) {
            echoStart = HCSR04_Timestamp();
            echoArmed = 1;
        } else if (echoArmed) {
            pulseWidth = (uint16_t)(HCSR04_Timestamp() - echoStart); // handles TIM7 wraparound
            echoArmed = 0;
            echoDone = 1;
        }
    }
}

// Prints a line on the OLED padded to 21 characters so leftovers from the previous reading are erased
static void showLine(uint8_t row, const char *text) {
    char padded[24];
    snprintf(padded, sizeof(padded), "%-21s", text);
    OLED_Print(row, 0, padded);
}

// Shows the note's color on all four NeoPixels, only when the note has changed
static void updateNeoPixels(uint8_t noteIndex) {
    if (noteIndex == currentNote) return;
    currentNote = noteIndex;
    NeoPixel_SetAll(notes[noteIndex].r, notes[noteIndex].g, notes[noteIndex].b);
    NeoPixel_Show_PB11();
}

// Triggers the sensor, waits for the echo, then updates the note, tone, OLED, SSD and NeoPixels
static void takeMeasurement(void) {
    char line[32];
    uint8_t noteIndex = NOTE_OFF;

    echoDone = 0;
    echoArmed = 0;
    HCSR04_Trigger();

    uint32_t startMs = msTicks;
    while (!echoDone && (msTicks - startMs) < ECHO_TIMEOUT_MS) { }

    if (!echoDone) {
        // No echo means nothing in range, so treat it like 15 inches or more
        showLine(3, "Pulse: no echo");
        showLine(4, "Dist: --- in");
    } else {
        uint32_t pw = pulseWidth;
        float inches = (float)pw / (US_PER_CM * CM_PER_INCH);

        // One note per whole inch: 4 to 5 in is C#4 (index 1), ..., 14 to 15 in is B4 (index 11)
        int whole = (int)inches;
        if (whole >= 15) {
            noteIndex = NOTE_OFF;
        } else if (whole < 4) {
            noteIndex = 0;
        } else {
            noteIndex = (uint8_t)(whole - 3);
        }

        // Split into whole and hundredths as integers, since printing floats with snprintf is unreliable here
        uint32_t hundredths = (uint32_t)(inches * 100.0f + 0.5f);
        snprintf(line, sizeof(line), "Pulse: %lu us", (unsigned long)pw);
        showLine(3, line);
        snprintf(line, sizeof(line), "Dist: %lu.%02lu in", (unsigned long)(hundredths / 100U), (unsigned long)(hundredths % 100U));
        showLine(4, line);
    }

    snprintf(line, sizeof(line), "Note: %s", notes[noteIndex].name);
    showLine(5, line);

    // SSD shows the frequency as xxx.x (truncated, e.g. 369.99 Hz shows 369.9)
    SSD_SetValueTenths((uint16_t)(notes[noteIndex].freqHundredths / 10U));

    if (noteIndex != NOTE_OFF) {
        Sound_SetFrequency(notes[noteIndex].freqHundredths);
    }
    updateNeoPixels(noteIndex); // also records the current note, which the main loop uses to decide if the amp may be on
}

int main(void) {
    FPU_Enable();

    // Same clock setup as Project 5: 16 MHz at reset, then 180 MHz
    SystemCoreClockUpdate();
    SystemClock_Config();
    SystemCoreClockUpdate();

    // Peripherals are set up after the clock change so every timer value is correct
    OLED_Init();
    HCSR04_Init();
    S1_Init();
    EXTI_Init();
    SSD_Init();
    SSD_SetDecimalPoint(2);   // xxx.x format: the decimal point sits after the ones digit
    SSD_SetValueTenths(0);
    Sound_Init();
    NeoPixel_Init_PB11();
    TIM5_Init();

    OLED_Clear();
    OLED_Print(0, 0, "HC-SR04 Theremin");
    OLED_Print(1, 0, "Brian Johnson");

    NeoPixel_SetAll(0, 0, 0); // start with all NeoPixels off
    NeoPixel_Show_PB11();

    SysTick_Init();           // start last so the first reading comes 200 ms after everything is ready

    // Each time SysTick raises the flag, take one reading; S1 controls whether the amp is on
    while (1) {
        if (measureFlag) {
            measureFlag = 0;
            takeMeasurement();
        }
        Sound_Enable(((S1_PORT->IDR & (1U << S1_PIN)) != 0U) && (currentNote != NOTE_OFF) && (currentNote != 0xFF));
    }
    return 0;
}
