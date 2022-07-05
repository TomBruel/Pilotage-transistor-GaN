//#############################################################################
//
// FILE:   adc_ex2_soc_epwm.c
//
// TITLE:  ADC ePWM Triggering
//
//! \addtogroup driver_example_list
//! <h1>ADC ePWM Triggering</h1>
//!
//! This example sets up ePWM1 to periodically trigger a conversion on ADCA.
//!
//! \b External \b Connections \n
//!  - A0 should be connected to a signal to convert
//!
//! \b Watch \b Variables \n
//! - \b adcAResults - A sequence of analog-to-digital conversion samples from
//!   pin A0. The time between samples is determined based on the period
//!   of the ePWM timer.
//!
//
//#############################################################################
// $TI Release: F2837xD Support Library v3.06.00.00 $
// $Release Date: Mon May 27 06:48:24 CDT 2019 $
// $Copyright:
// Copyright (C) 2013-2019 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//#############################################################################

//
// Included Files
//
#include "driverlib.h"
#include "device.h"
#include "DCL.h"
#include "DCLF32.h"
#include "dlog_4ch.h"
//
// Defines
//
#define RESULTS_BUFFER_SIZE     256
#define EX_ADC_RESOLUTION       12
#define Vref                    3.3f             // rk
//      quantum_k                       =               q     *      k
#define quantum_k               0.12936907          //(3.3/4096 * 754.4/4.4); // =(quantum * k) with k the coefficient of dividing bridge
#define quantum_k2              0.02827148
#define AC_FREQ                 50.0
#define INSTRU_BUFFER_SIZE      200

#define PI_DEFAULTS { 1.0f, 0.0f, 0.0f, 1.0f, -1.0f, 1.0f, 1.0f, -1.0f, \
                      NULL_ADDR, NULL_ADDR }


// 12 for 12-bit conversion resolution, which support (ADC_MODE_SINGLE_ENDED)
// Sample on single pin with VREFLO
// Or 16 for 16-bit conversion resolution, which support (ADC_MODE_DIFFERENTIAL)
// Sample on pair of pins
//
// Globals
//
uint16_t                adcAResults[RESULTS_BUFFER_SIZE];   // Buffer for results
uint16_t                index;                              // Index into result buffer
volatile uint16_t       bufferFull;                         // Flag to indicate buffer is full
DCL_PI                  PI_Gan;
volatile float          uk          = 0.0f;
volatile float          Vs;                                 // = yk
volatile unsigned int   x;
float32_t      uk_tab[15];
float32_t      vs_tab[15];
float32_t      v1_tab[15];
float32_t      v5_tab[15];
float32_t      v4_tab[15];
float32_t      v6_tab[15];
float32_t      v8_tab[15];
float32_t      v2_tab[15];
float32_t      i6_tab[15];
int            i = 0;


//variable datalog
DLOG_4CH dLog1;
float dVal1, dVal2, dVal3, dVal4;
float dBuff1[INSTRU_BUFFER_SIZE], dBuff2[INSTRU_BUFFER_SIZE], dBuff3[INSTRU_BUFFER_SIZE], dBuff4[INSTRU_BUFFER_SIZE];


//
// Function Prototypes
//
void initGPIOepwm(void);
void initADC(void);
void initEPWM(void);
void initADCSOC(void);
void initPI(DCL_PI *pi);
void setup_datalog(void);
void InstrumentationCode(void);
//float myPI(DCL_PI *pi, float Vref, float Vs);
__interrupt void adcA1ISR(void);

//
// Main
//
void main(void)
{
    //
    // Initialize device clock and peripherals
    //
    Device_init();

    //
    // Disable pin locks and enable internal pullups.
    //
    Device_initGPIO();

    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    Interrupt_initModule();

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    Interrupt_initVectorTable();

    //
    // Interrupts that are used in this example are re-mapped to ISR functions
    // found within this file.
    //
    Interrupt_register(INT_ADCA1, &adcA1ISR);

    //
    // Set up the ADC and the ePWM and initialize the SOC
    //
    initGPIOepwm(); // put this line after the function Device_initGPIO()
    initADC();
    initEPWM();
    initADCSOC();
    initPI(&PI_Gan);
    DLOG_4CH_init(&dLog1);
    setup_datalog();
    //
    // Initialize results buffer
    //
    for(index = 0; index < RESULTS_BUFFER_SIZE; index++)
    {
        adcAResults[index] = 0;
    }

    index = 0;
    bufferFull = 0;

    //
    // Enable ADC interrupt
    //
    Interrupt_enable(INT_ADCA1);

    //
    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    //
    EINT;
    ERTM;
    //
    // Loop indefinitely
    //
    i=0;
    while(1)
    {
        while(bufferFull == 0)
        {
        }
        bufferFull = 0;     // Clear the buffer full flag
    }
}


void initPI(DCL_PI *pi)
{
    pi -> Kp    =   -0.046f   ;     //!< Proportional gain
    pi -> Ki    =   1.57f    ;     //!< Integral gain
    pi -> i10   =   0.0f         ;     //!< I storage
    pi -> Umax  =   0.6f         ;     //!< Upper control saturation limit
    pi -> Umin  =   0.05f        ;     //!< Lower control saturation limit
    pi -> i6    =   1.0f         ;     //!< Saturation storage
    pi -> Imax  =   0.0f         ;     //!< Upper integrator saturation limit
    pi -> Imin  =   0.0f         ;     //!< Lower integrator saturation limit
    pi -> sps   =   NULL_ADDR    ;     //!< Pointer to the shadow parameter set
    pi -> css   =   NULL_ADDR    ;     //!< Pointer to the common support structure
}

//
// Function to configure and power up ADCA.
//
void initADC(void)
{
    //
    // Set ADCCLK divider to /4
    //
    ADC_setPrescaler(ADCA_BASE, ADC_CLK_DIV_4_0);

    //
    // Set resolution and signal mode (see #defines above) and load
    // corresponding trims.
    //
#if(EX_ADC_RESOLUTION == 12)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_12BIT, ADC_MODE_SINGLE_ENDED);
#elif(EX_ADC_RESOLUTION == 16)
    ADC_setMode(ADCA_BASE, ADC_RESOLUTION_16BIT, ADC_MODE_DIFFERENTIAL);
#endif
    //
    // Set pulse positions to late
    //
    ADC_setInterruptPulseMode(ADCA_BASE, ADC_PULSE_END_OF_CONV);

    //
    // Power up the ADC and then delay for 1 ms
    //
    ADC_enableConverter(ADCA_BASE);
    DEVICE_DELAY_US(1000);
}

//
// Function to configure ePWM1 to generate the SOC.
//
void initEPWM(void)
{
    //
    // Disable SOCA
    //
    EPWM_setClockPrescaler(EPWM1_BASE, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
    EPWM_disableADCTrigger(EPWM1_BASE, EPWM_SOC_A);

    //
    // Configure the SOC to occur on the first down-count event
    //

    EPWM_setADCTriggerSource(EPWM1_BASE, EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD);
    EPWM_setADCTriggerEventPrescale(EPWM1_BASE, EPWM_SOC_A, 1);

    //
    // Set the compare A value to 666 (0x29A) and the period to 333 (0x14D)
    //
    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 0x29A);
    EPWM_setTimeBasePeriod(EPWM1_BASE, 0x29A);

    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    EPWM_setActionQualifierAction(EPWM1_BASE, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);

    //EPWM_getTimeBasePeriod(uint32_t base) renvoi la valeur du compteur
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    GPIO_setPinConfig(GPIO_0_EPWM1A);

    //
    // Freeze the counter
    //
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_STOP_FREEZE);
    EPWM_enableADCTrigger(EPWM1_BASE, EPWM_SOC_A);
    EPWM_setTimeBaseCounterMode(EPWM1_BASE, EPWM_COUNTER_MODE_UP_DOWN);
}

//
// Init the GPIO output to see the pwm signal on the GPIO0
//
void initGPIOepwm(void)
{
    GPIO_setPadConfig(0, GPIO_PIN_TYPE_STD);
    //GPIO_setPinConfig(GPIO_0_EPWM1A);

}

//
// Function to configure ADCA's SOC0 to be triggered by ePWM1.
//
void initADCSOC(void)
{
    //
    // Configure SOC0 of ADCA to convert pin A0. The EPWM1SOCA signal will be
    // the trigger.
    //
    // For 12-bit resolution, a sampling window of 15 (75 ns at a 200MHz
    // SYSCLK rate) will be used.  For 16-bit resolution, a sampling window of
    // 64 (320 ns at a 200MHz SYSCLK rate) will be used.
    //
#if(EX_ADC_RESOLUTION == 12)
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN0, 15);
#elif(EX_ADC_RESOLUTION == 16)
       ADC_setupSOC(ADCA_BASE, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM1_SOCA,
                    ADC_CH_ADCIN0, 64);
#endif

    //
    // Set SOC0 to set the interrupt 1 flag. Enable the interrupt and make
    // sure its flag is cleared.
    //
    ADC_setInterruptSource(ADCA_BASE, ADC_INT_NUMBER1, ADC_SOC_NUMBER0);
    ADC_enableInterrupt(ADCA_BASE, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
}

void setup_datalog()
{
    dLog1.input_ptr1 = &dVal1;
    dLog1.input_ptr2 = &dVal2;
    dLog1.input_ptr3 = &dVal3;
    dLog1.input_ptr4 = &dVal4;
    dLog1.output_ptr1 = dBuff1;
    dLog1.output_ptr2 = dBuff2;
    dLog1.output_ptr3 = dBuff3;
    dLog1.output_ptr4 = dBuff4;
    dLog1.size = INSTRU_BUFFER_SIZE;
    dLog1.pre_scalar = 1;
    dLog1.trig_value = (float)(1.0f);
    dLog1.status = 1;
}

void InstrumentationCode(void)
{
    dVal1 = Vs;
    dVal2 = uk;
    dVal3 = i;
    dVal4 = Vs;
    DLOG_4CH_run(&dLog1);
}

/*float myPI(DCL_PI *pi, float Vref, float Vs)
{
    float erreur, erreur_p, v5;

    erreur = Vref-Vs;
    v5 = erreur * ( p->Kp + )
}
*/
//
// ADC A Interrupt 1 ISR
//
__interrupt void adcA1ISR(void)
{
    //
    // Add the latest result to the buffer
    //
    adcAResults[index++] = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);

    //
    // Set the bufferFull flag if the buffer is full
    //
    if(RESULTS_BUFFER_SIZE <= index)
    {
        index = 0;
        bufferFull = 1;
    }

    //
    // Clear the interrupt flag
    //
    ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);

    //
    // Check if overflow has occurred
    //
    if(true == ADC_getInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1))
    {
        ADC_clearInterruptOverflowStatus(ADCA_BASE, ADC_INT_NUMBER1);
        ADC_clearInterruptStatus(ADCA_BASE, ADC_INT_NUMBER1);
    }

    //
    // Acknowledge the interrupt
    //
    Interrupt_clearACKGroup(INTERRUPT_ACK_GROUP1);

    // Vana =      R2      / (    R1     +    R2     ) x Vs
    // Vana = (4.7 x 10^3) / (750 x 10^3 + 4.7 x 10^3) x Vs
    //Vs      = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0) * quantum_k;
    x = ADC_readResult(ADCARESULT_BASE, ADC_SOC_NUMBER0);
    Vs      = x * quantum_k2;

    // float32_t DCL_runPI_C3(DCL_PI *p, float32_t rk, float32_t yk)

    //uk      = DCL_runPI_C2(&PI_Gan, Vref, Vs);
    //uk      = DCL_runPI_C2(&PI_Gan, Vref, Vs,v1_tab,v4_tab,v5_tab,v6_tab,v8_tab,i6_tab,v2_tab,i);
    uk      = DCL_runPI_C3(&PI_Gan, Vref, Vs);

    if(i<15){
        vs_tab[i]=Vs;
        uk_tab[i]=uk;
        i++;
        i=i%15;
    }

    EPWM_setCounterCompareValue(EPWM1_BASE, EPWM_COUNTER_COMPARE_A, 666 * (uk));
    //GPIO0 qui est la broche 49 de la carte sort le signal carré de la pwm

    InstrumentationCode(); //datalog affichage des signaux en temps réel
}
