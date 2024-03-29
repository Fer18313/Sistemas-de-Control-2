/*
 *  PROGRAMACION DE LABORATORIO 6
 *
 *  FERNANDO SANDOVAL
 *  JULIO LOPEZ
 *
 *  SECCION 21
 */
//*****************************************************************************
//
// invert.c - Example demonstrating the PWM invert function.
//
// Copyright (c) 2010-2017 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
//
//   Redistribution and use in source and binary forms, with or without
//   modification, are permitted provided that the following conditions
//   are met:
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
//
// This is part of revision 2.1.4.178 of the Tiva Firmware Development Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "driverlib/pwm.h"


#define NUM_SPI_DATA    1
#define SPI_FREC  4000000
#define SPI_ANCHO      16

float v0 = 0.0, v1 = 0.0, v2= 0.0, v3= 0.0, v4= 0.0, Nbar, Control; // Floats
uint16_t valor_int;
uint16_t dato;
uint16_t freq_timer = 10000;    // Timer frequency
float Ts = 0.0001;     // Sample Time


//*****************************************************************************

// ADC var declaration
uint32_t s = 0;
uint32_t pui32ADC0Value[2];

// PWM control var declaration
int modo = 1; // switches case of PWM
uint16_t duty = 0;
float y = 0;
float u, Ref;
float temp = 0;


//***************************
// VARIABLES DE CONTROL LQR
//***************************

// A_obs Matrix
float alc00 = 0;
float alc01 = 1.0e04*-0.0100;
float alc10 = 1.0e04*1.0000;
float alc11 = 1.0e04*-0.0031;

// B_obs Matrix
float b0 = 330.0;
float b1 = 0;

// L_obs Matrix
float L0 = 0.0008;
float L1 = 0.2561;

// K control
float K[2]= {3.3529, 0.1829};

float u = 0;
float uss = 0.6061;


// X vector
float x[2] = {0.0, 0.0};
float x_k1[2];
float xss[2] = {0.0061, 2.0}; // Point of interest



// ********************
//          ADC Config
// *********************
void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, 2);
    while(!ADCIntStatus(ADC0_BASE, 2, false))
    {
    }
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, pui32ADC0Value);
 // Reads the output of plant
    y = (float)(pui32ADC0Value[0]*3.3/4095.0); // Output of plant conversion
    // ****************CONTROL*******************

 // Control signal calculation
    u = K[0]*(xss[0]-x[0]) + K[1]*(xss[1]-x[1]) + uss;

  // Show control signal on temp
    temp = u;

 // Refresh
    x_k1[0] = x[0] + Ts*(alc00*x[0] + alc01*x[1] + b0*u + L0*y); // All we need is the first vector
    //x_k1[1] = x[1] + Ts*(alc10*x[0] + alc11*x[1] + b1*u + L1*y);

    x[0] = 0.001*x_k1[0];
    //x[1] = x_k1[1];
    x[1] = y;

 // Delimit control signal
    if(u > 1.0)
        u = 1.0;
    if(u < 0.15)
        u = 0.15;

    switch(modo){
    case 0:
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,  (uint32_t)(SysCtlClockGet()/50000*0.2)); // 80% DUTY CYCLE
        break;
    case 1:
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0,  (uint32_t)(SysCtlClockGet()/50000*u)); // Control gives duty cycle
        break;
    }



}


//*****************************************************************************
//
// Configure PWM0 for a 25% duty cycle signal running at 250Hz.  This example
// also shows how to invert the PWM signal every 5 seconds for 5 seconds.
//
//*****************************************************************************
int
main(void)
{
    // ADC CONFIG
    // Set the clocking to run at 80 MHz (200 MHz / 2.5) using the PLL.  When
    // using the ADC, you must either use the PLL or supply a 16 MHz clock source.
    // TODO: The SYSCTL_XTAL_ value must be changed to match the value of the
    // crystal on your board.
    //SysCtlClockSet(SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    //                 SYSCTL_XTAL_16MHZ); // 20 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 80 MHz

    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for ADC operation

    // The ADC0 peripheral must be enabled for use.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);         // Activates ADC peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);        // E3 ch 0
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                               //Activar perifericos timer0
    IntMasterEnable();                                                          //Activar las interrupciones
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);                            //Configurar el timer
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_timer));
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);                            //Activamos las interrupciones por timer
    TimerEnable(TIMER0_BASE, TIMER_A);                                          //Activar el timer


    // Configuracion del PWM
    SysCtlPeripheralDisable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralDisable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralReset(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralReset(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0) || !SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB))
    {
    }
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinTypePWM(GPIO_PORTB_BASE,GPIO_PIN_6);

    //Configuracion del PWM calculo para 50khz, 1/50000 = 0.0005; en 80Mhz, esto representa (1/50000)/(1/80000000)= 1600
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, (uint32_t)(SysCtlClockGet()/50000));
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, (uint32_t)(SysCtlClockGet()/50000*u));

    PWMGenEnable(PWM0_BASE, PWM_GEN_0);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);


    while(1)
    {
   // increible
    }
}
