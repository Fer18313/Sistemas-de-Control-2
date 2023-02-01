
/*
 * LAB 2 CONTROL 2
 */

/*
 *  MODIFICADO POR: FERNANDO SANDOVAL
 */

/*
 * diente_de_sierra_spi.c
 * Programa que muestra el uso del SPI en la Tiva C. Se usa un Timer para incre-
 * mentar un contador, que se envía por SPI a un DAC (MCP4921). El resultado
 * es una señal de diente de sierra.
 *
 * Basado en los ejemplos spi_master.c y timers.c de TivaWare
 * Modificado por Luis Alberto Rivera
 */


//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/adc.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "driverlib/ssi.h"

//*****************************************************************************
// Definiciones para configuración del SPI y variable global
//*****************************************************************************
#define NUM_SPI_DATA    1  // Número de palabras que se envían cada vez
#define VALOR_MAX    4095  // Valor máximo del contador
#define SPI_FREC  4000000  // Frecuencia para el reloj del SPI
#define SPI_ANCHO      16  // Número de bits que se envían cada vez, entre 4 y 16
#define FREQ_TIMER  10000

uint16_t dato =  0b0111000000000000;
float rk, yk;
float ek_1 = 0;
float ek = 0;
float Ek_1 = 0;
float ed = 0;
float Ek = 0;
int Uk_int = 0;
float kp = 10.0;
float ki = 0.0;
float kd = 0.0;
float Ek_2 = 0.0;

float Uk = 0.0, Uk_1 = 0.0, Uk_2 = 0.0;
float r = 1.0, sysOutVol = 0, refVol = 0;

float b0 = 1, b1 = 1.037, b2 = -0.03711 ;
float a0 = 21.26, a1 = 39.76, a2 = 18.59;
uint16_t f = 1000; // Frecuencia de muestreo

uint16_t sysOutVal = 0, refVal = 0, ref = 1, DACval = 0;

int uk_int = 0;
float uk =0;
const float cota_sup =6.0;
const float cota_inf =-6.0;

//*****************************************************************************
// The interrupt handler for the timer interrupt.
//*****************************************************************************
void
Timer0IntHandler(void)
{
    uint32_t pui32DataTx[NUM_SPI_DATA]; // la función put pide tipo uint32_t
    uint8_t ui32Index;
    uint32_t pui32ADC0Value[2];
    // Clear the timer interrupt. Necesario para lanzar la próxima interrupción.
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE,2);
    while(!ADCIntStatus(ADC0_BASE,2,false))
    {
    }
    ADCIntClear(ADC0_BASE,2);
    ADCSequenceDataGet(ADC0_BASE,2,pui32ADC0Value);
    rk = pui32ADC0Value[0]*3.3/4095.0;
    yk = pui32ADC0Value[1]*3.3/4095.0;

    pui32DataTx[0] = (uint32_t)(dato);

    // Send data
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    while(SSIBusy(SSI0_BASE))
    {
    }
    // Obtención de datos del ADC, Conversion a valor de voltaje,
        // Calculo de ecuación de diferencias y reconversion a dato entero.

        Ek = rk - yk;
        Uk = b0*Ek + b1*Ek_1 + b2*Ek_2 - a1*Uk_1 - a2*Uk_2;
        Ek_2 = Ek_1;
        Ek_1 = Ek;
        Uk_2 = Uk_1;
        Uk_1 = Uk;

        if (Uk > cota_sup){
             Uk = cota_sup;
         }

         if (Uk < cota_inf){
             Uk = cota_inf;
         }

         Uk_int = (uint16_t)((Uk-cota_inf)*((4095-0)/(cota_sup-cota_inf)));
         dato = 0b0111000000000000;
         dato = dato + Uk_int;


    //if (uk > 4095.0){
    //uk = 4095.0;
    //}
    //if (uk < 0){
    //uk = 0;
    //}
    //uk_int = (int)uk;
    //dato = 0b0111000000000000;
    //dato = dato + uk_int;
}

int
main(void)
{
    uint32_t pui32residual[NUM_SPI_DATA];
    uint16_t freq_timer = 10000;    // En Hz

// ------ Configuración del reloj ---------------------------------------------
    // Set the clock to run at 80 MHz (200 MHz / 2.5) using the PLL.
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                   SYSCTL_XTAL_16MHZ); // 80 MHz. SÍ FUNCIONA
// ----------------------------------------------------------------------------

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |
                   GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
                       SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);

    SSIEnable(SSI0_BASE);

    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32residual[0]))
    {
    }

    // ADC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE |
                             ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2);
    ADCIntClear(ADC0_BASE, 2);


    // TMR0

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_timer));

    // Setup the interrupt for the timer timeout.
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Enable the timers.
    TimerEnable(TIMER0_BASE, TIMER_A);

    while(1)
    {
    }
}
