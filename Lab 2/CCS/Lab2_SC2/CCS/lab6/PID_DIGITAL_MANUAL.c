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
float kp = 10.0;
float ki = 0.015;
float kd = 0.0;
int uk_int = 0;
float uk =0;
const float superior_limit =6.0;
const float inferior_limit =-6.0;


//perfil de movimiento
volatile int inf_cont=0;
volatile int sup_cont=0;
volatile int limit_inf_0 =10;
volatile int limit_max_0 =250;
const uint16_t perfil_pos[250] =
   {   0,    0,    8,   16,   32,   48,   64,   80,  104,  120,
     144,  169,  201,  225,  249,  281,  305,  337,  369,  401,
     433,  458,  490,  530,  562,  594,  626,  658,  690,  730,
     763,  795,  835,  867,  899,  939,  971, 1003, 1044, 1076,
    1108, 1148, 1180, 1212, 1252, 1284, 1317, 1357, 1389, 1421,
    1453, 1485, 1517, 1557, 1589, 1614, 1646, 1678, 1710, 1742,
    1766, 1798, 1822, 1846, 1878, 1903, 1927, 1943, 1967, 1983,
    1999, 2015, 2031, 2039, 2047, 2047, 2047, 2047, 2047, 2047,
    2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
    2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
    2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
    2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047, 2047,
    2047, 2047, 2047, 2047, 2047, 2047, 2047, 2039, 2031, 2023,
    2007, 1991, 1975, 1959, 1935, 1911, 1886, 1862, 1838, 1806,
    1782, 1750, 1718, 1686, 1654, 1622, 1589, 1549, 1517, 1477,
    1445, 1405, 1373, 1333, 1300, 1260, 1220, 1188, 1148, 1108,
    1076, 1036,  995,  963,  923,  891,  851,  819,  779,  747,
     714,  674,  642,  610,  578,  546,  514,  490,  458,  433,
     401,  377,  353,  329,  305,  289,  265,  249,  233,  217,
     201,  185,  177,  161,  153,  136,  128,  120,  104,   96,
      88,   80,   72,   64,   56,   48,   48,   40,   32,   24,
      24,   16,   16,    8,    8,    8,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0,
       0,    0,    0,    0,    0,    0,    0,    0,    0,    0};


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
    yk = pui32ADC0Value[1]*3.3/4095.9;

    pui32DataTx[0] = (uint32_t)(dato);

    // Send data
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA; ui32Index++)
    {
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
    }

    while(SSIBusy(SSI0_BASE))
    {
    }

    // PID
    inf_cont = inf_cont + 1;
    if (inf_cont == limit_inf_0){
        inf_cont = 0;
        sup_cont = sup_cont + 1;
    }
    if (sup_cont == limit_max_0){
        sup_cont = 0;
    }

    ek = rk - yk;
    ed = ek - ek_1;
    Ek = Ek_1 + ek;
    uk = kp*ek + ki*Ek/FREQ_TIMER + (kd*ed)*FREQ_TIMER;
    ek_1 = ek;
    Ek_1 = Ek;
    //uk = uk*4095.0/3.3;

    if (uk > superior_limit){                                    // Upper limit
        uk = superior_limit;
    }
    if (uk < inferior_limit){                                         // Lower limit
        uk = inferior_limit;
    }
    uk_int = (uint16_t)((uk-inferior_limit)*((4095-0)/(superior_limit-inferior_limit)));
    dato = 0b0111000000000000;
    dato = dato + uk_int;


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
    uint16_t freq_muestreo = 1000;                          // Hz


    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //Configurando Timer 0

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    IntMasterEnable();
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_muestreo));
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    // Configuración de SPI

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);
    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
    SSIEnable(SSI0_BASE);

    // Configuración del ADC

    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                        //Activando puerto de los AIN0 y ANI1
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);                        // Configura el pin PE3 como AN-Input
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2);                        // Configura el pin PE2 como AN-Input
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_PROCESSOR, 0);       // Se configura la secuencia 2 para tomar 2 de 4 posibles muestras.
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);             // Step 0 en la secuencia 2: Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);  // Step 1 en la secuencia 2: Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
    ADCSequenceEnable(ADC0_BASE, 2);                                    // Activando secuencia 2 de ADC0
    ADCIntClear(ADC0_BASE, 2);                                          // Limpiamos la bandera de interrupción del ADC0.

    while(1)
    {
    }
}
