/*
 *  PROGRAMACION DE LABORATORIO 6
 *
 *  FERNANDO SANDOVAL
 *  JULIO LOPEZ
 *
 *  SECCION 21
 */

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

/*
 * Declaracion de Variables Temporales
 */
float v0,v1,v2,v3,v4; // para ilustrar el uso devariables tipo float
#define NUM_SPI_DATA 1 // Número de palabras que se envían cadavez
#define SPI_FREC 4000000 // Frecuencia para el reloj del SPI
#define SPI_ANCHO 16 // Número de bits que se envían cada vez,entre 4 y 16
#define frec 10000;


uint16_t dato = 0b0111000000000000; // Para lo que se envía por SPI.
uint16_t contador = 2048; // Para lo que se envía por SPI.

// Declaracion de Variables para PID
float uk_float=0;
float uk2=0;
float ek=0;
float ed=0;
float Ek=0;
float ek_1 = 0;
float Ek_1 = 0;

uint16_t uk_int = 0;
float k1= 0;
float k2= 0;
float k3= 0;
float Nbar =0;
float ref_n = 0;
float vc1_n = 0;
float vc2_n = 0;
float vc3_n = 0;
float vc1 = 0;
float vc2 = 0;
float vc3 = 0;


// VARIABLES CONTROLADOR
float ai = 0;
float aj = 0;
float ak = 0;
float bi = 0;
float bj = 0;
float bk = 0;
float ci = 0;
float cj = 0;
float ck = 0;
//Matriz Xk
float Xk_1 = 0;
float Xk_2 = 0;
float Xk_3 = 0;
//Matriz X_k
float Xk1_1 = 0;
float Xk1_2 = 0;
float Xk1_3 = 0;
//Matriz B
float B1 = 1000;
float B2 = 0;
float B3 = 0;

//Matriz L
float L1 = 0;
float L2 = 0;
float L3 = 0;
//Ts
float Ts = 1.0/frec;


//Variables de Feedback
float u = 0;
float y = 0;
float ref = 0;

int Seleccion = 1; // Selecciona el controlador a aplicar

void Timer0IntHandler(void){
    uint32_t pui32DataTx[NUM_SPI_DATA];
    uint8_t ui32Index;
    uint32_t pui32ADC0Value[5];
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    ADCProcessorTrigger(ADC0_BASE, 0);
    while(!ADCIntStatus(ADC0_BASE, 0, false))
    {

    }
    ADCIntClear(ADC0_BASE, 0); // Limpiamos lainterrupción del ADC0
    ADCSequenceDataGet(ADC0_BASE, 0, pui32ADC0Value);
    v0 = pui32ADC0Value[0]*3.3/4095.0; // Convertir a voltios
    v1 = pui32ADC0Value[1]*3.3/4095.0; // Convertir a voltios
    v2 = pui32ADC0Value[2]*3.3/4095.0; // Convertir a voltios
    v3 = pui32ADC0Value[3]*3.3/4095.0; // Convertir a voltios
    v4 = pui32ADC0Value[4]*3.3/4095.0; // Convertir a voltios
    pui32DataTx[0] = (uint32_t)(dato);
    for(ui32Index = 0; ui32Index < NUM_SPI_DATA ; ui32Index++)
    {
        SSIDataPut(SSI0_BASE, pui32DataTx[ui32Index]);
 }
    while(SSIBusy(SSI0_BASE))
    {

    }
 /*
 * LABORATORIO 7
 */
 /*
  * Definicion de K Nbar
  */

    // PARA POLE PLACEMENT

    // POLO 1.2
    if (Seleccion == 0){
        ref = v0;
        y = v1;
        Nbar = 6.3356;
        k1= 0.244;
        k2= 5.0916;
        k3= -31.2116;
        L1 = 4434;
        L2 = 3244;
        L3 = -2411.7;
        ai = -1100;
        aj = 0;
        ak = -10;
        bi = -4334;
        bj = -3244;
        bk = 2421.7;
        ci = -100;
        cj = -1000;
        ck = -20;
    }
    // POLO 1.12
    if (Seleccion == 1){
        ref = v0;
        y = v1;
        Nbar = 7.7;
        k1= 0.53;
        k2= 6.17;
        k3= -63.34;
        L1 = 5873;
        L2 = 3830;
        L3 = -1754.4;
        ai = -1100;
        aj = 0;
        ak = -10;
        bi = -5773;
        bj = -3830;
        bk = 1764.4;
        ci = -100;
        cj = -1000;
        ck = -20;
    }

    // PARA LA TERCERA PARTE

    // Q Y R IDENTIDAD
    if (Seleccion == 2){
        ref = v0;
        y = v1;
        Nbar = 1.7321;
        k1= 0.4356;
        k2= 0.2964;
        k3= -7.4084;
        L1 = 0.2373;
        L2 = 2.6086;
        L3 = -0.0029;
        ai = -1100;
        aj = 0;
        ak = -10;
        bi = 99.8;
        bj = -2.6;
        bk = 10;
        ci = -100;
        cj = -1000;
        ck = -20;
    }

    // Q Y R MEJORADAS
    if (Seleccion == 3){
        Nbar = 3.3317;
        k1= 0.2258;
        k2= 2.1059;
        k3= -22.3859;
        L1 = 1.6833;
        L2 = 18.4968;
        L3 = -0.1661;
        ref = v0;
        y = v1;
        ai = -1100;
        aj = 0;
        ak = -10;
        bi = 98.3;
        bj = -18.5;
        bk = 10.2;
        ci = -100;
        cj = -1000;
        bk = -20;
    }

 /*
 * Definicion de Controlador
 */
    ref_n = ref*Nbar;
    vc1_n = k1*Xk_1;
    vc2_n = k2*Xk_2;
    vc3_n = k3*Xk_3;
    uk_float = ref_n -(vc1_n + vc2_n + vc3_n);
    u = uk_float;

    Xk1_1 = (ai*Xk_1+bi*Xk_2+ci*Xk_3+B1*u+L1*y)*Ts +Xk_1;
    Xk1_2 = (aj*Xk_1+bj*Xk_2+cj*Xk_3+B2*u+L2*y)*Ts +Xk_2;
    Xk1_3 = (ak*Xk_1+bk*Xk_2+ck*Xk_3+B3*u+L3*y)*Ts +Xk_3;

    Xk_1 = Xk1_1;
    Xk_2 = Xk1_2;
    Xk_3 = Xk1_3;

 // Limites de Uk para que salida este entre el rango
    uk_float=uk_float*4095.0/3.3;
    if(uk_float>4095.0){
        uk_float = 4095.0;
    }
    if(uk_float<0){
        uk_float = 0;
 }
 uk_int = (int)(uk_float);
// Mapeo para salida por SPI para el DAC
 dato = 0b0111000000000000;
 dato = dato + uk_int;
}
int
main(void)
{
    uint32_t pui32residual[NUM_SPI_DATA];
     uint16_t freq_muestreo = 10000; // En Hz
     SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |SYSCTL_XTAL_16MHZ); // 80 MHz
     // Configuración de SPI
     SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
     GPIOPinConfigure(GPIO_PA2_SSI0CLK);
     GPIOPinConfigure(GPIO_PA3_SSI0FSS);
     GPIOPinConfigure(GPIO_PA4_SSI0RX);
     GPIOPinConfigure(GPIO_PA5_SSI0TX);
     GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 |GPIO_PIN_2);
     SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,SSI_MODE_MASTER, SPI_FREC, SPI_ANCHO);
     SSIEnable(SSI0_BASE);




 // Configuración del ADC IMPORTANTE
 SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //Activandopuerto de los AIN0 y ANI1
 SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //Activandopuerto de los AIN0 y ANI1
 GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0); //Configura el pin PE0 como AN-Input
 GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1); //Configura el pin PE1 como AN-Input
 GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); // Configura el pin PE2 como AN-Input
 GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //Configura el pin PE3 como AN-Input
 GPIOPinTypeADC(GPIO_PORTD_BASE, GPIO_PIN_3); //Configura el pin PD3 como AN-Input
 ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0); // Se configura la secuencia 2 para tomar 2 de 4 posibles muestras.
 ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0); // Step 0 en la secuencia 2: PE3 Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
 ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1); // Step 1 en la secuencia 2: PE2 Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
 ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH2); // Step 2 en la secuencia 2: PE1 Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
 ADCSequenceStepConfigure(ADC0_BASE, 0, 3, ADC_CTL_CH3); // Step 3 en la secuencia 2: PE0 Canal 0 (ADC_CTL_CH0) en modo single-ended (por defecto).
 ADCSequenceStepConfigure(ADC0_BASE, 0, 4, ADC_CTL_CH4 | ADC_CTL_IE | ADC_CTL_END); // Step 4 en la secuencia 2: PD3 Canal 1 (ADC_CTL_CH1) en modo single-ended (por defecto),
 ADCSequenceEnable(ADC0_BASE, 0); //Activando secuencia 2 de ADC0
 ADCIntClear(ADC0_BASE, 0); //Limpiamos la bandera de interrupción del ADC0.



 //Configurando Timer 0
 SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
 IntMasterEnable();
 TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
 TimerLoadSet(TIMER0_BASE, TIMER_A, (uint32_t)(SysCtlClockGet()/freq_muestreo));
 IntEnable(INT_TIMER0A);
 TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
 TimerEnable(TIMER0_BASE, TIMER_A);
 while(1)
 {
 }
}
