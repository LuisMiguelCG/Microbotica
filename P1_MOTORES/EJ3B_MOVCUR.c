// Luis Miguel Campos Garcia

/* **************************************************************************   */
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h" // NHR: Libreria GPIO
#include "driverlib/pwm.h"  // NHR: Libreria PWM
#include "drivers/buttons.h" // NHR
#include "driverlib/timer.h" // Definicion de funciones de Timer (Timer..())
#include "driverlib/interrupt.h" // NHR
#include "inc/hw_ints.h" // NHR


#define PERIOD_PWM 50000    // TODO: Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms
#define CYCLE_INCREMENTS (abs(ADELANTE_DRC-ADELANTE_IZQ))/NUM_STEPS  // Variacion de amplitud tras pulsacion


//Rueda Derecha       PWM_OUT_6
#define ADELANTE_DRC            3550                //2500
#define ATRAS_DRC               4100                //5000
#define PARADA_DRC              3850
#define V_MEDIA_DRC     (PARADA_DRC - (PARADA_DRC - ADELANTE_DRC)*0.5)

//Rueda Izquierda     PWM_OUT_7
#define ADELANTE_IZQ            4100                //5000
#define ATRAS_IZQ               3550                //2500
#define PARADA_IZQ              3850
#define V_MEDIA_IZQ     (PARADA_IZQ + (ADELANTE_IZQ - PARADA_IZQ)*0.5)

#define PARADA 0
#define AVANCE 1
#define RETROCESO 2

uint16_t  ESTADO;

void ConfigButtons(void);
void ConfigServos(void);
void ConfigTimer(void);


int main(void){

    // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable
    SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);  //200MHz/5 = 40MHz

    ConfigButtons();
    ConfigServos();
    ConfigTimer();


  // Codigo principal, (poner en bucle infinito o bajo consumo)
    while(1);
}

void ConfigButtons(void){

    ButtonsInit();
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
    GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_FALLING_EDGE);     //Interrupt de ambos por flanco de bajada
    GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);                         //Habilitada la interrupt en ambos botones
    IntEnable(INT_GPIOF);                                               //Habilita interruciones del puerto F

}

void ConfigServos(void){

    // Habilita puerto salida para señal PWM (ver en documentacion que pin se corresponde a cada módulo PWM)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    // Establece divisor del reloj del sistema (40MHz/16=2.5MHz)
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    // Configura el pin PF2 como  PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    // Configura el pin PF3 como  PWM
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    //Configura el modo de funcionamiento del PWM
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);             // Habilita la salida de la señal
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);             // Habilita la salida de la señal
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);            // Carga la cuenta que establece la frecuencia de la señal PWM
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, V_MEDIA_DRC);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, V_MEDIA_IZQ);
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

}

void ConfigTimer(void){

    uint32_t ui32Period;

    // Habilita periferico Timer0
     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
     // Configura el Timer0 para cuenta periodica de 32 bits (no lo separa en TIMER0A y TIMER0B)
     TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
     // Periodo de cuenta de 1s.
     ui32Period = SysCtlClockGet();
     // Carga la cuenta en el Timer0A
     TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
     // Habilita interrupcion del modulo TIMER
     IntEnable(INT_TIMER0A);
     // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta"
     TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
     // Habilita permiso general de interrupciones el sistema.
     IntMasterEnable();
     // Activa el Timer0A (empezara a funcionar)
     //TimerEnable(TIMER0_BASE, TIMER_A);
     TimerDisable(TIMER0_BASE, TIMER_A);
}


// Rutinas de interrupción de pulsadores
// Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF3, hasta llegar a  COUNT_1MS
// Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF3, hasta llegar a COUNT_2MS
void GPIOFIntHandler(void)
{
    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);
    uint32_t ui32StatusPWM2;

    // Boton Derecho: aumenta ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a MAXCOUNT
    if(((i32Status & RIGHT_BUTTON) == RIGHT_BUTTON)){

        ui32StatusPWM2 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_7);

        if(ui32StatusPWM2 < ADELANTE_IZQ)
        {

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32StatusPWM2 + CYCLE_INCREMENTS);
        }
    }
    // Boton Izquierdo: avance hacia adelante
    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){

        ui32StatusPWM2 = PWMPulseWidthGet(PWM1_BASE, PWM_OUT_7);

        if(ui32StatusPWM2 > PARADA_IZQ + CYCLE_INCREMENTS){

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32StatusPWM2 - CYCLE_INCREMENTS);
        }
    }

    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);  //limpiamos flags
}

void Timer0IntHandler(void){}

