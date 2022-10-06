/*
 * Configuracion.h
 *
 *  Created on: 28 oct. 2021
 *      Author: Luis Miguel Campos Garcia
 */

#ifndef CONFIGURACION_H_
#define CONFIGURACION_H_

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"       // TIVA: Definiciones del mapa de memoria
#include "inc/hw_types.h"        // TIVA: Definiciones API
#include "inc/hw_ints.h"         // TIVA: Definiciones para configuracion de interrupciones
#include "driverlib/gpio.h"      // TIVA: Funciones API de GPIO
#include "driverlib/pin_map.h"   // TIVA: Mapa de pines del chip
#include "driverlib/rom.h"       // TIVA: Funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/rom_map.h"   // TIVA: Mapeo automatico de funciones API incluidas en ROM de micro (MAP_)
#include "driverlib/sysctl.h"    // TIVA: Funciones API control del sistema
#include "driverlib/uart.h"      // TIVA: Funciones API manejo UART
#include "driverlib/interrupt.h" // TIVA: Funciones API manejo de interrupciones
#include "utils/uartstdioMod.h"     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"        // TIVA: Funciones API manejo de RGB
#include "driverlib/pwm.h"       // TIVA: Funciones API manejo PWM
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de Timers


#define PERIOD_PWM 12500

#define COUNT_ADELANTE_DRC      750       //AJUSTAR ENCODER
#define COUNT_ATRAS_DRC         1065       // AJUSTAR ENCODER
#define STOPCOUNT_DRC_MIN       930
#define STOPCOUNT_DRC_MAX       975       // 990 REAL
#define NUM_STEPS_DRC_ADELANTE  0.2*(STOPCOUNT_DRC_MIN - COUNT_ADELANTE_DRC)    // Pasos/pulsacion boton
#define NUM_STEPS_DRC_ATRAS     0.2*(COUNT_ATRAS_DRC - STOPCOUNT_DRC_MAX)
#define V_MEDIA_ADELANTE_DRC    (STOPCOUNT_DRC_MIN - 0.5*(STOPCOUNT_DRC_MIN - COUNT_ADELANTE_DRC))+25
#define V_MEDIA_ATRAS_DRC       STOPCOUNT_DRC_MAX + 0.5*(COUNT_ATRAS_DRC - STOPCOUNT_DRC_MAX)
#define COUNT_ADELANTE_IZQ      1095      // 1025 REAL  AJUSTAR ENCODER
#define COUNT_ATRAS_IZQ         710       // AJUSTAR ENCODER
#define STOPCOUNT_IZQ_MIN       945
#define STOPCOUNT_IZQ_MAX       985
#define NUM_STEPS_IZQ_ATRAS     0.2*(STOPCOUNT_IZQ_MIN - COUNT_ATRAS_IZQ)
#define NUM_STEPS_IZQ_ADELANTE  0.2*(COUNT_ADELANTE_IZQ - STOPCOUNT_IZQ_MAX)
#define V_MEDIA_ADELANTE_IZQ    STOPCOUNT_IZQ_MAX + 0.5*(COUNT_ADELANTE_IZQ - STOPCOUNT_IZQ_MAX)
#define V_MEDIA_ATRAS_IZQ       (STOPCOUNT_IZQ_MIN - 0.5*(STOPCOUNT_IZQ_MIN - COUNT_ATRAS_IZQ))+45

#define SENSOR_FRONTAL          GPIO_PIN_2
#define ENCODER_IZQ             GPIO_PIN_3
#define ENCODER_DRC             GPIO_PIN_4


#define PARADA              0
#define AVANCE              1
#define BUSCAR_REFERENCIA   2
#define BUSCAR_OBJETO       3
#define FUERA               4
#define GIRO_DERECHA        5
#define GIRO_IZQUIERDA      6


#define CUATRO_CM           1570
#define SEIS_CM             1191
#define OCHO_CM             960
#define DIEZ_CM             810
#define DOCE_CM             740
#define CATORCE_CM          650
#define DIECISEIS_CM        580
#define DIECIOCHO_CM        530
#define VEINTE_CM           515
#define VEINTIDOS_CM        475
#define VEINTICUATRO_CM     430
#define VEINTISEIS_CM       424
#define VEINTIOCHO_CM       400
#define TREINTA_CM          385



void ConfigButtons(void){

    ButtonsInit();
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
    GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_FALLING_EDGE);     //Interrupt de ambos por flanco de bajada
    IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);     //Da prioridad a la interrup (5)
    GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);                         //Habilitada la interrupt en ambos botones
    IntEnable(INT_GPIOF);                                               //Habilita interruciones del puerto F

}

void ConfigEncSFrontal(void){

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIODirModeSet(GPIO_PORTB_BASE, SENSOR_FRONTAL|ENCODER_IZQ|ENCODER_DRC , GPIO_DIR_MODE_IN);    //Los ponemos como entrada

    //Sensor Frontal
    GPIOPadConfigSet(GPIO_PORTB_BASE,SENSOR_FRONTAL, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);     //ponemos una resistencia de pull-up
    GPIOIntTypeSet(GPIO_PORTB_BASE, SENSOR_FRONTAL, GPIO_FALLING_EDGE);                             //Int por flanco de bajada
    GPIOIntEnable(GPIO_PORTB_BASE, SENSOR_FRONTAL);
    GPIOIntClear(GPIO_PORTB_BASE, SENSOR_FRONTAL);

    //Encoders
    GPIOIntTypeSet(GPIO_PORTB_BASE, ENCODER_IZQ|ENCODER_DRC, GPIO_HIGH_LEVEL);                             //Int cuando tiene un nivel alto
    GPIOIntEnable(GPIO_PORTB_BASE, ENCODER_IZQ|ENCODER_DRC);
    GPIOIntClear(GPIO_PORTB_BASE, ENCODER_IZQ|ENCODER_DRC);


    IntPrioritySet(INT_GPIOB, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_GPIOB);

}


void ConfigServos(void){

    // Habilita puerto salida para señal PWM (ver en documentacion que pin se corresponde a cada módulo PWM)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    //Habilita modulo PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    // Establece divisor del reloj del sistema (40MHz/64=625KHz)
    SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    // Configura el pin PF2 como  PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    // Configura el pin PF2 como  PWM
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);
    //Configura el modo de funcionamiento del PWM
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);   // Módulo PWM contara hacia abajo
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);             // Habilita la salida de la señal
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);             // Habilita la salida de la señal
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);            // Carga la cuenta que establece la frecuencia de la señal PWM
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DRC_MIN);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ_MAX);

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
     TimerEnable(TIMER0_BASE, TIMER_A);
//     TimerDisable(TIMER0_BASE, TIMER_A);
}





#endif /* CONFIGURACION_H_ */
