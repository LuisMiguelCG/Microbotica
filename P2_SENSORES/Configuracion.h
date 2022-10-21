
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
//#include "utils/uartstdioMod.h"     // TIVA: Funciones API UARTSTDIO (printf)
#include "drivers/buttons.h"     // TIVA: Funciones API manejo de botones
#include "drivers/rgb.h"        // TIVA: Funciones API manejo de RGB
#include "driverlib/pwm.h"       // TIVA: Funciones API manejo PWM
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de Timers


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


//Sensores (Puerto B)
#define SENSOR_CONTAC           GPIO_PIN_2
#define ENCODER_IZQ             GPIO_PIN_3
#define ENCODER_DRC             GPIO_PIN_4
#define SENSOR_BAJO             GPIO_PIN_5

//Estados de los motores
#define PARADA              0
#define AVANCE              1
#define BUSQUEDA            2
#define FUERA               3
#define AVANCE_OR           4
#define CHOQUE              5

//Estados del sensor de distancia
#define SENSORM             7
#define SENSORL             8



void Configuracion(void){

    /*---------------------Configuracion de los botones de la placa-------------------*/

    ButtonsInit();
    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
    GPIOIntTypeSet(GPIO_PORTF_BASE, ALL_BUTTONS,GPIO_FALLING_EDGE);     // Interrupt de ambos por flanco de bajada
    IntPrioritySet(INT_GPIOF,configMAX_SYSCALL_INTERRUPT_PRIORITY);     // Da prioridad a la interrup (5)
    GPIOIntEnable(GPIO_PORTF_BASE,ALL_BUTTONS);                         // Habilitada la interrupt en ambos botones
    IntEnable(INT_GPIOF);                                               // Habilita interruciones del puerto F



    /*---------------------Configuracion del Puerto B para sensores-------------------*/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);                                            // Habilita el puerto B
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);                                       // Habilita el puerto B en bajo consumo
    GPIODirModeSet(GPIO_PORTB_BASE, SENSOR_BAJO|SENSOR_CONTAC, GPIO_DIR_MODE_IN);           // Los ponemos como entrada
    IntPrioritySet(INT_GPIOB, configMAX_SYSCALL_INTERRUPT_PRIORITY);                        // Da prioridad a la interrupcion
    IntEnable(INT_GPIOB);                                                                   // Habilita la interrupcion

    /*---------------------------Sensor de contacto-----------------------------------*/

    GPIOPadConfigSet(GPIO_PORTB_BASE,SENSOR_CONTAC, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);     // Ponemos una resistencia de pull-up
    GPIOIntTypeSet(GPIO_PORTB_BASE, SENSOR_CONTAC, GPIO_FALLING_EDGE);                             // Int por flanco de bajada
    GPIOIntEnable(GPIO_PORTB_BASE, SENSOR_CONTAC);                                                 // Habilitar Int del pin
    GPIOIntClear(GPIO_PORTB_BASE, SENSOR_CONTAC);                                                  // Limpia el flag de Int

    /*----------------------------Sensor Bajo-----------------------------------------*/

    GPIODirModeSet(GPIO_PORTB_BASE, SENSOR_BAJO, GPIO_DIR_MODE_IN);                             // Pone en pin como entrada
    GPIOPadConfigSet(GPIO_PORTB_BASE, SENSOR_BAJO, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);        //
    GPIOIntTypeSet(GPIO_PORTB_BASE, SENSOR_BAJO, GPIO_FALLING_EDGE);                            // Int cuando tiene un nivel bajo
    GPIOIntEnable(GPIO_PORTB_BASE, SENSOR_BAJO);                                                // Habilita la interrupcion en el pin
    GPIOIntClear(GPIO_PORTB_BASE, SENSOR_BAJO);                                                 // Limpia flag de interrupcion del pin

    /*-----------------------------Encoders-------------------------------------------*/


    /*----------------------Configuracion de los servos-------------------------------*/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);                    // Habilita puerto salida para señal PWM
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);                     // Habilita modulo PWM
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);                            // Establece divisor del reloj del sistema (40MHz/16=2.5MHz)

    GPIOPinConfigure(GPIO_PF2_M1PWM6);                              // Configura el pin PF2 como  PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);

    GPIOPinConfigure(GPIO_PF3_M1PWM7);                              // Configura el pin PF3 como  PWM
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

                                                                    // Configura el modo de funcionamiento del PWM
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);       // Módulo PWM contara hacia abajo
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT, true);                 // Habilita la salida de la señal
    PWMOutputState(PWM1_BASE, PWM_OUT_7_BIT, true);                 // Habilita la salida de la señal
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, PERIOD_PWM);              // Carga la cuenta que establece la frecuencia de la señal PWM

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, PARADA_DRC);             // Carga valor al servo derecho
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, PARADA_IZQ);             // Carga valor al servo izquierdo

    PWMGenEnable(PWM1_BASE, PWM_GEN_3);                             // Habilita el generador PWM 3


    /*-----------------------Configuracion Timer 0(ADC)----------------------------------*/

    uint32_t ui32PeriodADC;

     SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                  //Habilita periferico Timer0
     TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);               //Configura el Timer0 para cuenta periodica de 32 bits
     ui32PeriodADC = SysCtlClockGet()*0.1;                          //Periodo de cuenta de 0.1s.
     TimerLoadSet(TIMER0_BASE, TIMER_A, ui32PeriodADC -1);          //Carga la cuenta en el Timer0A
     IntEnable(INT_TIMER0A);                                        //Habilita interrupcion del modulo TIMER
     TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);               //Habilita la interrupcion de "fin de cuenta"
     IntMasterEnable();                                             //Habilita permiso general de interrupciones el sistema.
//   TimerEnable(TIMER0_BASE, TIMER_A);                             //Activa el Timer0A (empezara a funcionar)
     TimerDisable(TIMER0_BASE, TIMER_A);                            //Desabilita el Timer0A

     /*---------------------Configuracion Timer2(????)----------------------------------*/

     uint32_t ui32PeriodXX;

      SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);                 // Habilita periferico Timer2
      TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);              // Configura el Timer2 para cuenta periodica de 32 bits
      ui32PeriodXX = SysCtlClockGet();                              // Periodo de cuenta de 1s.
      TimerLoadSet(TIMER2_BASE, TIMER_A, ui32PeriodXX -1);          // Carga la cuenta en el Timer2A
      IntEnable(INT_TIMER2A);                                       // Habilita interrupcion del modulo TIMER
      TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT);              // Habilita la interrupcion de "fin de cuenta"
      IntMasterEnable();                                            // Habilita permiso general de interrupciones el sistema.
//      TimerEnable(TIMER2_BASE, TIMER_A);                            // Activa el Timer2A (empezara a funcionar)
      TimerDisable(TIMER2_BASE, TIMER_A);                           // Desabilita el Timer2A


}



#endif /* CONFIGURACION_H_ */
