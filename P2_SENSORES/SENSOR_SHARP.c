/*
 * main.c
 *
 *  Created on: 28 oct. 2021
 *      Author: Luis Miguel Campos Garcia
 */


//Librerias

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
#include "drivers/rgb.h"         // TIVA: Funciones API manejo de RGB
#include "driverlib/pwm.h"       // TIVA: Funciones API manejo PWM
#include "driverlib/adc.h"       // TIVA: Funciones API manejo del ADC
#include "driverlib/timer.h"     // TIVA: Funciones API manejo de Timers
#include "FreeRTOS.h"            // FreeRTOS: definiciones generales
#include "task.h"                // FreeRTOS: definiciones relacionadas con tareas
#include "semphr.h"              // FreeRTOS: definiciones para uso de semaforos
#include "queue.h"               // FreeRTOS: definiciones para uso de ola de mensajes
#include "Configuracion.h"       // Configuracion de perifericos
#include "drivers/configADC.h"   // Configuracion del adc
#include "commands.h"
#include "utils/cpu_usage.h"
#include "usb_dev_serial.h"

//Defines



// Definiciones de tareas


// Variables globales "main"

uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
uint32_t ESTADO = 0;

//Importante: Los arrays con los parametros tengo que poner como globales !!!
// por que? se pasan por referencia a la funcion TaskCreate,
// como son variables locales automaticas, cuando se llega a ajecutar la tarea
// la tarea la memoria reservada para estas variables ya no existe (el compilador interpreta que ya no se usan tras llamar
// a task create  (falla incluso sin optimizacion y aunque se copien al comenzar la tarea).


//Variables globales tareas e interrupciones

xSemaphoreHandle semaforo_choque;
xSemaphoreHandle semaforo_adc;
QueueHandle_t cola_usb;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// This hook is called by FreeRTOS when an stack overflow error is detected.
//
//*****************************************************************************
void vApplicationStackOverflowHook(TaskHandle_t pxTask, char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
    static uint8_t count = 0;

    if (++count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        count = 0;
    }
    //return;
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void)
{
    SysCtlSleep();
}


//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationMallocFailedHook (void)
{
    while(1);
}

//*****************************************************************************
//
// This task toggles the user selected LED at a user selected frequency.
//
//*****************************************************************************

static portTASK_FUNCTION(ChoqueTask, pvParameters ){

    //xSemaphoreTake(semaforo_choque,0);   //Nos aseguramos de que el semaforo esta cerrado
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);

        //
        // Loop forever.
        //
        while(1)
        {
            xSemaphoreTake(semaforo_choque,portMAX_DELAY); //Semaforo del RTOS

            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_ATRAS_DRC);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_ATRAS_IZQ);


            vTaskDelay(configTICK_RATE_HZ*0.5);


            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, COUNT_ATRAS_DRC);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, COUNT_ADELANTE_IZQ);

            vTaskDelay(configTICK_RATE_HZ*0.5);

            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, V_MEDIA_ADELANTE_DRC);
            PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, V_MEDIA_ADELANTE_IZQ);
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);

//            TimerEnable(TIMER0_BASE, TIMER_A);
            TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        }
}

static portTASK_FUNCTION(ADCTask,pvParameters)
{
    uint16_t Valor_SenM;

    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        xSemaphoreTake(semaforo_adc, portMAX_DELAY);

        configADC_DisparaADC();             //Dispara la conversion (por software)

       configADC_LeeADC(&Valor_SenM);    //Espera y lee muestras del ADC (BLOQUEANTE)


    }
}




//*****************************************************************************
//
// Initialize FreeRTOS and start the initial set of tasks.p
//
//*****************************************************************************
int main(void)
{

    //
    // Set the clocking to run at 40 MHz from the PLL.
    //
    MAP_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
            SYSCTL_OSC_MAIN);


    ConfigButtons();                    //Configuracion de los botones de la placa
    ConfigServos();                     //Configuracion de los motores servos
    ConfigEncSFrontal();                     //Configuracion de los botones frontales para choque
    ConfigTimer();                      //Configuracion TIMER de ADC
    configADC_IniciaADC();              //Configura el adc



    // Get the system clock speed.
    g_ui32SystemClock = SysCtlClockGet();

    //Habilita el clock gating de los perifericos durante el bajo consumo
    ROM_SysCtlPeripheralClockGating(false);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no este dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER3 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    // El timer 0 y 1 se utilizan para los LEDS
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 3);

    //Crea la tarea que maneja el choque
    if(xTaskCreate(ChoqueTask, "Choque", 512, NULL, tskIDLE_PRIORITY + 2, NULL) != pdTRUE){

        while(1);
    }

    //Crea la tarea del ADC
    if((xTaskCreate(ADCTask, (portCHAR *)"ADC", 256, NULL, tskIDLE_PRIORITY + 1, NULL) != pdTRUE)){

        while(1);

    }

    //Crea los semaforos compartidos por tareas e ISR
     semaforo_choque=xSemaphoreCreateBinary();
     if ((semaforo_choque==NULL))
     {
         while (1);  //No hay memoria para los semaforo
     }

     semaforo_adc=xSemaphoreCreateBinary();
      if ((semaforo_adc==NULL))
      {
          while (1);  //No hay memoria para los semaforo
      }

    //
    // Start the scheduler.  This should not return.
    //
    vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

    //
    // In case the scheduler returns for some reason, print an error and loop
    // forever.
    //

    while(1)
    {
    }
}


// Rutinas de interrupcion
void INT_SENSOR_BUTTON(void){


    BaseType_t higherPriorityTaskWoken=pdFALSE;
    int32_t i32Status = GPIOIntStatus(GPIO_PORTB_BASE, ALL_CBUTTON);

    if(((i32Status & ALL_CBUTTON) == RIGHT_CBUTTON) || ((i32Status & ALL_CBUTTON) == LEFT_CBUTTON)){

        TimerIntDisable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

        xSemaphoreGiveFromISR(semaforo_choque,&higherPriorityTaskWoken);

    }

    GPIOIntClear(GPIO_PORTB_BASE, ALL_CBUTTON);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);

}

void STOP_MOVE(void){

    int32_t i32Status = GPIOIntStatus(GPIO_PORTF_BASE,ALL_BUTTONS);

    if(((i32Status & LEFT_BUTTON) == LEFT_BUTTON)){

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, STOPCOUNT_DRC_MIN);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, STOPCOUNT_IZQ_MAX);
    }

    GPIOIntClear(GPIO_PORTF_BASE,ALL_BUTTONS);
}


void PERIOD_TIMER_ADC(void){

    BaseType_t higherPriorityTaskWoken=pdFALSE;

    xSemaphoreGiveFromISR(semaforo_adc, &higherPriorityTaskWoken);

    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    portEND_SWITCHING_ISR(higherPriorityTaskWoken);

}



