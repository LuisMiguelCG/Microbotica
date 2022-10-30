# Practica 2 (I): Caracterizacion y uso de sensores en el Skybot.

## Ejercicio 1. Uso basico de sensores de contacto.


Realizar un codigo donde, si en su movimiento, el sensor del robot choca con un obstáculo, se cambie el estado del LED y se modifique la trayectoria del robot –por ejemplo, hace retroceder un poco al robot y girar hacia un lado otro poco, para luego continuar su trayectoria rectilínea- . La detección en el puerto se debe realizar por interrupción



## Ejercicio 2. Aplicacion de respuesta a las lecturas del sensor. Tablas de *look-up*.


Programa una aplicación para la placa TIVA conectada a un sensor de distancia tipo SHARP, establece 3 posibles intervalos activos equiespaciados de funcionamiento en el rango del sensor que hayas medido, y programa una aplicación que realice las siguientes acciones: 
• Si se detecta una distancia en el primer intervalo activo [x1,x2]cm, se deberá encender el LED verde.
• Si se detecta una distancia en el segundo intervalo activo [x2,x3]cm, se deberá encender el LED rojo.
• Si se detecta una distancia en el tercer intervalo [x3-x4cm], se deberán encender ambos LEDS.
• Para cualquier otra distancia (x<x1, o x > x4) los LEDS deben permanecer apagados.
• Indica claramente en tu programa a que valores en centimetros se corresponden esos intervalos.




