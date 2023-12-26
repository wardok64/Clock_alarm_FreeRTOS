# Clock_alarm_FreeRTOS
## Reloj con alarma en FreeRTOS

**Materiales:**

- LCD 16x2 JHD162A.
- PCF8574 driver I2C.
- Cables jumper.
- 2 resistencias 4.7k. 1/2W.
- Botón pulsador.
- STM32F103C8T6 (Bluepill).
- Flasher (ST-link / BlackMagicProbe / J-link)

**Diagrama electronico:**
  ![image](https://github.com/wardok64/Clock_alarm_FreeRTOS/assets/104173190/36af5478-44d5-44d2-bc69-6b28cb6bb78f)

En un proyecto nuevo con FreeRTOS habilitado se desarrolla el firmware necesario para implementar un reloj con alarma que cumple con los siguientes requerimientos:

**• Se inicializa el reloj en “00:00:00”, con un valor de alarma a cierta hora, minuto y segundo, especificada mediante macros:**
```
/* MACROS to set the time alarm */
#define SET_SEC_ALARM (5U)
#define SET_MIN_ALARM (5U)
#define SET_HOU_ALARM (1U)
```

**• Se inicializa un TIMER, el cual se configura para ejecutar una callback cada segundo, durante la ejecucion de la callback asociada a este TIMER se libera un semáforo identificado como “xSemphrSec” el cual la tarea “secondsTask” estará configurada para tratar de tomar (take) este semaforo, esto causa que la tarea “secondsTask” se sincronice y ejecute cada segundo. Se utiliza la función “xSemaphoreGiveFromISR()” dentro de la Callback, y al final se utiliza “portYIELD_FROM_ISR()” la cual obliga al scheduler a evaluar inmediatamente qué tarea debe ponerse en contexto.**

**• Se declara una tarea secondsTask (que representa los segundos), la cual:**

 - Inicializa el TIMER mediante las siguientes funciones:
```
MX_TIM2_Init();
HAL_TIM_Base_Start_IT(&htim2);
```
 - Se debe ejecutar de manera periódica cada segundo (solicitando el semáforo proporcionado por la callback del TIMER asociado).

 - Aproximadamente cada segundo que transcurre incrementa una variable identificada como “value” la cual al llegar al valor de 60 (valor establecido como máximo), liberara un semáforo identificado como “xSemphrMin” para posteriormente reiniciar el valor de la variable “value” a 0.
```
xSemaphoreGive(xSemphrMin);
psecondsTime->value = RESET;
```
 - Evaluá si el valor actual de la variable “value” es igual al valor “segundos” correspondiente al tiempo de la alarma (SET_SEC_ALARM), si esto se cumple, entonces se hace set de una bandera en un grupo de eventos (uno de tres bits).
```
if (psecondsTime->value == SET_SEC_ALARM)
{
	/* Set seconds bit for event group */
	xEventGroupSetBits(xEventAlarm, BIT_SEC_ALARM);
```

 - Envı́a un mensaje a una queue llamada “queueTime”, usando la estructura definida más adelante,que contiene la variable seconds, y el identificador de segundos. Este mensaje se pasa por referencia.
```
psecondsTime->time_types = SECONDS;
xQueueSend(queueTime, &psecondsTime, portMAX_DELAY);
```
 - Estructura:
```
typedef enum {
	seconds_type,
	minutes_type,
	hours_type
}time_types_t;

typedef struct {
	time_types_t time_types;
	uint8_t value;
}time_msg_t;
```
**• Se declara una tarea con el identificador “minutesTask” (que representa los minutos), la cual:**
```
if (xTaskCreate(minutesTask, "task2", stackSize, NULL, tskIDLE_PRIORITY + 1, NULL) == pdPASS)
{
```
 - Espera a que se libere el semáforo “xSemphrMin”.
```
/* Block waiting for the semaphore "xSemphrSec" to become available. */
if( xSemaphoreTake( xSemphrSec, portMAX_DELAY ) == pdTRUE )
{
```
 - Una vez liberado el semáforo “xSemphrMin”, incrementa una variable “value”, para intentar tomar el semáforo “xSemphrMin” una vez más (en un ciclo infinito).
```
pminutesTime->value++;
```
 - Cuando la variable minutes llega a 60, libera un semáforo llamado “xSemphrHou” y se regresa el valor de la variable “value” a 0.
```
xSemaphoreGive(xSemphrMin);
psecondsTime->value = RESET;
```
 - Cuando el valor de “value” es igual al valor de minutos de la alarma, se hace Set de una bandera en un grupo de eventos (uno de tres bits).
```
if (pminutesTime->value == SET_MIN_ALARM) 
{
	xEventGroupSetBits(xEventAlarm, BIT_MIN_ALARM);
```
 - Envı́a un mensaje a una queue llamada “queueTime”, usando la estructura definida más adelante, que contiene la variable “value”, y el identificador de minutes. Este mensaje se pasa por referencia.
```
pminutesTime->time_types = MINUTES;
xQueueSend(queueTime, &pminutesTime, portMAX_DELAY);
```
 - Estructura:
```
typedef enum {
	seconds_type,
	minutes_type,
	hours_type
}time_types_t;

typedef struct {
	time_types_t time_types;
	uint8_t value;
}time_msg_t;
```
**• Se declara una tarea hoursTask(que representa las horas), la cual:**

 - Espera a que se libere el semáforo “xSemphrHou”.
```
if(xSemaphoreTake(xSemphrHou, portMAX_DELAY) == pdTRUE)
{
```
 - Una vez liberado el semáforo “xSemphrHou”, incrementa una variable “value”, para intentar tomar el semáforo “xSemphrHou” una vez más (en un ciclo infinito).
```
phoursTime->value++;
```
 - Cuando el valor de “value” es igual al valor de horas de la alarma, se hace Set de una bandera en un grupo de eventos (uno de tres bits).
```
if (phoursTime->value == SET_HOU_ALARM)
{
	xEventGroupSetBits(xEventAlarm, BIT_HOU_ALARM);
```
 - Cuando “value” llega a 24 se asigna nuevamente su valor a 0 (RESET).

 - Envı́a un mensaje a una queue llamada “queueTime”, usando la estructura definida más adelante, que contiene la variable “value”, y el identificador de horas. Este mensaje se pasa por referencia.
```
phoursTime->time_types = HOURS;
xQueueSend(queueTime, &phoursTime, portMAX_DELAY);
```
 - Estructura:
```
typedef enum {
	seconds_type,
	minutes_type,
	hours_type
}time_types_t;

typedef struct {
	time_types_t time_types;
	uint8_t value;
}time_msg_t;
```
**• Se declara una tarea alarmTask, la cual:**

 - Espera a que las tres banderas del grupo de eventos, accionados por las tareas de horas, minutos y segundos, están en Set, las tres al mismo tiempo.
```
xEventGroupValue = xEventGroupWaitBits(xEventAlarm, uxBits, pdFALSE, pdTRUE, portMAX_DELAY);
if(((xEventGroupValue & (uxBits)) != 0 ))
{
```
 - En caso de que las tres banderas estén accionadas al mismo tiempo, se espera por el mutex para escribir por el periferico I2C en la LCD el mensaje “ALARM” indicando que el tiempo actual hace match con el establecido de alarma.
```
/* Try to take the mutex to print by I2C */
if(xSemaphoreTake(mutexI2CPeriph, portMAX_DELAY) == pdPASS)
{
	LCD_SET_cursor(0,7);
	LCD_I2C_write_text("ALARM");
 ```
 - El mensaje de ALARM, deberá esta visible en la LCD hasta que el usuario presione un botón conectado al pin 10 del puerto A.
```
if ((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10))
```
 - Posteriormente vuelve a esperar por el grupo de eventos.

**• Se declara una tarea printTask, la cual:**

 - Espera por mensajes en la queue “queueTime”, y cada que llegan nuevos mensajes, actualiza el valor del tiempo local en horas y minutos y segundos, dependiendo de que tipo de mensaje se recibió, la queue es de tamano 1.
```
/* Receive data from queue, and try for semaphr to print by I2C periph*/
if(xQueueReceive(queueTime, &actual_time, portMAX_DELAY) == pdTRUE)
…
switch (actual_time->time_types)
{
	case SECONDS:
	...
	case MINUTES:
	...
	case HOURS:
```
 - Cada que llega un mensaje nuevo, solicita el mutex para tener acceso al periferico I2C e imprime el valor de la hora en la pantalla LCD.
```
if(xSemaphoreTake(mutexI2CPeriph, portMAX_DELAY) == pdPASS)
{
```
**• La pantalla LCD debe estar protegido correctamente, esto es: Los mensajes que se desplieguen en la pantalla LCD no deben verse interrumpidos, para ello utilizamos el mutex.**

**Diagrama de bloques:**
![image](https://github.com/wardok64/Clock_alarm_FreeRTOS/assets/104173190/2ecbe8c9-febc-4b3d-ad0a-d2a355ab9f37)




