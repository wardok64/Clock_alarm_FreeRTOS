
#include "main.h"
#include <stdbool.h>
#include <stdio.h>
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>
#include <event_groups.h>


#define RESET   0 

/* Macros for switch type */
#define SECONDS        (0U)
#define MINUTES        (1U)
#define HOURS          (2U)

/* Macros for max counter for seconds/minutes/hours */
#define MAX_CLOCK      (60U)
#define MAX_CLOCK_HOUR (24U)

/* Bits to event group ALARM */
#define BIT_SEC_ALARM  (1<<0)
#define BIT_MIN_ALARM  (1<<1)
#define BIT_HOU_ALARM  (1<<2)

/* MACROS to set the time alarm */
#define SET_SEC_ALARM  (5U)
#define SET_MIN_ALARM  (5U)
#define SET_HOU_ALARM  (1U)

/* I2C address of LCD */
#define SLAVE_ADDRESS_LCD  0x4E	

/* Definiciones de Comandos de LCD */
#define     LCD_CLEAR	        0x01    //Limpia pantalla
#define     LCD_HOME	        0x02    //Retorno al inicio
#define     LCD_CURSOR_ON	    0x0F    //Cursor on
#define     LCD_CURSOR_OFF	  0x0C    //Cursor off
#define     LCD_LINEA1	      0x00	  //Promera Fila
#define     LCD_LINEA2		    0XC0	  //Segunda Fila
#define     LCD_LINEA3		    0x94	  //Tercera Fila
#define     LCD_LINEA4		    0xD4	  //Cuarta Fila
#define     LCD_LEFT		      0x10	  //Cursor a la izquierda
#define     LCD_RIGHT		      0x14	  //Cursor a la derecha
#define     LCD_ROT_LEFT	    0x18	  //Rotar a la izquierda
#define     LCD_ROT_RIGHT	    0x1C	  //Rotar a la derecha
#define     LCD_OFF 		      0x08	  //apaga el display

/* The handle of I2C communication will be by DMA (non-blocking) */
I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_tx;

/* The handle of base time will be by TIMER 2 */
TIM_HandleTypeDef htim2;

/* Private function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);

/* Task FreeRTOS */
void secondsTask(void *argument);
void minutesTask(void *argument);
void hoursTask(void *argument);
void printTask(void *argument);
void alarmTask(void *argument);

/* User LCD funcions */
void LCD_I2C_init(void);				         // Inicializa LCD
void LCD_I2C_cmd(char cmd);  			       // Envia comando LCD
void LCD_I2C_char(char data);  			     // Envia dato LCD
void LCD_I2C_write_text(char *dato);  	 // Envia cadena de caracteres
void LCD_SET_cursor(int row, int col);   // set cursor.

void citoa(int num, char* str);
void reverse(char str[], int length);

/* Semaphores for enable; second, minute and hour counter */
SemaphoreHandle_t xSemphrSec = NULL;
SemaphoreHandle_t xSemphrMin = NULL;
SemaphoreHandle_t xSemphrHou = NULL;

/* This is a handle for the queue that will be used by tasks 
queue to comunicate time tasks with print task, everytime a new time
send to queue and print task receive and print */
static QueueHandle_t queueTime = NULL;

/* Event group to indicate actual time is equal to alarm time
it set every event; second (bit second), minute (bit minute) and hour (bit hour) */
EventGroupHandle_t xEventAlarm;

/* Create storage for a pointer to a mutex (this is the same container as a semaphore)
This is the mutex for I2C (shared resource) */
SemaphoreHandle_t mutexI2CPeriph = NULL;

typedef enum {
  seconds_type,
  minutes_type,
  hours_type
}time_types_t;

typedef struct {
  time_types_t time_types;
  uint8_t value;
}time_msg_t;
/**
    Below, we define three instances of the time_msg_t struct. Since these are globals 
    they will always be available for use. If they were defined non-statically inside 
    a function their scope would be limited to the function (and memory contents of 
    the pointer reference passed would be on the stack, potentially/likely getting 
    corrupt/changed before use).
  */
time_msg_t secondsTime, minutesTime, hoursTime;

typedef uint8_t time;

typedef struct
{
	time hour;
	time minute;
	time second;
}alarm_t;

int main(void)
{
  // some common variables to use for each task
	// 128 * 4 = 512 bytes
  // 64 * 5 = 320 bytes
	//(recommended min stack size per task)
	const static uint32_t stackSize = 64; // 128

  static alarm_t alarm;

  alarm.second = SET_SEC_ALARM;
  alarm.minute = SET_MIN_ALARM;
  alarm.hour   = SET_HOU_ALARM;

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  
  /* Init the LCD  before start scheduler */
  LCD_I2C_init();
  LCD_I2C_write_text("RELOJ");
  LCD_SET_cursor(1,0);
  LCD_I2C_write_text("00:00:00");

  /* We are using the semaphore for synchronisation so we create a binary
  semaphore rather than a mutex.  We must make sure that the interrupt
  does not attempt to use the semaphore before it is created! */
  xSemphrSec = xSemaphoreCreateBinary();
  xSemphrMin = xSemaphoreCreateBinary();
  xSemphrHou = xSemaphoreCreateBinary();

  /* Create a queue that can store 2 uint8_t's
  using ledCmdQueue to point to it */
  queueTime = xQueueCreate(1, sizeof(time_msg_t*));
  assert_param(queueTime != NULL);

  /* Event group for alarm */
  xEventAlarm = xEventGroupCreate();
  assert_param(xEventAlarm != NULL);

  /* Create a mutex - note this is just a special case of a binary semaphore */
  mutexI2CPeriph = xSemaphoreCreateMutex();
  assert_param(mutexI2CPeriph != NULL);

  /* Ensure proper priority grouping for freeRTOS */
	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);	

	if (xTaskCreate(secondsTask, "task1", stackSize, (void *)&alarm, tskIDLE_PRIORITY + 1, NULL) == pdPASS)
	{
		if (xTaskCreate(minutesTask, "task2", stackSize, (void *)&alarm, tskIDLE_PRIORITY + 1, NULL) == pdPASS)
		{
			if (xTaskCreate(hoursTask, "task3", stackSize, (void *)&alarm, tskIDLE_PRIORITY + 1, NULL) == pdPASS)
			{
        if (xTaskCreate(printTask, "print_task", stackSize, NULL, tskIDLE_PRIORITY + 2, NULL) == pdPASS)
			  {
          if (xTaskCreate(alarmTask, "alarm_task", stackSize, NULL, tskIDLE_PRIORITY + 1, NULL) == pdPASS)
			    {
				    /* Start the scheduler - shouldn't return unless there's a problem */
				    vTaskStartScheduler();
          }
        }
			}
		}
	}

  while (1)
  {
    /* In RTOS the program should never get here */
  }
}

void secondsTask(void *argument)
{
  time_msg_t* psecondsTime = &secondsTime;
  psecondsTime->value = RESET;

  alarm_t alarm;
	alarm = *((alarm_t*)argument);
  
  /* Es importante inicializar el timer despues de inizializar el semphr
     por que corremos el riezgo de que se ejecute la Callback del TIMER 
     antes del ciclo while, es decir la callback usara el semphr antes de
     solicitarlo por la tarea "secondsTask" */
  MX_TIM2_Init();
  HAL_TIM_Base_Start_IT(&htim2);

  while(1)
  {
    /* Block waiting for the semaphore "xSemphrSec" to become available. */
    if( xSemaphoreTake( xSemphrSec, portMAX_DELAY ) == pdTRUE )
    {
      if (psecondsTime->value == alarm.second)
      {
        /* Set seconds bit for event group */
        xEventGroupSetBits(xEventAlarm, BIT_SEC_ALARM);
      }
      else 
      {
        /* Clear seconds bit for event group */
        xEventGroupClearBits(xEventAlarm, BIT_SEC_ALARM);
      }
      psecondsTime->value++;
      /* If met the max seconds time, restore to 0 and give semphr to minutesTask */
      if(MAX_CLOCK == psecondsTime->value)
      {
        psecondsTime->value = RESET;
        xSemaphoreGive(xSemphrMin);
      }
      
      /* Save values in struct and send to queue every second */
      psecondsTime->time_types = seconds_type;
      xQueueSend(queueTime, &psecondsTime, portMAX_DELAY);
    }
  }
}

void minutesTask( void* argument )
{
  time_msg_t  *pminutesTime = &minutesTime;
  pminutesTime->value = RESET;

  alarm_t alarm;
	alarm = *((alarm_t*)argument);

	while(1)
	{
    if(xSemaphoreTake(xSemphrMin, portMAX_DELAY) == pdTRUE)
    {
      pminutesTime->value++;
      if(MAX_CLOCK == pminutesTime->value)
      {
        pminutesTime->value = RESET;
        xSemaphoreGive(xSemphrHou);
      }

      if (pminutesTime->value == alarm.minute) 
      {
        xEventGroupSetBits(xEventAlarm, BIT_MIN_ALARM);
      }
      else 
      {
        xEventGroupClearBits(xEventAlarm, BIT_MIN_ALARM);
      }
      
      pminutesTime->time_types = minutes_type;
      xQueueSend(queueTime, &pminutesTime, portMAX_DELAY);
    }
	}
}

void hoursTask( void* argument )
{
  time_msg_t* phoursTime = &hoursTime;
  phoursTime->value = RESET;

  alarm_t alarm;
	alarm = *((alarm_t*)argument);

	while(1)
	{
    if(xSemaphoreTake(xSemphrHou, portMAX_DELAY) == pdTRUE)
    {
      phoursTime->value++;

      if(MAX_CLOCK_HOUR == phoursTime->value)
      {
        phoursTime->value = RESET;
      }
      if (phoursTime->value == alarm.hour)
      {
        xEventGroupSetBits(xEventAlarm, BIT_HOU_ALARM);
      }
      else 
      {
        xEventGroupClearBits(xEventAlarm, BIT_HOU_ALARM);
      }
     
      phoursTime->time_types = hours_type;
      xQueueSend(queueTime, &phoursTime, portMAX_DELAY);
    }
	}
}

/* Task to implement
   a QUEUE MULTIPLE TASK WRITE SINGLE READER.
   to receive the time struct and print by I2C to LCD */
void printTask (void* argument)
{
  time_msg_t* actual_time;

  /* Aux variables to convert Integer time to ASCI in format:
  [0] = 0
  [1] = 0
  [2] = \0  caracter null */
  char seconds_s[3], minutes_s[3], hours_s[3];

  while (1)
  {
    /* Receive data from queue, and try for semaphr to print by I2C periph*/
    if(xQueueReceive(queueTime, &actual_time, portMAX_DELAY) == pdTRUE)
    {
      /* Take MUTEX for share I2C */
      if(xSemaphoreTake(mutexI2CPeriph, portMAX_DELAY) == pdPASS)
      {
        switch (actual_time->time_types)
        {
          case seconds_type:
            citoa(actual_time->value, seconds_s);
            LCD_SET_cursor(1,6);
            LCD_I2C_write_text(seconds_s);
          break;
          case minutes_type:
            citoa(actual_time->value, minutes_s);
            LCD_SET_cursor(1,3);
            LCD_I2C_write_text(minutes_s);
          break;
          case hours_type:
            citoa(actual_time->value, hours_s);
            LCD_SET_cursor(1,0);
            LCD_I2C_write_text(hours_s);
          break;
          default:
          break;
        }
        /* Task give the mutex for other tasks (Shared resource )*/
        xSemaphoreGive(mutexI2CPeriph);
      }
    }
  }
}

void alarmTask (void* argument)
{
  /* Define a variable which holds the state of events (bits) */
  const EventBits_t uxBits = (BIT_SEC_ALARM | BIT_MIN_ALARM | BIT_HOU_ALARM);
  EventBits_t xEventGroupValue;

  /* Flag to print alarm message once and disable for event group 
      until press user button RESET ALARM */
  bool flagPrintAlarmI2COnce = true;

  while (1)
  {
    if(flagPrintAlarmI2COnce)
    {
      /* We need to pass "pdFALSE" in clearOnExit, to keep the flags, and then clear the flags only with 
      a user button using the "xEventGroupClearBits()" */
      xEventGroupValue = xEventGroupWaitBits(xEventAlarm, uxBits, pdFALSE, pdTRUE, portMAX_DELAY);
      if(((xEventGroupValue & (uxBits)) != 0 ))
      {
        /* Try to take the mutex to print by I2C */
        if(xSemaphoreTake(mutexI2CPeriph, portMAX_DELAY) == pdPASS)
        {
          LCD_SET_cursor(0,7);
          LCD_I2C_write_text("ALARM");
          HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
          xSemaphoreGive(mutexI2CPeriph);
        }
        /* Set flag to false, this allow us to execute once the print ALARM code */
        flagPrintAlarmI2COnce = false;
      }
    }
    if ((!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10)) && (!flagPrintAlarmI2COnce))
    {
      /* If user button and flag is false, then try to take the mutex to use I2C peripheral */
      if(xSemaphoreTake(mutexI2CPeriph, portMAX_DELAY) == pdPASS)
      {
        /* Only until press button will be cleared bits of event group */
        xEventGroupClearBits(xEventAlarm, uxBits);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
        LCD_SET_cursor(0,7);
        LCD_I2C_write_text("CLEAN");
        /* At the end give the mutex and set the flag to true */
        xSemaphoreGive(mutexI2CPeriph);
        flagPrintAlarmI2COnce = true;
      }
    }
  }
}

/* Timer callback, give semphr from ISR to secondTask every second */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /*  */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  if(htim->Instance == TIM2)
  {
    /* Unblock the task by releasing the semaphore. */
    xSemaphoreGiveFromISR( xSemphrSec, &xHigherPriorityTaskWoken );
  }
  /* Yield if xHigherPriorityTaskWoken is true.  The 
  actual macro used here is port specific. */
  portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}


void LCD_SET_cursor(int row, int col)
 { 
	switch (row) { 
		case 0: col |= 0x80; 
		break; 
		case 1: col |= 0xC0; 
		break; 
	}
	 LCD_I2C_cmd(col); 
}

void LCD_I2C_cmd(char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
 do
  {
    if(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)SLAVE_ADDRESS_LCD, (uint8_t*)data_t, 4)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }

    /*##-3- Wait for the end of the transfer #################################*/  
    /*  Before starting a new communication transfer, you need to check the current   
        state of the peripheral; if it’s busy you need to wait for the end of current
        transfer before starting a new one.
        For simplicity reasons, this example is just waiting till the end of the 
        transfer, but application may perform other tasks while transfer operation
        is ongoing. */  
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    } 

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
  } while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
}


void LCD_I2C_char(char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
do
  {
    if(HAL_I2C_Master_Transmit_DMA(&hi2c1, (uint16_t)SLAVE_ADDRESS_LCD, (uint8_t*)data_t, 4)!= HAL_OK)
    {
      /* Error_Handler() function is called when error occurs. */
      Error_Handler();
    }

    /*##-3- Wait for the end of the transfer #################################*/  
    /*  Before starting a new communication transfer, you need to check the current   
        state of the peripheral; if it’s busy you need to wait for the end of current
        transfer before starting a new one.
        For simplicity reasons, this example is just waiting till the end of the 
        transfer, but application may perform other tasks while transfer operation
        is ongoing. */  
    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
    {
    } 

    /* When Acknowledge failure occurs (Slave don't acknowledge it's address)
       Master restarts communication */
  } while(HAL_I2C_GetError(&hi2c1) == HAL_I2C_ERROR_AF);
}

void LCD_I2C_init(void)
{
  HAL_Delay(50);
  LCD_I2C_cmd(0x30);
  HAL_Delay(40);
  LCD_I2C_cmd(0x30);
  HAL_Delay(20);
  LCD_I2C_cmd(0x30);
  HAL_Delay(20);
  LCD_I2C_cmd(0x20);
  HAL_Delay(20);
  LCD_I2C_cmd(0x28);
  HAL_Delay(10);
  LCD_I2C_cmd(0x08);
  HAL_Delay(10);
  LCD_I2C_cmd(0x01);
  HAL_Delay(20);
  LCD_I2C_cmd(0x06);
  HAL_Delay(10);
  LCD_I2C_cmd(0x0C);
  HAL_Delay(20);
}

void LCD_I2C_write_text(char *str)
{
	while (*str) LCD_I2C_char(*str++);
}



/* A utility function to reverse a string */
void reverse(char str[], int length)
{
    int start = 0;
    int end = length - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        end--;
        start++;
    }
}

/* Function to convert integer to ASCII format for TIME format 00:00:00 */
void citoa(int num, char* str)
{
  /* The base is a constant (10) for this project, but could be changed */
  const int base = 10;
  int i = 0;

  /* Handle 0 explicitly, otherwise empty string is
    * printed for 00 */
  if (num == 0) {
      str[0] = '0';   //0
      str[1] = '0';   //1
      str[2] = '\0';  //2
      //return str;   
  }
  else
  {
    if((num < 10) && (num > 0)) {  // shift unidad and add 0 ej -> 01 - 02 - 03 etc
      int rem = num % base;
      str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
      num = num / base;
      str[i++] = '0';
    }
    // Process individual digits
    while (num != 0) {
        int rem = num % base;
        str[i++] = (rem > 9) ? (rem - 10) + 'a' : rem + '0';
        num = num / base;
    }

    str[i] = '\0'; // Append string terminator
    // Reverse the string
    reverse(str, i);
  }
}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7200-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500; //10000 // fastest 500
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}


static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();


  /*Configure GPIO pin : USER_LED_Pin */
  GPIO_InitTypeDef GPIO_InitStruct;
  /* LED to indicate ALARM mode */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  /*Configure GPIO pin : USER_LED_Pin */
  /* BUTTON to reset the ALARM mode*/
  GPIO_InitTypeDef pulsador;
  pulsador.Pin = GPIO_PIN_10;
  pulsador.Mode = GPIO_MODE_INPUT;
  pulsador.Pull = GPIO_PULLUP;
  pulsador.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &pulsador);

}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
