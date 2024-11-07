/* USER CODE BEGIN Header */
/**
  *******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  *******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  *******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "fonts.h"
#include "ssd1306.h"
#include <stdio.h>

/* Private defines -----------------------------------------------------------*/
#define TASK_STACK_SIZE 128  // Reduced stack size for tasks
#define DISTANCE_QUEUE_SIZE 1 // Queue size of 1 for single distance value

/* Shared resource mutex */
SemaphoreHandle_t i2cMutex;
SemaphoreHandle_t gpioMutex;

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA

uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance = 0;  // cm
char strCopy[15];

// Queue handle
QueueHandle_t distanceQueue;

/* Task period and deadline definitions */
#define TASK1_PERIOD 200   // Period for Task 1 (in ms)
#define TASK2_PERIOD 300   // Period for Task 2 (in ms)

uint32_t task1Deadline = TASK1_PERIOD;  // Deadline for Task 1
uint32_t task2Deadline = TASK2_PERIOD;  // Deadline for Task 2

// Task priorities
#define GPIO_CEILING_PRIORITY 3  // Example ceiling priority for GPIO
#define I2C_CEILING_PRIORITY   2  // Example ceiling priority for I2C

// Task handles
TaskHandle_t task1_handle, task2_handle;

// Variables to store task's original priority before raising it
UBaseType_t originalTask1Priority;
UBaseType_t originalTask2Priority;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void task1_handler(void* parameters);
static void task2_handler(void* parameters);
void create_semaphores(void);
void update_task_priorities(void);

/* USER CODE BEGIN 4 */

// Function to update task priorities based on deadlines (DMS)
void update_task_priorities(void) {
    uint32_t currentTime = HAL_GetTick();

    // Calculate remaining time to deadline for each task
    uint32_t timeToTask1Deadline = (task1Deadline > currentTime) ? (task1Deadline - currentTime) : 0;
    uint32_t timeToTask2Deadline = (task2Deadline > currentTime) ? (task2Deadline - currentTime) : 0;

    // Update priorities: Shorter deadline = higher priority
    if (timeToTask1Deadline < timeToTask2Deadline) {
        // Task 1 has an earlier deadline, so it gets a higher priority
        vTaskPrioritySet(task1_handle, 2);  // Task 1 gets higher priority
        vTaskPrioritySet(task2_handle, 1);  // Task 2 gets lower priority
    } else {
        // Task 2 has an earlier deadline, so it gets a higher priority
        vTaskPrioritySet(task1_handle, 1);  // Task 1 gets lower priority
        vTaskPrioritySet(task2_handle, 2);  // Task 2 gets higher priority
    }
}

static void task1_handler(void* parameters) {
    while (1) {
        // Update task priorities based on deadlines (DMS)
        update_task_priorities();

        // Save the original task priority
        originalTask1Priority = uxTaskPriorityGet(NULL);

        // Take the GPIO semaphore and raise the priority to the ceiling priority
        if (xSemaphoreTake(gpioMutex, portMAX_DELAY) == pdTRUE) {
            vTaskPrioritySet(NULL, GPIO_CEILING_PRIORITY);  // Raise to ceiling priority

            // Start distance measurement
            HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
            __HAL_TIM_SET_COUNTER(&htim1, 0);
            while (__HAL_TIM_GET_COUNTER(&htim1) < 10);  // wait for 10 us
            HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

            pMillis = HAL_GetTick(); // used to avoid infinite while loop (for timeout)
            // wait for the echo pin to go high
            while (!(HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 10 > HAL_GetTick());
            Value1 = __HAL_TIM_GET_COUNTER(&htim1);

            pMillis = HAL_GetTick(); // used to avoid infinite while loop (for timeout)
            // wait for the echo pin to go low
            while ((HAL_GPIO_ReadPin(ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
            Value2 = __HAL_TIM_GET_COUNTER(&htim1);

            Distance = (Value2 - Value1) * 0.034 / 2;

            // Send distance value to the queue
            xQueueSend(distanceQueue, &Distance, portMAX_DELAY);

            xSemaphoreGive(gpioMutex);  // Release the GPIO mutex

            // Restore the original task priority after releasing the resource
            vTaskPrioritySet(NULL, originalTask1Priority);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100ms before next measurement
    }
}

static void task2_handler(void* parameters) {
    uint16_t receivedDistance;

    while (1) {
        // Update task priorities based on deadlines (DMS)
        update_task_priorities();

        // Save the original task priority
        originalTask2Priority = uxTaskPriorityGet(NULL);

        // Take the I2C mutex and raise the priority to the ceiling priority
        if (xSemaphoreTake(i2cMutex, portMAX_DELAY) == pdTRUE) {
            vTaskPrioritySet(NULL, I2C_CEILING_PRIORITY);  // Raise to ceiling priority

            // Receive distance value from the queue
            if (xQueueReceive(distanceQueue, &receivedDistance, portMAX_DELAY) == pdPASS) {
                // Display the distance
                SSD1306_GotoXY(0, 0);
                SSD1306_Puts("Distance:", &Font_11x18, 1);
                sprintf(strCopy, "%d    ", receivedDistance);
                SSD1306_GotoXY(0, 30);
                SSD1306_Puts(strCopy, &Font_16x26, 1);
                SSD1306_UpdateScreen();
            }

            xSemaphoreGive(i2cMutex);  // Release the I2C mutex

            // Restore the original task priority after releasing the resource
            vTaskPrioritySet(NULL, originalTask2Priority);
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Delay to reduce display update rate
    }
}

/* USER CODE END 4 */

/* System Initialization Functions (Unchanged) */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

/* Initialize peripherals like I2C, Timer, GPIO, etc. */
static void MX_I2C1_Init(void) {
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM1_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
    Error_Handler();
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = TRIG_PIN|ECHO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void create_semaphores(void) {
    gpioMutex = xSemaphoreCreateMutex();
    i2cMutex = xSemaphoreCreateMutex();
    if (gpioMutex == NULL || i2cMutex == NULL) {
        Error_Handler();  // Handle error if semaphore creation fails
    }
}

/* Main function to initialize system and start tasks */
int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    SSD1306_Init();

    // Create distance queue
    distanceQueue = xQueueCreate(DISTANCE_QUEUE_SIZE, sizeof(uint16_t));
    if (distanceQueue == NULL) {
        Error_Handler();  // Handle error if queue creation fails
    }

    // Create semaphores
    create_semaphores();

    // Create tasks
    BaseType_t status;

    status = xTaskCreate(task1_handler, "Task 1", TASK_STACK_SIZE, NULL, 2, &task1_handle);
    configASSERT(status == pdPASS);

    status = xTaskCreate(task2_handler, "Task 2", TASK_STACK_SIZE, NULL, 1, &task2_handle);
    configASSERT(status == pdPASS);

    // Start the FreeRTOS scheduler
    vTaskStartScheduler();

    // If we get here, there was an error
    Error_Handler();
}
void Error_Handler(void) {
    // Optionally, add code to signal an error, e.g., turn on an LED, or send an error message
    __disable_irq();  // Disable interrupts
    while (1) {
        // Stay here, perhaps blink an LED or add a debug log
    }
}
