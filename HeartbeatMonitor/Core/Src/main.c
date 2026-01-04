/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes -------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "max30102.h"
#include "i2c-lcd.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Definitions for HeartbeatTask (Legacy) */
osThreadId_t HeartbeatTaskHandle;
const osThreadAttr_t HeartbeatTask_attributes = {
  .name = "HeartbeatTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

/* Definitions for AcquisitionTask */
osThreadId_t AcquisitionTaskHandle;
const osThreadAttr_t AcquisitionTask_attributes = {
  .name = "AcquisitionTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal, // Higher priority to capture data
};

/* Definitions for ProcessingTask */
osThreadId_t ProcessingTaskHandle;
const osThreadAttr_t ProcessingTask_attributes = {
  .name = "ProcessingTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

/* Definitions for Queue and Mutex */
osMessageQueueId_t rawDataQueueHandle;
const osMessageQueueAttr_t rawDataQueue_attributes = {
  .name = "rawDataQueue"
};

osMutexId_t bpmMutexHandle;
const osMutexAttr_t bpmMutex_attributes = {
  .name = "bpmMutex"
};
/* USER CODE BEGIN PV */
uint32_t ir_val = 0;
uint32_t red_val = 0;
float temp_val = 0.0f;
float bpm = 0.0f;
volatile uint32_t last_beat_time = 0;
int finger_on_counter = 0;
// Circular Buffer for Dynamic Thresholding
#define BUFFER_SIZE 50
uint32_t ir_buffer[BUFFER_SIZE];
int buffer_idx = 0;
uint32_t amplitude = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartAcquisitionTask(void *argument);
void StartProcessingTask(void *argument);
void StartDisplayTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  // I2C Scanner
  printf("Scanning I2C bus...\r\n");
  HAL_StatusTypeDef result;
  uint8_t i;
  for (i=1; i<128; i++) {
      result = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i<<1), 2, 2);
      if (result == HAL_OK) {
          printf("I2C device found at address 0x%02X\r\n", i);
      }
  }

  MAX30102_Init(&hi2c1);
  lcd_init(&hi2c1);
  lcd_clear();
  lcd_put_cur(0, 0);
  lcd_send_string("Heartbeat Init");
  HAL_Delay(1000);
  lcd_clear();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the Queue */
  rawDataQueueHandle = osMessageQueueNew(16, sizeof(uint32_t), &rawDataQueue_attributes);

  /* Create the Mutex */
  bpmMutexHandle = osMutexNew(&bpmMutex_attributes);

  /* Create the thread(s) */
  /* Step 2: Architecture - Separate Tasks */
  AcquisitionTaskHandle = osThreadNew(StartAcquisitionTask, NULL, &AcquisitionTask_attributes);
  ProcessingTaskHandle = osThreadNew(StartProcessingTask, NULL, &ProcessingTask_attributes);
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* Legacy Handles (Unused but kept to match generated code footprint if needed) */
  HeartbeatTaskHandle = osThreadNew(StartDefaultTask, NULL, &HeartbeatTask_attributes); // Kept minimal

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
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
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the HeartbeatTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartAcquisitionTask(void *argument)
{
  /* Step 1: Pulse Acquisition Task */
  for(;;)
  {
    MAX30102_ReadFIFO(&red_val, &ir_val);
    
    // Add raw sample to Queue (Wait 0, drop if full to be real-time safe)
    osMessageQueuePut(rawDataQueueHandle, &ir_val, 0, 0);

    // With 8x hardware averaging at 100Hz, effective output is ~12.5 samples/sec
    osDelay(80); // 12.5Hz Sampling to match sensor
  }
}

void StartProcessingTask(void *argument)
{
  /* Step 4: Core Algorithm Task */
  uint32_t raw_ir;
  
  for(;;)
  {
    // Wait for new sample
    if (osMessageQueueGet(rawDataQueueHandle, &raw_ir, NULL, 100) == osOK) {
        
        // --- 1. Signal Processing ---
        // 1. Add to Circular Buffer
        ir_buffer[buffer_idx] = raw_ir;
        buffer_idx = (buffer_idx + 1) % BUFFER_SIZE;
    
        // 2. Dynamic Threshold
        uint32_t min_val = 0xFFFFFFFF;
        uint32_t max_val = 0;
        for(int i=0; i<BUFFER_SIZE; i++) {
            if(ir_buffer[i] < min_val) min_val = ir_buffer[i];
            if(ir_buffer[i] > max_val) max_val = ir_buffer[i];
        }
        
        // AC Amplitude (Min/Max Difference)
        amplitude = max_val - min_val;
        // 85% Threshold - Only catches main systolic peak
        uint32_t threshold = min_val + (amplitude * 85 / 100);

        // 3. Finger Detection
        if (raw_ir > 10000 && amplitude > 400) {
            finger_on_counter = 50; 
        } else {
            if(finger_on_counter > 0) finger_on_counter--;
        }
    
        float computed_bpm = 0.0f;
        
        // Adaptive Lockout Period
        // HIGHER initial lockout (700ms = max 85 BPM for first reading)
        // Then adapts based on 65% of previous interval
        static uint32_t adaptive_lockout = 700;

        if (finger_on_counter > 0) { 
            static uint8_t is_above = 0;
            uint32_t hyst = amplitude / 15;
            
            if (raw_ir > threshold + hyst) { 
                if (is_above == 0 && (HAL_GetTick() - last_beat_time > adaptive_lockout)) { 
                    is_above = 1;
                    
                    uint32_t delta = HAL_GetTick() - last_beat_time;
                    
                    // Update adaptive lockout (65% of interval)
                    // Min 350ms (max 170 BPM), Max 1000ms
                    uint32_t new_lockout = (delta * 65) / 100;
                    if (new_lockout < 350) new_lockout = 350;
                    if (new_lockout > 1000) new_lockout = 1000;
                    adaptive_lockout = new_lockout;
                    
                    float instant_bpm = 60000.0f / delta;
                    
                    // Sanity Check: Reject readings that are >1.5x or <0.6x current BPM
                    // This catches double-count spikes (e.g., 70 -> 140 rejected)
                    if (bpm > 10.0f) {
                        if (instant_bpm > bpm * 1.5f || instant_bpm < bpm * 0.6f) {
                            instant_bpm = 0; // Reject this reading
                        }
                    }
                    
                    if (instant_bpm > 40 && instant_bpm <= 180) {
                         computed_bpm = instant_bpm;
                    }
                    last_beat_time = HAL_GetTick();
                }
            } else if (raw_ir < threshold - hyst) {
                is_above = 0;
            }
        } else {
            adaptive_lockout = 700; // Reset to safe value
        }
        
        // --- 4. Shared Data Update (Mutex) ---
        osMutexAcquire(bpmMutexHandle, osWaitForever);
        if (finger_on_counter > 0 && computed_bpm > 0) {
             if (bpm < 10.0f) { 
                bpm = computed_bpm; 
            } else {
                bpm = (bpm * 0.8f) + (computed_bpm * 0.2f); 
            }
            
            if (bpm > 100.0f) {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
                HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            }

        } else if (finger_on_counter == 0) {
             bpm = 0.0f;
             HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
        }
        osMutexRelease(bpmMutexHandle);
        
    } // End Queue Get
  }
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;) { osDelay(1000); }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the DisplayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
/* USER CODE END Header_StartTask02 */
void StartDisplayTask(void *argument)
{
  /* Step : Display Update Task */
  char buf[32]; 
  /* Infinite loop */
  for(;;)
  {
    // Securely read BPM first to use for both rows
    float display_bpm = 0.0f;
    osMutexAcquire(bpmMutexHandle, osWaitForever);
    display_bpm = bpm;
    osMutexRelease(bpmMutexHandle);

    uint32_t bpm_int = (uint32_t)display_bpm;
    if (bpm_int > 250) bpm_int = 0;

    // --- Top Row: Status ---
    lcd_put_cur(0, 0);
    if (finger_on_counter > 0) {
        if (bpm_int > 100) {
            snprintf(buf, 32, "Status: High    ");
        } else if (bpm_int < 60 && bpm_int > 10) { // >10 to avoid "Low" when just starting
            snprintf(buf, 32, "Status: Low     ");
        } else if (bpm_int <= 10) {
             snprintf(buf, 32, "Status: Calc... "); // Initializing
        } else {
             snprintf(buf, 32, "Status: Normal  ");
        }
    } else {
         snprintf(buf, 32, "Status: Idle    ");
    }
    lcd_send_string(buf);
    
    // --- Bottom Row: BPM Value ---
    lcd_put_cur(1, 0);

    if (finger_on_counter > 0) {
        if (amplitude < 300) {
             snprintf(buf, 32, "Weak Signal...  ");
        } else {
             snprintf(buf, 32, "BPM: %lu        ", bpm_int);
        }
    } else {
        snprintf(buf, 32, "No Finger       "); 
    }
    lcd_send_string(buf);
    
    osDelay(200); // 5Hz Update
  }
}

void StartTask02(void *argument) { StartDisplayTask(argument); } // Redirect if RTOS calls old name

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
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
