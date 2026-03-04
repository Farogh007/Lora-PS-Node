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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "usart.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RAK.h"
#include "battery_adc.h"
#include "car_detector.h"
#include "log.h"
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

/* USER CODE BEGIN PV */
static RAK_Handle rak;

static uint32_t next_send_ms = 0;

/* Battery-efficient schedule: sample only in a window before send */
#define SEND_INTERVAL_MS    60000u   /* 1 min between sends */
#define WINDOW_MS           30000u   /* sample sensors in last 30 s before send */
#define WINDOW_WARMUP_MS    2000u    /* warmup inside window before counting (baseline/radar settle) */
#define SAMPLE_INTERVAL_MS  100u     /* record car state every 100 ms for majority vote */
#define IDLE_LOOP_MS        500u     /* when idle, service RAK then sleep 500 ms */

/* Majority vote: count car_present over the window; send 1 if > 50% */
static uint32_t window_car_count;
static uint32_t window_sample_count;
static uint32_t window_last_sample_ms;

/* Battery: Li-ion 3000 mV empty, 4200 mV full -> 0..100% */
#define BAT_MV_EMPTY  3000u
#define BAT_MV_FULL   4200u

static uint8_t battery_mV_to_percent(uint32_t bat_mV)
{
    if (bat_mV <= BAT_MV_EMPTY) return 0;
    if (bat_mV >= BAT_MV_FULL) return 100;
    return (uint8_t)((bat_mV - BAT_MV_EMPTY) * 100u / (BAT_MV_FULL - BAT_MV_EMPTY));
}

static CarDetector_t g_cd;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
	Log_Write(ptr, len);
	return len;
}
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
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  BAT_ADC_Init(&hadc1);

  Log_Init(&huart1);
  Log_Enable();

  RAK_Cfg cfg = { .huart = &hlpuart1 };
  RAK_Init(&rak, &cfg);

  next_send_ms = HAL_GetTick() + SEND_INTERVAL_MS;
  window_car_count = 0;
  window_sample_count = 0;
  window_last_sample_ms = 0;

  printf("Fusion car detector start\r\n");

    MagneticSensorConfig_t mag_cfg;
    MagneticSensor_DefaultConfig(&mag_cfg);

    CarDetectorConfig_t fcfg;
    CarDetector_DefaultConfig(&fcfg);

    if (!CarDetector_Init(&g_cd, &hi2c1, MMC5983_I2C_ADDR_8BIT, 100,
  		  &mag_cfg, &fcfg, RADAR_PRESET_SHORT_RANGE))
    {
  	  printf("CarDetector_Init FAIL\r\n");
    }

    uint32_t last_print = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		uint32_t now = HAL_GetTick();
		int32_t time_until_send = (int32_t)(next_send_ms - now);
		bool in_window = (time_until_send > 0 && time_until_send <= (int32_t)WINDOW_MS);

		/* Idle: before sampling window — only service RAK, no sensor reads */
		if (!in_window) {
			RAK_Task(&rak);

			/* Send time reached */
			if (time_until_send <= 0) {
				uint16_t bat_raw = 0;
				uint32_t bat_mV = BAT_ADC_Read_mV(&bat_raw);
				uint8_t battery_pct = battery_mV_to_percent(bat_mV);

				if (RAK_IsJoined(&rak)) {
					/* Use majority vote over the last window (or 0 if no samples) */
					uint8_t car = (window_sample_count > 0 && window_car_count > window_sample_count / 2) ? 1 : 0;
					RAK_Status st = RAK_SendStatus(&rak, car, battery_pct);

					if (st == RAK_OK) {
						next_send_ms += SEND_INTERVAL_MS;
					} else {
						next_send_ms += 5000;
					}
				} else {
					next_send_ms += SEND_INTERVAL_MS;
				}
				window_car_count = 0;
				window_sample_count = 0;
				window_last_sample_ms = 0;
			} else {
				HAL_Delay(IDLE_LOOP_MS);
			}
			continue;
		}

		/* In sampling window: run sensors */
		RAK_Task(&rak);
		CarDetector_Tick(&g_cd, now);

		/* Warmup: run sensors a few seconds before counting (baseline/radar settle) */
		int32_t time_since_window_start = (int32_t)(now - (next_send_ms - WINDOW_MS));
		bool warmup_done = (time_since_window_start >= (int32_t)WINDOW_WARMUP_MS);

		if (warmup_done && (now - window_last_sample_ms >= SAMPLE_INTERVAL_MS)) {
			window_last_sample_ms = now;
			window_sample_count++;
			if (CarDetector_GetCarPresent(&g_cd)) {
				window_car_count++;
			}
		}

		/* Debug print only during window, every 200 ms */
		if (now - last_print >= 200) {
			last_print = now;
			MagneticSensorSample_t ms;
			(void) MagneticSensor_GetLatest(&g_cd.mag, &ms);
			bool mag_detected = (MagneticSensor_GetCarPresent(&g_cd.mag) != 0);
			bool rad_detected = g_cd.radar.car_detected;
			bool car_confirmed = CarDetector_GetCarPresent(&g_cd);
			printf("t=%lu mag=%u rad=%u CONFIRMED=%u | samples=%lu car_count=%lu | radRaw=%u intra=%ld dist=%ldmm\r\n",
					(unsigned long) now,
					(unsigned) mag_detected,
					(unsigned) rad_detected,
					(unsigned) car_confirmed,
					(unsigned long) window_sample_count,
					(unsigned long) window_car_count,
					(unsigned) g_cd.radar.presence_detected,
					(long) g_cd.radar.intra_score_x1000,
					(long) g_cd.radar.distance_mm);
		}

		HAL_Delay(5);
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    {
        RAK_OnUartRxCplt(&rak);
    }
}
/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
