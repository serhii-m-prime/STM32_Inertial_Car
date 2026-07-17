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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sh1107.h"
#include "debug_ui.h"
#include "crsf.h"
#include "main_fsm.h"
#include "motor.h"
#include "servo.h"
#include "rc_input.h"
#include "vehicle_config.h"
#include "config_console.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
	APP_MODE_DRIVE = 0, APP_MODE_CONFIG
} AppMode_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CONFIG_BUTTON_DEBOUNCE_MS 50U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

static AppMode_t app_mode = APP_MODE_DRIVE;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static bool ConfigButton_IsPressed(void);
static void USART2_EnterConfigMode(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Variables for DWT Profiling
uint32_t last_ui_update = 0;
uint32_t max_loop_time_us = 0;
uint32_t current_cycles = 0;
uint32_t current_loop_us = 0;

static bool ConfigButton_IsPressed(void) {
	if (HAL_GPIO_ReadPin(CONFIG_BUTTON_GPIO_Port, CONFIG_BUTTON_Pin)
			!= GPIO_PIN_RESET) {
		return false;
	}

	HAL_Delay(CONFIG_BUTTON_DEBOUNCE_MS);
	return HAL_GPIO_ReadPin(CONFIG_BUTTON_GPIO_Port, CONFIG_BUTTON_Pin)
			== GPIO_PIN_RESET;
}

static void USART2_EnterConfigMode(void) {
	HAL_UART_AbortReceive(&huart2);
	HAL_UART_DeInit(&huart2);

	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;

	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

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
	MX_DMA_Init();
	MX_SPI1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	MX_USART2_UART_Init();
	MX_I2C1_Init();
	MX_SPI2_Init();
	/* USER CODE BEGIN 2 */

	app_mode = ConfigButton_IsPressed() ? APP_MODE_CONFIG : APP_MODE_DRIVE;
	ConfigStatus_t config_load_status = VehicleConfig_Load();

	SH1107_Init();
	DebugUI_Init();
	MainFSM_Init();
	RC_Input_Init();
	Motor_Init();
	Servo_Init();

	if (app_mode == APP_MODE_CONFIG) {
		static const uint8_t config_banner[] =
				"\r\nSTM32 Car configuration mode\r\n";

		Motor_SetPower(0, 0);
		Servo_Center();
		USART2_EnterConfigMode();
		HAL_UART_Transmit(&huart2, (uint8_t*) config_banner,
				sizeof(config_banner) - 1U, 100U);
		if (config_load_status != CONFIG_STATUS_OK) {
			char error_message[80];
			int length = snprintf(error_message, sizeof(error_message),
					"WARNING: %s; defaults loaded\r\n",
					VehicleConfig_StatusText(config_load_status));
			HAL_UART_Transmit(&huart2, (uint8_t *)error_message,
					(uint16_t)length, 100U);
		}
		ConfigConsole_Init(&huart2);
	} else {
		CRSF_Init();
	}

	// Enable DWT Counter for nanosecond precision profiling
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	uint32_t mcu_freq_mhz = HAL_RCC_GetHCLKFreq() / 1000000;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		uint32_t loop_start_cycle = DWT->CYCCNT;

		if (app_mode == APP_MODE_CONFIG) {
			ConfigConsole_Update();
			Motor_SetPower(0, 0);
			Servo_Center();

			if (HAL_GetTick() - last_ui_update >= UI_REFRESH_INTERVAL_MS) {
				if (!SH1107_IsBusy()) {
					last_ui_update = HAL_GetTick();
					DebugUI_Clear();
					DebugUI_PrintLine(0, "%s", ConfigConsole_GetDisplayTitle());
					DebugUI_PrintLine(2, "%s", ConfigConsole_GetDisplayLine());
					DebugUI_PrintLine(4, "UART2 115200 8N1");
					DebugUI_PrintLine(6, "MOTORS DISABLED");
					ConfigStatus_t current_config_status = VehicleConfig_GetLastStatus();
					if (current_config_status != CONFIG_STATUS_OK) {
						DebugUI_PrintLine(8, "BOOT: DEFAULTS");
						DebugUI_PrintLine(10, "%s", VehicleConfig_StatusText(current_config_status));
					}
					DebugUI_Show();
				}
			}
			continue;
		}

		bool new_rc_data = CRSF_Update();
		RC_Input_Update(new_rc_data);
		CRSF_UpdateFailsafe(HAL_GetTick());
		MainFSM_Update(new_rc_data);

		if (HAL_GetTick() - last_ui_update >= UI_REFRESH_INTERVAL_MS) {
			if (!SH1107_IsBusy()) {
				last_ui_update = HAL_GetTick();
				DebugUI_Clear();

				VehicleState_t current_vehicle_state = MainFSM_GetState();

				if (current_vehicle_state == STATE_WAITING) {
					DebugUI_PrintLine(0, "WAITING FOR ELRS...");
				} else if (current_vehicle_state == VEHICLE_CALIBRATING) {
					DebugUI_PrintLine(0, "CALIBRATING STICKS...");
				} else if (current_vehicle_state == STATE_ERROR) {
					DebugUI_PrintLine(0, "! STICK ERROR !");
					DebugUI_PrintLine(2, "CENTER STEERING");
					DebugUI_PrintLine(4, "AND DROP THROTTLE!");
				} else if (current_vehicle_state == STATE_READY) {
					uint16_t thr = MainFSM_GetOutput(CRSF_MAP_THROTTLE);
					uint16_t str = MainFSM_GetOutput(CRSF_MAP_STEERING);

					DebugUI_PrintLine(0, "LQ:%d%%  %ddBm", crsf.link_quality,
							crsf.rssi_dbm);
					DebugUI_PrintLine(1, "THR: %d", thr);
					DebugUI_StickSlider(0, 16, 128, 6, thr);
					DebugUI_PrintLine(3, "STR: %d", str);
					DebugUI_StickSlider(0, 32, 128, 6, str);

					DriveDirection_t current_dir = MainFSM_GetDirection();
					if (current_dir == DIR_FORWARD) {
						DebugUI_PrintLine(5, "DIR: FORWARD");
					} else if (current_dir == DIR_REVERSE) {
						DebugUI_PrintLine(5, "DIR: REVERSE");
					} else {
						DebugUI_PrintLine(5, "DIR: NEUTRAL (HOLD)");
					}

					DriveMode_t current_mode = MainFSM_GetMode();
					if (current_mode == MODE_MANUAL) {
						DebugUI_PrintLine(6, "MODE: MANUAL CONT");
					} else if (current_mode == MODE_RTH_DIRECT) {
						DebugUI_PrintLine(6, "MODE: RTH DIRECT");
					} else {
						DebugUI_PrintLine(6, "MODE: RTH BACKTRACE");
					}
				}

				DebugUI_PrintLine(15, "Loop %d max %d", current_loop_us,
						max_loop_time_us);
				DebugUI_Show();
			}
			max_loop_time_us = 0;
		}

		current_cycles = DWT->CYCCNT - loop_start_cycle;
		current_loop_us = current_cycles / mcu_freq_mhz;
		if (current_loop_us > max_loop_time_us) {
			max_loop_time_us = current_loop_us;
		}

		/* USER CODE END WHILE */
		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 12;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

// !!! This triggers when a full CRSF packet arrives via DMA
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI1) {
		// Push the screen state machine forward asynchronously
		SH1107_FSM_Step();
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART2 && app_mode == APP_MODE_DRIVE) {
		// Prevent permanent DMA lock up caused by Overrun Errors (ORE)
		HAL_UART_AbortReceive(huart);
		extern uint8_t crsf_rx_buffer[];
		HAL_UARTEx_ReceiveToIdle_DMA(&huart2, crsf_rx_buffer, 64);
	}
}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
