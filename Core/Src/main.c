/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "dma2d.h"
#include "ltdc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "time.h"
#include "fonts.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
enum Direction {
	LEFT = 0,
	RIGHT = 1,
	UP = 2,
	DOWN = 3
};

enum Colors {
	EMPTY = 0,
	SNAKE = 1,
	FRUIT = 2
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define LCD_FRAME_BUFFER          SDRAM_DEVICE_ADDR

uint16_t gameSpeed = 500; // delay between ticks in ms
uint16_t width = 20; // game width
uint16_t height = 20; // game height
uint16_t x; // head x
uint16_t y; // head y
uint16_t lstX; // tail x
uint16_t lstY; // tail y
uint16_t frtX; // fruit x
uint16_t frtY; // fruit y
uint16_t deathX; // used in death calculations
uint16_t deathY; // used in death calculations
uint16_t len = 2; // snake lenght
uint16_t dir[256] = {}; // direction of body parts, [0] -> head, [len-1] -> tail
uint16_t scr = 0; // score
char score[10] = {};
uint8_t fruitEaten = 0; // fruit flag
uint8_t run = 1;

static TS_StateTypeDef  TS_State;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void drawSquare(uint16_t x, uint16_t y, uint8_t color) {
	if ( color == EMPTY ) {
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	} else if ( color == SNAKE ) {
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	} else if ( color == FRUIT ) {
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
	}
	BSP_LCD_FillRect((x*9)+150, (y*9)+46, 9, 9);
}


void updateHead() {
	if ( dir[0] == LEFT ) {	// HEAD MOVEMENT
		x--;
	} else if ( dir[0] == RIGHT ) {
		x++;
	} else if ( dir[0] == DOWN ) {
		y++;
	} else if ( dir[0] == UP ) {
		y--;
	}
	if (BSP_LCD_ReadPixel((x*9)+150, (y*9)+46) == LCD_COLOR_GREEN) {
		run = 0;
	}
	drawSquare(x, y, SNAKE);
}


void updateTail() {	// kill tail block, call before updateMovement
	if (fruitEaten == 0) {
		drawSquare(lstX, lstY, EMPTY);
		if ( dir[len-1] == LEFT ) {	// TAIL MOVEMENT
			lstX--;
		} else if ( dir[len-1] == RIGHT ) {
			lstX++;
		} else if ( dir[len-1] == DOWN ) {
			lstY++;
		} else if ( dir[len-1] == UP ) {
			lstY--;
		}
		for (int i = len-1; i>0; i--) {
			dir[i] = dir[i-1];
		}
	} else {
		fruitEaten = 0;
		for (int i = len; i>0; i--) {
			dir[i] = dir[i-1];
		}
	}
}


void newFruit() {	// randomize position of new fruit
	do {
		frtX = (rand() % (width-1)) + 1;
		frtY = (rand() % (height-1)) + 1;
	} while(( frtX == x && frtY == y ) || ( BSP_LCD_ReadPixel((frtX*9)+150, (frtY*9)+46) == LCD_COLOR_GREEN ));
	drawSquare(frtX, frtY, FRUIT);
}

void endGame() {
	run = 0;
	BSP_LCD_Clear(LCD_COLOR_RED);
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	//BSP_LCD_DisplayStringAt(240, 136, "you lost", CENTER_MODE);
}

void checkDeath() {
	if ((x>width-1) || (x<0) || (y>height-1) || (y<0)) {
		run = 0;
	}
}

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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_DMA2D_Init();
  MX_FMC_Init();
  MX_LTDC_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  BSP_TS_Init(480,272);
  BSP_LCD_Init();
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER,LCD_FRAME_BUFFER);
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER);
  BSP_LCD_DisplayOn();
  BSP_LCD_Clear(LCD_COLOR_BLACK);
  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
  BSP_LCD_DrawRect(149, 45, 181, 181);
  BSP_LCD_SetFont(&Font16);
  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);

  dir[0] = RIGHT;
  dir[1] = RIGHT;
  x = width/2; // head x
  y = height/2; // head y
  lstX = x-1; // tail x
  lstY = y; // tail y
  HAL_TIM_Base_Start_IT(&htim2);
  srand( TIM2->CNT );
  newFruit();

  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  BSP_TS_GetState(&TS_State);
	  if ( frtX == x && frtY == y ) {
		  fruitEaten = 1;
		  srand( TIM2->CNT );
		  newFruit();
		  scr++;
		  len++;
	  }

	  if ( TS_State.touchDetected ) {
		  float X = TS_State.touchX[0];
		  float Y = TS_State.touchY[0];

		  if (( Y > 0.567 * X ) && ( Y > -0.567 * X + 272 ) && ( dir[1] != UP )) {
			  dir[0] = DOWN;
		  } else if (( Y < 0.567 * X ) && ( Y < -0.567 * X + 272 ) && ( dir[1] != DOWN )) {
			  dir[0] = UP;
		  } else if (( Y > 0.567 * X ) && ( Y < -0.567 * X + 272 ) && ( dir[1] != RIGHT )) {
			  dir[0] = LEFT;
		  } else if (( Y < 0.567 * X ) && ( Y > -0.567 * X + 272 ) && ( dir[1] != LEFT )) {
			  dir[0] = RIGHT;
		  }
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
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
	if (htim->Instance == TIM3) {
		if ( run == 1 ) {
			updateTail();
			updateHead();
			checkDeath();
			itoa(scr, score, 10);
			BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
			BSP_LCD_DisplayStringAt(150, 100, score, CENTER_MODE);
			if (run == 0) {
				endGame();
				BSP_LCD_SetBackColor(LCD_COLOR_RED);
				BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
				BSP_LCD_DisplayStringAt(0, 136, score, CENTER_MODE);
			}
		}
	}
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
