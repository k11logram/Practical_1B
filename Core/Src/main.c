/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "stm32f0xx.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MAX_ITER 100
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//TODO: Define and initialise the global varibales required
/*
  start_time
  end_time
  execution_time 
  checksum: should be uint64_t
  initial width and height maybe or you might opt for an array??
*/
#define SCALE 1000000LL // Fixed-point scale factor (S = 10^6)

uint32_t start_time = 0;//utilized in benchmarking
uint32_t end_time = 0;//utilized in benchmarking
uint32_t execution_time = 0;//measure of how long ittook for the program to finish executing
uint64_t checksum = 0; //the total number of coordinates that fall into the mandelbrot set

int width  = 0;
int height = 0;
int maxIter = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations);
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations);


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
  /* USER CODE BEGIN 2 */
  //TODO: Turn on LED 0 to signify the start of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Turn on LED0 by setting pin.

  //TODO: Record the start time
  start_time = HAL_GetTick();
  
  
  //TODO: Call the Mandelbrot Function and store the output in the checksum variable defined initially
  checksum  = calculate_mandelbrot_fixed_point_arithmetic(128,128,MAX_ITER);
  

  //TODO: Record the end time
  end_time = HAL_GetTick();
  

  //TODO: Calculate the execution time
  execution_time = end_time - start_time;
  

  //TODO: Turn on LED 1 to signify the end of the operation
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET); // Turn on LED1 by setting pin.

  //TODO: Hold the LEDs on for a 1s delay
  HAL_Delay(0b1111101000);

  //TODO: Turn off the LEDs
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET); // Turning OFF LED's.
  

  /* USER CODE END 2 */

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//TODO: Mandelbroat using variable type integers and fixed point arithmetic
uint64_t calculate_mandelbrot_fixed_point_arithmetic(int width, int height, int max_iterations){
  uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation

   /*we are going through the complex plane multiple times
   any coordinate left after that iteration falls under the mandelbrot set*/
    for(int y = 0 ; y < height; y++){
    	for(int x = 0 ; x < width ; x++ ){
    		int64_t x0 = (((int64_t)((x * 3500000LL) / width)) - 2500000LL); // we are trying keeping the set in between [-2.5 ,1.0]
    		int64_t y0 = (((int64_t)((y * 2000000LL) / height)) - 1000000LL);//we are keeping or taking the cooridantes in between [-1,1]
    		int64_t xi = 0; int64_t yi = 0;
    		int iteration = 0;
    		while((iteration <  max_iterations) && ((((xi * xi)/ SCALE) + ((yi * yi)/ SCALE)) <= 4000000LL)){
    			int temp  = ((xi * xi)/ SCALE) - ((yi * yi)/ SCALE);//This is the imaginary representation of Z^2
    			yi = ((2LL*(xi * yi)/ SCALE) + y0);//this is the real z^2
    			xi = temp + x0;//this is the real representation of z^2 +c

    			iteration++;//keeping track of the number of times we have done iterations
    		}
    		mandelbrot_sum += iteration;
    	}
    }
    return mandelbrot_sum;

}

//TODO: Mandelbroat using variable type double(double precision)
uint64_t calculate_mandelbrot_double(int width, int height, int max_iterations){
    uint64_t mandelbrot_sum = 0;
    //TODO: Complete the function implementation
    //everything  is in (double precision)
   /*we are going through the complex plane multiple times
   any coordinate left after that iteration falls under the mandelbrot set*/
    for(int y = 0 ; y < height ; y++){
    	for(int x = 0 ; x < width ; x++ ){
    		double x0  = ((((double)(x / width)) * 3.5 ) - 2.5);// we are keeping the set in between [-2.5 ,1.0]
    		double y0  = ((((double)(y / width)) * 2.0 ) - 1.0);//we are keeping or taking the cooridantes in between [-1,1]
    		double xi = 0.0; double yi = 0.0;
    		double iteration =  0;//keeping track of the number of times we have done iterations
    		while((iteration < max_iterations) && (((xi * xi) + (yi * yi)) <= 4.0)){
    			double temp = ((xi * xi) - (yi * yi));//This is the imaginary representation of Z^2
    			yi  = (2.0 * (xi * yi) + y0);//this is the real z^2
    			xi = (temp + x0);//this is the real representation of z^2 +c
    			iteration++;
    		}
    		mandelbrot_sum += iteration;
    	}
    }
    return mandelbrot_sum;
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
