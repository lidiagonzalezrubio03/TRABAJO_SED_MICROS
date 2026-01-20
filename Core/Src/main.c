/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Proyecto Simon Dice - STM32F407
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h> // Para rand() y srand()

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
int pasos_por_nivel[5] = {2, 3, 4, 5, 6};
int nivel_actual = 0;
int secuencia[50]; // Almacena la secuencia de colores

// LED
uint16_t leds_juego[5] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};
#define LED_VERDE GPIO_PIN_5
#define LED_ROJO  GPIO_PIN_6

// Botones
uint16_t botones[5] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12};

volatile int boton_pulsado_flag = -1;
volatile uint32_t ultimo_tiempo_pulsacion = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void iniciar_semilla_aleatoria(void);
void reproducir_secuencia(int pasos, int velocidad);
void efecto_victoria(void);
void efecto_derrota(void);
int obtener_indice_boton(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//Interrupciones (cuando se pulsa botón)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t tiempo_actual = HAL_GetTick();
	if (tiempo_actual - ultimo_tiempo_pulsacion >300){
		boton_pulsado_flag = obtener_indice_boton(GPIO_Pin);
		ultimo_tiempo_pulsacion = tiempo_actual;
	}

}

//convierte pin en indice
int obtener_indice_boton(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_0) return 0;
    if(GPIO_Pin == GPIO_PIN_1) return 1;
    if(GPIO_Pin == GPIO_PIN_10) return 2;
    if(GPIO_Pin == GPIO_PIN_11) return 3;
    if(GPIO_Pin == GPIO_PIN_12) return 4;
    return -1;
}

//ADC
// Lee ruido electrico de un pin al aire para que el juego sea aleatorio
void iniciar_semilla_aleatoria(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    uint32_t ruido = HAL_ADC_GetValue(&hadc1);
    srand(ruido);
    HAL_ADC_Stop(&hadc1);
}

void reproducir_secuencia(int pasos, int velocidad) {
    HAL_Delay(500);
    for(int i = 0; i < pasos; i++) {
        HAL_GPIO_WritePin(GPIOA, leds_juego[secuencia[i]], GPIO_PIN_SET);
        HAL_Delay(velocidad);
        HAL_GPIO_WritePin(GPIOA, leds_juego[secuencia[i]], GPIO_PIN_RESET);
        HAL_Delay(400);
    }
}

void efecto_derrota(void) {
    HAL_GPIO_WritePin(GPIOA, LED_ROJO, GPIO_PIN_SET);
    HAL_Delay(1000);
    HAL_GPIO_WritePin(GPIOA, LED_ROJO, GPIO_PIN_RESET);
}

void efecto_victoria(void) {
    for(int i=0; i<3; i++) {
        HAL_GPIO_WritePin(GPIOA, LED_VERDE, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(GPIOA, LED_VERDE, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();

  /* USER CODE BEGIN 2 */

  iniciar_semilla_aleatoria();


  HAL_TIM_Base_Start(&htim2);

  // Parpadeo del principio
  efecto_victoria();
  nivel_actual = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      int num_pasos = pasos_por_nivel[nivel_actual];

      // Velocidad aumenta con el nivel
      int velocidad = 600 - (nivel_actual * 100);
      if (velocidad < 400) velocidad = 400;

      // secuencia nueva
      for(int i=0; i<num_pasos; i++){
          secuencia[i] = rand() % 5;
      }

      reproducir_secuencia(num_pasos, velocidad);
      boton_pulsado_flag = -1;
      HAL_Delay(50)

      //pasos jugador
      int error = 0;
      for(int i = 0; i < num_pasos; i++) {
          boton_pulsado_flag = -1;

          // REVISAR ESTO
          __HAL_TIM_SET_COUNTER(&htim2, 0); // Reiniciar cronometro

          // Esperar pulsar boton o 30 segundos
          while(boton_pulsado_flag == -1) {
              if (__HAL_TIM_GET_COUNTER(&htim2) > 30000) {
                  error = 1;
                  break;
              }
          }

          if(error) break;

          // acierto
          if(boton_pulsado_flag == secuencia[i]) {
              // enciende el led pulsado
              HAL_GPIO_WritePin(GPIOA, leds_juego[boton_pulsado_flag], GPIO_PIN_SET);
              HAL_Delay(300);
              HAL_GPIO_WritePin(GPIOA, leds_juego[boton_pulsado_flag], GPIO_PIN_RESET);
          } else {
              error = 1; //mal
              break;
          }
      }

      // solucion
      if(error) {
          efecto_derrota();
          nivel_actual = 0; // Reiniciar
          HAL_Delay(1000);
      } else {
          // juego ganado
          HAL_GPIO_WritePin(GPIOA, LED_VERDE, GPIO_PIN_SET);
          HAL_Delay(500);
          HAL_GPIO_WritePin(GPIOA, LED_VERDE, GPIO_PIN_RESET);

          if(nivel_actual < 4) {
              nivel_actual++;
          } else {
              efecto_victoria(); // efecto ganar
              nivel_actual = 0;
          }
          HAL_Delay(1000);
      }
  }
  /* USER CODE END 3 */
}

void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  HAL_ADC_Init(&hadc1);
  sConfig.Channel = ADC_CHANNEL_7; // Ajusta según tu pin ADC
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_TIM2_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  // Salidas
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Entradas
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING; // Interrupción en flanco subida
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Activar interrupciones en NVIC
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void Error_Handler(void) {
  __disable_irq();
  while (1) {}
}
