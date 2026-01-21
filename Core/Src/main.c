/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : main program body
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdlib.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
int pasos_por_nivel[5] = {2, 3, 4, 5, 6};
int nivel_actual = 0;
int secuencia[50];

uint16_t leds_juego[5] = {GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3, GPIO_PIN_4};
#define LED_VERDE GPIO_PIN_5
#define LED_ROJO  GPIO_PIN_6

volatile int boton_pulsado_flag = -1;
volatile uint32_t ultimo_tiempo_pulsacion = 0;

#define LCD_ADDR (0x27 << 1)
/* USER CODE END PV */

/* Prototypes ----------------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);

void iniciar_semilla_aleatoria(void);
void reproducir_secuencia(int pasos, int velocidad);
void efecto_derrota(void);
void barrido_celebracion(void);
int obtener_indice_boton(uint16_t GPIO_Pin);

// Funciones LCD para la pantalla
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_init(void);
void lcd_send_string(char *str);
void lcd_put_cur(int row, int col);
void lcd_clear(void);

/* USER CODE BEGIN 0 */
// LCD
void lcd_send_cmd(char cmd) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; data_t[1] = data_u | 0x08;
    data_t[2] = data_l | 0x0C; data_t[3] = data_l | 0x08;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, (uint8_t *)data_t, 4, 100);
}

void lcd_send_data(char data) {
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; data_t[1] = data_u | 0x09;
    data_t[2] = data_l | 0x0D; data_t[3] = data_l | 0x09;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_ADDR, (uint8_t *)data_t, 4, 100);
}

void lcd_init(void) {
    HAL_Delay(50);
    lcd_send_cmd(0x30); HAL_Delay(5);
    lcd_send_cmd(0x30); HAL_Delay(1);
    lcd_send_cmd(0x32); HAL_Delay(10);
    lcd_send_cmd(0x28); lcd_send_cmd(0x0C); lcd_send_cmd(0x01);
    HAL_Delay(2);
}

void lcd_send_string(char *str) {
    while (*str) lcd_send_data(*str++);
}

void lcd_put_cur(int row, int col) {
    switch (row) {
        case 0: col |= 0x80; break;
        case 1: col |= 0xC0; break;
    }
    lcd_send_cmd(col);
}

void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

//luces cuando se gana
void barrido_celebracion(void) {
    for(int vueltas = 0; vueltas < 5; vueltas++) {
        for(int i = 0; i < 5; i++) {
            HAL_GPIO_WritePin(GPIOA, leds_juego[i], GPIO_PIN_SET);
            HAL_Delay(50);
        }
        HAL_GPIO_WritePin(GPIOA, LED_VERDE, GPIO_PIN_SET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(GPIOA, 0x7F, GPIO_PIN_RESET); // Apaga PA0 a PA6
        HAL_Delay(100);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    uint32_t tiempo_actual = HAL_GetTick();
    if (tiempo_actual - ultimo_tiempo_pulsacion > 50) {
        boton_pulsado_flag = obtener_indice_boton(GPIO_Pin);
        ultimo_tiempo_pulsacion = tiempo_actual;
    }
}

int obtener_indice_boton(uint16_t GPIO_Pin) {
    if(GPIO_Pin == GPIO_PIN_0) return 0;
    if(GPIO_Pin == GPIO_PIN_1) return 1;
    if(GPIO_Pin == GPIO_PIN_10) return 2;
    if(GPIO_Pin == GPIO_PIN_11) return 3;
    if(GPIO_Pin == GPIO_PIN_12) return 4;
    return -1;
}

void iniciar_semilla_aleatoria(void) {
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 100);
    srand(HAL_ADC_GetValue(&hadc1));
    HAL_ADC_Stop(&hadc1);
}

void reproducir_secuencia(int pasos, int velocidad) {
    lcd_put_cur(1, 0);
    lcd_send_string("MIRA LOS LEDS!");
    HAL_Delay(500);
    for(int i = 0; i < pasos; i++) {
        HAL_GPIO_WritePin(GPIOA, leds_juego[secuencia[i]], GPIO_PIN_SET);
        HAL_Delay(velocidad);
        HAL_GPIO_WritePin(GPIOA, leds_juego[secuencia[i]], GPIO_PIN_RESET);
        HAL_Delay(300);
    }
    lcd_put_cur(1, 0);
    lcd_send_string("TE TOCA JUGAR");
}

void efecto_derrota(void) {
    lcd_clear();
    lcd_send_string("FALLO, VUELVE A ");
    lcd_put_cur(1, 0);
    lcd_send_string("   INTENTARLO  ");
    HAL_GPIO_WritePin(GPIOA, LED_ROJO, GPIO_PIN_SET);
    HAL_Delay(2000);
    HAL_GPIO_WritePin(GPIOA, LED_ROJO, GPIO_PIN_RESET);
}
/* USER CODE END 0 */

int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  lcd_init();
  lcd_send_string("  SIMON DICE  ");
  lcd_put_cur(1, 0);
  lcd_send_string("   A JUGAR  ");

  iniciar_semilla_aleatoria();
  HAL_TIM_Base_Start(&htim2);
  HAL_Delay(2000);
  nivel_actual = 0;
  /* USER CODE END 2 */

  while (1) {
      lcd_clear();
      char buffer[16];
      sprintf(buffer, "NIVEL: %d", nivel_actual + 1);
      lcd_send_string(buffer);

      int num_pasos = pasos_por_nivel[nivel_actual];
      int velocidad = 800 - (nivel_actual * 100);
      if (velocidad < 300) velocidad = 300;

      for(int i=0; i<num_pasos; i++){
          secuencia[i] = rand() % 5;
      }

      reproducir_secuencia(num_pasos, velocidad);

      int error = 0;
      for(int i = 0; i < num_pasos; i++) {
          boton_pulsado_flag = -1;
          __HAL_TIM_SET_COUNTER(&htim2, 0);

          while(boton_pulsado_flag == -1) {
              if (__HAL_TIM_GET_COUNTER(&htim2) > 30000) {
                  error = 1; break;
              }
          }
          if(error) break;

          if(boton_pulsado_flag == secuencia[i]) {
              HAL_GPIO_WritePin(GPIOA, leds_juego[boton_pulsado_flag], GPIO_PIN_SET);
              HAL_Delay(300);
              HAL_GPIO_WritePin(GPIOA, leds_juego[boton_pulsado_flag], GPIO_PIN_RESET);
          } else {
              error = 1; break;
          }
      }

      if(error) {
          efecto_derrota();
          nivel_actual = 0;
          HAL_Delay(1000);
      } else {
          if(nivel_actual < 4) {
              lcd_put_cur(1, 0);
              lcd_send_string("A POR EL SIGUIENTE ");
              lcd_put_cur(1, 0);
                lcd_send_string(" NIVEL!  ");
              nivel_actual++;
              HAL_Delay(1500);
          } else {
              lcd_clear();
              lcd_send_string("HAS GANADO!!!");
              barrido_celebracion();
              nivel_actual = 0;
              HAL_Delay(2000);
          }
      }
  }
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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0);
}

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
  HAL_I2C_Init(&hi2c1);
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
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_TIM2_Init(void) {
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 15999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(&htim2);
}

static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
