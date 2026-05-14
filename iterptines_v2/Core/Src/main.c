/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ssd1306_fonts.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLES     50
#define CHANNELS    3
#define VDD         3.21f
#define COEF        374917UL
#define R_SENSOR    4700.0f

#define AGC_LOW     410U    /* ~10 % skales – stiprinamas labiau   */
#define AGC_HIGH    3686U   /* ~90 % skales – stiprinamas mažiau   */

static const float gain_table[8] = {
    1.0f, 2.0f, 4.0f, 5.0f, 8.0f, 10.0f, 16.0f, 32.0f
};
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

volatile uint16_t adc[CHANNELS * SAMPLES];

float vidurkis_ev1 = 0.0f;   /* apsviestumo kanalas 1 [lx]  */
float vidurkis_ev2 = 0.0f;   /* apsviestumo kanalas 2 [lx]  */
float skirtumas    = 0.0f;   /* ev1 - ev2 [lx]              */
float vidurkis_u1  = 0.0f;   /* itampos kanalas 1 [V]       */
float vidurkis_u2  = 0.0f;   /* itampos kanalas 2 [V]       */

float calib1 = 11.0f;
float calib2 = 11.0f;

volatile uint8_t adc_full      = 0;
volatile uint8_t uart_ready    = 1;
volatile uint8_t uart_received = 0;

typedef enum {
    UART_STATE_IDLE = 0,
    UART_STATE_WAIT_C1,
    UART_STATE_WAIT_C2
} UartState;
volatile UartState uart_state = UART_STATE_IDLE;

char tx[80];
char rx[20];
char string_display[22];

static uint8_t pga_active_ch = 0;   /* 0 = U1, 1 = U2 */

static uint8_t gain_idx_u1 = 0;   /* pradzia: 1x */
static uint8_t gain_idx_u2 = 0;   /* pradzia: 1x */

/* --- AGC: praleisti N ciklu po kanalo perjungimo ---
       > 0  = dar laukiame stabilizacijos (MCP6S22 isejimas dar nestabilizavosi)
       = 0  = duomenys galioja                                                   */
static uint8_t pga_skip_cycle = 0;

/* --- UART ciklo skaitliukas:
       uart_cycle = 0..4  -> apsviestumo eilute  (~200 ms x 5 = ~1 s)
       uart_cycle = 5     -> itampos eilute       (~1.2 s), tada reset i 0      */
static uint8_t uart_cycle = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
static void PGA_SetChannel(uint8_t ch);
static void PGA_SetGain(uint8_t gain_idx);
static void Display_Update(void);
static void UART_SendLight(void);
static void UART_SendVoltage(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void PGA_SetChannel(uint8_t ch)
{
    uint16_t cmd = ((uint16_t)0x41 << 8) | (ch & 0x07);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd, 1, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

static void PGA_SetGain(uint8_t gain_idx)
{
    uint16_t cmd = ((uint16_t)0x40 << 8) | (gain_idx & 0x07);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t *)&cmd, 1, 10);
    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief  Atnaujina OLED ekrana.
 */
static void Display_Update(void)
{
    ssd1306_Fill(Black);

    ssd1306_SetCursor(0, 0);
    snprintf(string_display, sizeof(string_display), "Ev1 = %7.1f lx", vidurkis_ev1);
    ssd1306_WriteString(string_display, Font_6x8, White);

    ssd1306_SetCursor(0, 11);
    snprintf(string_display, sizeof(string_display), "Ev2 = %7.1f lx", vidurkis_ev2);
    ssd1306_WriteString(string_display, Font_6x8, White);

    ssd1306_SetCursor(0, 22);
    snprintf(string_display, sizeof(string_display), "dEv = %7.1f lx", skirtumas);
    ssd1306_WriteString(string_display, Font_6x8, White);

    ssd1306_SetCursor(0, 33);
    snprintf(string_display, sizeof(string_display), "U1  = %7.2f V ", vidurkis_u1);
    ssd1306_WriteString(string_display, Font_6x8, White);

    ssd1306_SetCursor(0, 44);
    snprintf(string_display, sizeof(string_display), "U2  = %7.2f V ", vidurkis_u2);
    ssd1306_WriteString(string_display, Font_6x8, White);

    ssd1306_UpdateScreen();
}

/**
 * @brief  Siuncia apsviestumo duomenis per UART (kas ~0.2 s).
 *         Viena eilute: Ev1, Ev2, dEv.
 */
static void UART_SendLight(void)
{
    if (!uart_ready) return;
    uart_ready = 0;
    int len = snprintf(tx, sizeof(tx),
        "Ev1 = %.1f lx, Ev2 = %.1f lx, dEv = %.1f lx\r\n",
        vidurkis_ev1, vidurkis_ev2, skirtumas);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx, (uint16_t)len);
}

/**
 * @brief  Siuncia itampos duomenis per UART (kas ~1.2 s).
 *         Viena eilute: U1, U2.
 */
static void UART_SendVoltage(void)
{
    if (!uart_ready) return;
    uart_ready = 0;
    int len = snprintf(tx, sizeof(tx),
        "                                                    U1 = %.2f V,  U2 = %.2f V\r\n",
        vidurkis_u1, vidurkis_u2);
    HAL_UART_Transmit_IT(&huart2, (uint8_t *)tx, (uint16_t)len);
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_ADC_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

    HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);

    /* MCP6S22 pradine konfiguracija: kanalas 0 (U1), stiprinimas 1x.*/
    PGA_SetGain(0);
    PGA_SetChannel(0);
    pga_active_ch  = 0;
    pga_skip_cycle = 2;   /* Leisti stabilizuotis po pradines konfiguracijos */

    if (HAL_ADCEx_Calibration_Start(&hadc, ADC_SINGLE_ENDED) != HAL_OK)
        Error_Handler();

    HAL_ADC_Start_DMA(&hadc, (uint32_t *)adc, SAMPLES * CHANNELS);
    HAL_TIM_Base_Start(&htim2);

    ssd1306_Init();
    ssd1306_Fill(Black);
    ssd1306_SetCursor(10, 24);
    ssd1306_WriteString("Duomenys", Font_7x10, White);
    ssd1306_SetCursor(10, 36);
    ssd1306_WriteString("renkami...", Font_7x10, White);
    ssd1306_UpdateScreen();

    HAL_UART_Receive_IT(&huart2, (uint8_t *)rx, 6);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

    uint32_t tick_display = 0;

    while (1)
    {
        if (uart_received)
        {
            uart_received = 0;

            switch (uart_state)
            {
                case UART_STATE_IDLE:
                {
                    if (strncmp(rx, "config", 6) == 0)
                    {
                        uart_state = UART_STATE_WAIT_C1;
                        const char *prompt =
                            "\r\nIveskite Ev1 kalibravimo koeficienta (2 skaitmenys, pvz. 15):\r\n";
                        HAL_UART_Transmit(&huart2, (uint8_t *)prompt,
                                          (uint16_t)strlen(prompt), 200);
                        HAL_UART_Receive_IT(&huart2, (uint8_t *)rx, 2);
                    }
                    else
                    {
                        memset(rx, 0, sizeof(rx));
                        HAL_UART_Receive_IT(&huart2, (uint8_t *)rx, 6);
                    }
                    break;
                }

                case UART_STATE_WAIT_C1:
                {
                    int val = 0;
                    if (rx[0] >= '0' && rx[0] <= '9' &&
                        rx[1] >= '0' && rx[1] <= '9')
                    {
                        val = (rx[0] - '0') * 10 + (rx[1] - '0');
                        calib1 = (float)val;
                    }
                    uart_state = UART_STATE_WAIT_C2;
                    const char *prompt2 =
                        "\r\nIveskite Ev2 kalibravimo koeficienta (2 skaitmenys):\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t *)prompt2,
                                      (uint16_t)strlen(prompt2), 200);
                    HAL_UART_Receive_IT(&huart2, (uint8_t *)rx, 2);
                    break;
                }

                case UART_STATE_WAIT_C2:
                {
                    int val = 0;
                    if (rx[0] >= '0' && rx[0] <= '9' &&
                        rx[1] >= '0' && rx[1] <= '9')
                    {
                        val = (rx[0] - '0') * 10 + (rx[1] - '0');
                        calib2 = (float)val;
                    }
                    uart_state = UART_STATE_IDLE;
                    const char *done =
                        "\r\nKalibracija atlikta. Tesiamas matavimas.\r\n\r\n";
                    HAL_UART_Transmit(&huart2, (uint8_t *)done,
                                      (uint16_t)strlen(done), 200);
                    memset(rx, 0, sizeof(rx));
                    HAL_UART_Receive_IT(&huart2, (uint8_t *)rx, 6);
                    break;
                }

                default:
                    uart_state = UART_STATE_IDLE;
                    HAL_UART_Receive_IT(&huart2, (uint8_t *)rx, 6);
                    break;
            }
        }

        if (adc_full)
        {
            adc_full = 0;

            uint32_t suma_ev1 = 0, suma_ev2 = 0;
            for (int i = 0; i < SAMPLES * CHANNELS; i += 3)
            {
                suma_ev1 += adc[i + 1];
                suma_ev2 += adc[i + 2];
            }
            float avg_ev1_raw = (float)suma_ev1 / SAMPLES;
            float avg_ev2_raw = (float)suma_ev2 / SAMPLES;

            vidurkis_ev1 = (avg_ev1_raw * VDD / 4095.0f) / R_SENSOR * (float)COEF * calib1;
            vidurkis_ev2 = (avg_ev2_raw * VDD / 4095.0f) / R_SENSOR * (float)COEF * calib2;
            skirtumas    = vidurkis_ev1 - vidurkis_ev2;

            const float VOLTAGE_DIV = 7.788f;

            uint32_t suma_u = 0;
            for (int i = 0; i < SAMPLES * CHANNELS; i += 3)
                suma_u += adc[i];

            uint32_t avg_u_raw = suma_u / SAMPLES;

            if (pga_skip_cycle > 0)
            {
                /* Stabilizacijos laukimas */
                pga_skip_cycle--;
            }
            else
            {
                uint8_t *p_gain = (pga_active_ch == 0) ? &gain_idx_u1 : &gain_idx_u2;
                uint8_t gain_changed = 0;

                if (avg_u_raw < AGC_LOW && *p_gain < 7)
                {
                    (*p_gain)++;
                    gain_changed = 1;
                }
                else if (avg_u_raw > AGC_HIGH && *p_gain > 0)
                {
                    (*p_gain)--;
                    gain_changed = 1;
                }

                if (gain_changed)
                {
                    /* Stiprinimas pasikeite – reikia 1 ciklo stabilizacijai.
                       Šio ciklo rezultato neissaugome. */
                    PGA_SetGain(*p_gain);
                    pga_skip_cycle = 1;
                }
                else
                {
                    float v_adc  = (float)avg_u_raw * VDD / 4095.0f;
                    float v_real = (v_adc / gain_table[*p_gain]) * VOLTAGE_DIV;

                    if (pga_active_ch == 0)
                        vidurkis_u1 = v_real;
                    else
                        vidurkis_u2 = v_real;

                    pga_active_ch ^= 1;
                    PGA_SetGain((pga_active_ch == 0) ? gain_idx_u1 : gain_idx_u2);
                    PGA_SetChannel(pga_active_ch);
                    pga_skip_cycle = 2;
                }
            }

            if (uart_state == UART_STATE_IDLE)
            {
                if (uart_cycle < 5)
                {
                    UART_SendLight();
                    uart_cycle++;
                }
                else
                {
                    UART_SendVoltage();
                    uart_cycle = 0;
                }
            }
        }

        if ((HAL_GetTick() - tick_display) >= 2000UL)
        {
            tick_display = HAL_GetTick();
            Display_Update();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.OversamplingMode = DISABLE;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc.Init.DMAContinuousRequests = ENABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerFrequencyMode = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x00B07CB4;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_1LINE;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc_cb)
{
    (void)hadc_cb;
    adc_full = 1;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart_cb)
{
    (void)huart_cb;
    uart_ready = 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart_cb)
{
    (void)huart_cb;
    uart_received = 1;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    __disable_irq();
    while (1)
    {
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        HAL_Delay(200);
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
