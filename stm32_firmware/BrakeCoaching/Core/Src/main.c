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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
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
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
#define RX_LINE_MAX 32                     // 한 줄(응답) 버퍼의 최대 길이를 32바이트로 정의하는 매크로 상수

volatile uint8_t rx_byte;                  // 인터럽트로 한 바이트씩 새로 받아올 때 임시로 담아두는 1바이트짜리 변수
                                            // (volatile: 인터럽트 안에서 값이 바뀌므로, 컴파일러가 "안 바뀌는 값"이라고 착각해 최적화해버리는 걸 방지)

volatile char rx_line[RX_LINE_MAX];        // 받은 바이트들을 한 글자씩 이어붙여서 "한 줄"을 완성해가는 문자 배열(버퍼)
                                            // (volatile: 인터럽트 안에서 계속 채워지므로 최적화 방지 목적으로 붙임)

volatile uint8_t rx_index = 0;             // rx_line 배열에서 지금 몇 번째 칸까지 채웠는지를 기억하는 위치 표시 변수, 시작값은 0(맨 앞)

volatile uint8_t line_ready = 0;           // "한 줄이 완성됐다"는 신호를 메인 루프에 전달하는 깃발(flag) 변수, 시작값 0=아직 안 됨, 1=완성됨

volatile int16_t last_speed_kmh = -1;      // 가장 최근에 파싱에 성공한 속도값(km/h)을 저장해두는 변수
                                            // 시작값 -1은 "아직 한 번도 유효한 값을 못 받았다"는 뜻으로 정한 특수값

#define RADAR_FRAME_LEN 7                  // 레이더 프레임 하나의 길이를 7바이트로 정의 (임시 가정치, 실물 확인 후 수정 필요)

volatile uint8_t radar_byte;               // 레이더에서 인터럽트로 한 바이트씩 받아올 임시 저장 변수
volatile uint8_t radar_frame[RADAR_FRAME_LEN];  // 레이더 프레임 7바이트를 모아 담는 배열
volatile uint8_t radar_index = 0;          // radar_frame에 지금 몇 번째 바이트까지 채웠는지 가리키는 인덱스
volatile uint8_t radar_frame_ready = 0;    // 프레임 하나가 완성됐다는 신호를 메인루프에 전달하는 깃발

volatile float radar_distance_m = -1.0f;   // 파싱된 최신 거리값(미터), -1.0은 "아직 유효값 없음"을 의미
volatile float radar_speed_kmh = 0.0f;     // 파싱된 최신 상대속도값(km/h), 양수=멀어짐/음수=가까워짐 (부호 규칙은 실물 확인 후 조정)
volatile int8_t radar_angle_deg = 0;       // 파싱된 최신 각도값(도)

// ===== Pi 통신(USART2) 관련 =====
#define PI_RX_LINE_MAX 32                  // Pi로부터 받는 한 줄의 최대 길이

volatile uint8_t pi_rx_byte;               // Pi에서 인터럽트로 한 바이트씩 받을 임시 버퍼
volatile char pi_rx_line[PI_RX_LINE_MAX];  // Pi가 보낸 한 줄을 모으는 버퍼
volatile uint8_t pi_rx_index = 0;          // pi_rx_line에 지금 몇 번째까지 채웠는지
volatile uint8_t pi_line_ready = 0;        // Pi로부터 한 줄 다 받았다는 신호

volatile float pi_brightness_delta = 0.0f; // Pi가 보내준 밝기 델타값 (파싱 결과 저장)
volatile uint8_t pi_anomaly_flag = 0;      // Pi가 보내준 이상여부 (0=정상, 1=이상)
volatile uint8_t pi_data_valid = 0;        // Pi로부터 유효한 데이터를 받은 적 있는지 (아직 한 번도 못 받았으면 0)

uint32_t last_pi_send_tick = 0;            // STM32가 Pi한테 마지막으로 데이터 보낸 시각(ms) 기록용

// ===== 판정 로직(이상감지) 관련 =====
#define SPEED_HISTORY_LEN 10                // 앞차 절대속도 이력을 몇 개까지 저장할지 (최근 몇 번의 측정값)

typedef struct {                            // 속도 측정값 하나를 저장하는 구조체 (값+측정시각을 묶어서 관리)
    float speed_kmh;                        // 그 시점의 앞차 절대속도
    uint32_t tick;                          // 그 값을 측정한 시각(ms 단위, HAL_GetTick() 값)
} SpeedSample;

volatile SpeedSample front_speed_history[SPEED_HISTORY_LEN];  // 앞차 절대속도 이력을 담는 배열(원형버퍼처럼 사용)
volatile uint8_t history_index = 0;         // 다음에 이력을 저장할 배열 위치(인덱스)
volatile uint8_t history_count = 0;         // 지금까지 몇 개의 이력이 쌓였는지 (최대 SPEED_HISTORY_LEN)

volatile float front_car_speed_kmh = 0.0f;  // 계산된 앞차 절대속도 (레이더+내차속도 결합 결과)

#define DECEL_THRESHOLD_KMH 5.0f            // 이 정도(km/h) 이상 감속하면 "감속 중"으로 판단하는 기준치
#define DECEL_WINDOW_MS 1000                // 이 시간(ms) 동안의 속도 변화를 비교해서 감속 여부 판단
#define ANOMALY_PERSIST_MS 500              // 이상 상황이 이 시간(ms) 이상 지속돼야 최종 "이상"으로 확정

volatile uint8_t is_decelerating = 0;       // 지금 앞차가 감속 중인지 여부 (0=아니오, 1=예)
volatile uint32_t anomaly_start_tick = 0;   // "이상 의심" 상황이 시작된 시각 (지속시간 측정용)
volatile uint8_t anomaly_confirmed = 0;     // 최종적으로 "이상"이 확정됐는지 (0.5초 지속 필터 통과 여부)

#define LCD_WIDTH  240                      // 디스플레이 가로 픽셀 수
#define LCD_HEIGHT 240                      // 디스플레이 세로 픽셀 수
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void ELM327_RequestSpeed(void);            // ELM327에게 "지금 속도 알려줘"라는 명령을 보내는 함수가 있다고 미리 알려주는 선언(원형)

int ParseSpeedResponse(const char *raw);   // ELM327이 보낸 응답 문자열(raw)을 받아서 정수 속도값으로 바꿔주는 함수의 선언(원형)
                                            // 매개변수 raw: 파싱할 원본 문자열 / 반환값: 성공시 속도값(0 이상), 실패시 -1

void ParseRadarFrame(void);                // 완성된 레이더 프레임을 해석해서 거리/속도/각도로 변환하는 함수 선언

void Pi_SendRadarData(void);                // STM32가 Pi에게 레이더 데이터(거리, 상대속도)를 보내는 함수
int Pi_ParseBrightnessResponse(const char *raw);  // Pi가 보낸 응답 문자열을 파싱하는 함수

void UpdateFrontCarSpeed(void);             // 레이더+내차속도를 결합해서 앞차 절대속도를 계산하고 이력에 저장하는 함수
uint8_t CheckDeceleration(void);            // 최근 이력을 보고 "지금 감속 중인지" 판정하는 함수
void UpdateAnomalyJudgement(void);          // 감속여부+밝기이상여부를 종합해서 최종 이상감지 판정을 내리는 함수

void LCD_Reset(void);                       // LCD를 하드웨어적으로 리셋시키는 함수
void LCD_WriteCommand(uint8_t cmd);         // LCD에 "명령"을 보내는 함수
void LCD_WriteData(uint8_t data);           // LCD에 "데이터"를 보내는 함수
void LCD_Init(void);                        // LCD 초기화(전원 인가 후 켜지도록 설정하는) 함수
void LCD_FillScreen(uint16_t color);        // 화면 전체를 특정 색으로 채우는 함수

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
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&radar_byte, 1);  // USART1(레이더) 인터럽트 수신 시작(기존 그대로 둠)

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 부저 PWM 출력 시작 (소리 나기 시작)

  HAL_Delay(250);                           // 3000밀리초(3초) 동안 여기서 그냥 멈춰서 기다림 (다른 아무것도 안 하고 순수하게 대기)

  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);   // 3초 지났으니 PWM 출력 정지 (소리 끊김)

  HAL_UART_Receive_IT(&huart2, (uint8_t*)&pi_rx_byte, 1);  // USART2(Pi)에서도 1바이트씩 받는 인터럽트 수신 시작

  LCD_Init();                                 // LCD 초기 설정 명령어들을 순서대로 전송
  HAL_Delay(100);                             // 초기화 안정화를 위해 잠깐 대기

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)                                  // 전원이 켜져있는 동안 무한히 반복되는 메인 루프의 시작
  {                                           // while 루프의 코드 블록 시작 중괄호

      ELM327_RequestSpeed();                 // 매 반복(loop)마다 ELM327에게 속도 요청 명령을 한 번 전송

      HAL_Delay(67);                       // 200밀리초 동안 프로그램을 멈춰서, 블루투스 왕복+ELM327 응답 처리시간을 기다려줌

      if (line_ready)                        // line_ready 깃발이 1(완성됨)인지 확인하는 조건문 시작
      {                                       // if문 코드 블록 시작 중괄호

          int speed = ParseSpeedResponse((const char*)rx_line);
          // rx_line에 쌓인 문자열을 파싱 함수에 넘겨서 결과를 speed라는 지역변수에 저장

          if (speed >= 0) // 파싱 결과가 0 이상(=에러 아님, 유효한 값)인지 확인하는 조건문
          {  // 이 조건이 참일 때 실행될 블록 시작 중괄호

              last_speed_kmh = speed;         // 유효한 값이므로 전역변수 last_speed_kmh에 이번 속도값을 저장(갱신)

              HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
              HAL_Delay(67);   // 원래 200ms 의도 → 보정값
              HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);
          }                                   // if (speed >= 0) 블록 끝

          line_ready = 0;                    // 이번 응답 처리를 다 끝냈으므로 깃발을 다시 0(대기 상태)으로 내림

          rx_index = 0;                      // 다음 번 응답을 처음 칸부터 새로 채울 수 있도록 인덱스를 0으로 초기화

      }                                       // if (line_ready) 블록 끝

      if (radar_frame_ready)                 // 레이더 프레임이 완성됐는지 확인하는 조건문
          {                                       // 조건 블록 시작

              ParseRadarFrame();                 // 완성된 프레임을 해석하는 함수 호출 (결과는 전역변수에 저장됨)

              radar_frame_ready = 0;             // 처리 끝났으니 깃발을 다시 내림
              radar_index = 0;                   // 다음 프레임을 처음부터 받을 수 있도록 인덱스 초기화

          }                                       // if (radar_frame_ready) 블록 끝

      // ===== Pi와 주기적으로 데이터 주고받기 =====
          if (HAL_GetTick() - last_pi_send_tick >= 200)          // 200ms마다 한 번씩 (레이더 데이터 갱신 주기와 맞춤)
          {
              Pi_SendRadarData();                                 // Pi한테 지금 레이더 값(거리, 상대속도) 전송
              last_pi_send_tick = HAL_GetTick();                  // 마지막 전송 시각 갱신
          }

          if (pi_line_ready)                                      // Pi로부터 한 줄(응답)이 다 도착했으면
          {
              Pi_ParseBrightnessResponse((const char*)pi_rx_line); // 그 응답을 파싱해서 pi_brightness_delta, pi_anomaly_flag에 저장
              pi_line_ready = 0;                                  // 처리 끝났으니 깃발 내림
              pi_rx_index = 0;                                    // 다음 줄 받을 준비
          }

          // ===== 판정 로직 실행 =====
          UpdateFrontCarSpeed();                                  // 레이더+내차속도로 앞차 절대속도 계산 및 이력 저장
          UpdateAnomalyJudgement();                                // 감속여부+밝기이상여부 종합해서 최종 판정
      /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }                                           // while(1) 루프 블록 끝 (실제로는 무한루프라 여기 도달 후 다시 맨 위로)
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 250;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RES_Pin|LCD_DC_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RES_Pin LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_RES_Pin|LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void ELM327_RequestSpeed(void)             // 속도 요청 명령을 보내는 함수의 실제 내용(구현) 시작
{                                           // 함수 블록 시작 중괄호

    uint8_t cmd[] = "010D\r"; // 전송할 명령 문자열을 담은 배열: OBD-II Mode01+PID0D(속도 요청)+캐리지리턴
    HAL_UART_Transmit(&huart3, cmd, sizeof(cmd) - 1, 100);
                                            // USART3로 cmd 배열을 전송 / sizeof(cmd)-1: 문자열 끝 널문자는 제외한 실제 길이 / 100: 타임아웃(ms)
}                                           // ELM327_RequestSpeed 함수 블록 끝


int ParseSpeedResponse(const char *raw)    // 응답 문자열을 파싱하는 함수의 실제 내용 시작, raw는 파싱할 원본 문자열
{                                           // 함수 블록 시작 중괄호

    const char *p = strstr(raw, "41 0D");  // raw 문자열 안에서 "41 0D"(공백 포함 버전)가 어디서 시작하는지 찾아 그 위치를 p에 저장

    if (p == NULL)                         // "41 0D"를 못 찾았을 경우(NULL이 반환됨)를 확인하는 조건문
    {                                       // 이 조건 블록 시작

        p = strstr(raw, "410D");           // 공백 없이 붙어서 오는 "410D" 버전도 찾아봄

        if (p == NULL) return -1;          // 그마저도 없으면 이 응답엔 속도 데이터가 없다는 뜻이므로 즉시 -1(에러) 반환하고 함수 종료

        p += 4;                            // "410D" 네 글자만큼 포인터를 이동시켜, 그 바로 뒤(값이 시작될 위치)를 가리키게 함

    }                                       // if (p == NULL) 블록 끝
    else                                    // "41 0D"를 (공백 버전으로) 찾은 경우
    {                                       // else 블록 시작

        p += 5;                            // "41 0D" 다섯 글자(공백 포함)만큼 포인터를 이동시켜 값 위치로 이동

    }                                       // else 블록 끝

    while (*p == ' ') p++;                 // p가 가리키는 곳에 공백이 남아있는 동안 계속 한 칸씩 앞으로 이동(공백 건너뛰기)

    unsigned int value;                    // 16진수를 변환해서 담을 부호없는 정수 변수를 선언

    if (sscanf(p, "%2x", &value) != 1) return -1;
                                            // p 위치부터 16진수 2자리를 읽어 value에 저장 시도, 실패(읽은 개수가 1이 아니면)하면 -1 반환

    return (int)value;                     // 성공적으로 읽은 value를 정수로 형변환해서 최종 반환

}                                           // ParseSpeedResponse 함수 블록 끝


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)          // USART3(HC-05/ELM327)에서 발생한 인터럽트인 경우
    {
        char c = (char)rx_byte;
        if (c == '\r' || c == '>')
        {
            rx_line[rx_index] = '\0';
            line_ready = 1;
        }
        else if (rx_index < RX_LINE_MAX - 1)
        {
            rx_line[rx_index++] = c;
        }
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_byte, 1);
    }

    else if (huart->Instance == USART1)      // USART1(레이더)에서 발생한 인터럽트인 경우
    {
        radar_frame[radar_index++] = radar_byte;
        if (radar_index >= RADAR_FRAME_LEN)
        {
            radar_frame_ready = 1;
        }
        HAL_UART_Receive_IT(&huart1, (uint8_t*)&radar_byte, 1);
    }

    else if (huart->Instance == USART2)      // USART2(Pi)에서 발생한 인터럽트인 경우 (새로 추가된 부분)
    {
        char c = (char)pi_rx_byte;           // 받은 1바이트를 문자로 변환

        if (c == '\n' || c == '\r')          // 줄바꿈 문자를 만나면 (Pi는 파이썬이라 \n을 씀)
        {
            if (pi_rx_index > 0)             // 빈 줄이 아니라 실제로 뭔가 받은 경우에만
            {
                pi_rx_line[pi_rx_index] = '\0';  // 문자열 끝에 널문자 추가
                pi_line_ready = 1;            // 한 줄 완성 깃발 올림
            }
        }
        else if (pi_rx_index < PI_RX_LINE_MAX - 1)  // 버퍼에 공간 남아있으면
        {
            pi_rx_line[pi_rx_index++] = c;   // 문자 저장하고 인덱스 증가
        }

        HAL_UART_Receive_IT(&huart2, (uint8_t*)&pi_rx_byte, 1);  // USART2 다음 바이트 재무장
    }
}

// 완성된 레이더 프레임(바이트 배열)을 해석해서 거리/속도/각도로 변환하는 함수
// ⚠ 아래 바이트 순서·스케일은 임시 가정치. HLK-LD2451 실제 데이터시트로 반드시 재확인 필요
void ParseRadarFrame(void)
{
    if (radar_frame[0] != 0xAA || radar_frame[1] != 0x55) return;
                                            // 앞 2바이트가 정해진 헤더(0xAA, 0x55)와 다르면 잘못된 프레임이므로 그냥 종료

    uint16_t distance_cm = (radar_frame[2] << 8) | radar_frame[3];
                                            // 3,4번째 바이트를 합쳐서 거리값(cm)을 만듦 (앞 바이트를 8비트 왼쪽으로 밀고 OR 연산으로 합침)

    int8_t speed_raw = (int8_t)radar_frame[4];
                                            // 5번째 바이트를 부호있는 정수로 해석 (음수면 가까워지는 중이라는 의미로 가정)

    int8_t angle_raw = (int8_t)radar_frame[5];
                                            // 6번째 바이트를 부호있는 정수로 해석 (각도값)

    radar_distance_m = distance_cm / 100.0f;  // cm를 100으로 나눠서 m 단위로 변환 후 전역변수에 저장

    radar_speed_kmh = (float)speed_raw;       // 상대속도값을 float으로 변환해서 전역변수에 저장

    radar_angle_deg = angle_raw;              // 각도값을 전역변수에 저장

    // TODO: radar_frame[6]에 들어있는 체크섬 검증 로직 추가 필요 (실제 포맷 확인 후)
}
// UART1(레이더) 수신 콜백 — HAL_UART_RxCpltCallback은 이미 위에서 USART3용으로 만들었으므로,
// 아래처럼 else if로 이어붙여서 하나의 콜백 함수가 USART1과 USART3를 둘 다 구분해서 처리하게 함

// STM32가 Pi에게 레이더 데이터를 보내는 함수
void Pi_SendRadarData(void)
{
    char msg[48];                            // 보낼 메시지를 담을 임시 문자 배열

    int len = snprintf(msg, sizeof(msg), "R,%.1f,%.1f\r\n", radar_distance_m, radar_speed_kmh);
    // "R,거리,상대속도\r\n" 형식의 문자열을 만듦, snprintf는 printf처럼 포맷팅해서 문자열로 만들어줌
    // %.1f는 소수점 첫째자리까지 표시하라는 뜻

    HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, 100);  // USART2로 그 문자열 전송, 타임아웃 100ms
}

// Pi가 보낸 응답("B,밝기델타,이상여부")을 파싱하는 함수
int Pi_ParseBrightnessResponse(const char *raw)
{
    if (raw[0] != 'B' || raw[1] != ',')     // 응답이 "B,"로 시작하지 않으면 (형식이 다르면)
    {
        return -1;                           // 잘못된 형식이므로 에러 반환
    }

    float delta_val;                         // 파싱될 밝기 델타값을 담을 변수
    int anomaly_val;                         // 파싱될 이상여부(0/1)를 담을 변수

    int parsed = sscanf(raw + 2, "%f,%d", &delta_val, &anomaly_val);
    // raw+2는 "B," 두 글자를 건너뛴 위치부터 읽기 시작한다는 뜻
    // "%f,%d" 형식으로 부동소수점 하나, 정수 하나를 순서대로 읽어옴

    if (parsed != 2)                         // 두 개 값을 다 못 읽었으면 (형식이 안 맞으면)
    {
        return -1;                           // 에러 반환
    }

    pi_brightness_delta = delta_val;         // 파싱 성공하면 전역변수에 밝기 델타값 저장
    pi_anomaly_flag = (uint8_t)anomaly_val;  // 이상여부도 전역변수에 저장
    pi_data_valid = 1;                       // "유효한 Pi 데이터를 받은 적 있다"고 표시

    return 0;                                // 성공 반환
}

// 레이더의 상대속도 + 내 차 절대속도(ELM327)를 결합해서 앞차 절대속도를 계산하고 이력에 저장하는 함수
void UpdateFrontCarSpeed(void)
{
    if (last_speed_kmh < 0)                  // 아직 내 차 속도값을 한 번도 못 받았으면 (-1인 초기상태)
    {
        return;                              // 계산 불가능하므로 그냥 함수 종료 (아무것도 안 함)
    }

    // TODO: 아래 부호 관계는 레이더 실물 데이터로 검증 후 필요시 수정
    // radar_speed_kmh: 양수=멀어짐, 음수=가까워짐 (radar_parser.py 설계 기준)
    front_car_speed_kmh = (float)last_speed_kmh + radar_speed_kmh;
    // 내 차 속도에 레이더가 알려준 상대속도를 더해서 앞차의 절대속도를 역산

    front_speed_history[history_index].speed_kmh = front_car_speed_kmh;  // 이번 계산값을 이력 배열에 저장
    front_speed_history[history_index].tick = HAL_GetTick();             // 저장 시각도 같이 기록

    history_index = (history_index + 1) % SPEED_HISTORY_LEN;  // 다음 저장 위치로 이동 (배열 끝에 도달하면 다시 처음으로, 원형버퍼 방식)

    if (history_count < SPEED_HISTORY_LEN)   // 아직 이력이 배열 최대 크기만큼 안 쌓였으면
    {
        history_count++;                     // 쌓인 개수를 하나 늘림
    }
}

// 최근 이력을 보고 "지금 앞차가 감속 중인지"를 판정하는 함수
uint8_t CheckDeceleration(void)
{
    if (history_count < 2)                   // 비교할 만큼 이력이 충분히 쌓이지 않았으면
    {
        return 0;                            // 판단 불가, 감속 아님으로 처리
    }

    uint32_t now = HAL_GetTick();            // 현재 시각

    float oldest_speed_in_window = front_car_speed_kmh;  // 비교 기준이 될 "예전 속도값", 기본값은 현재값으로 초기화
    uint8_t found_old_sample = 0;             // 비교할 만큼 오래된 샘플을 찾았는지 표시

    for (uint8_t i = 0; i < history_count; i++)  // 저장된 이력을 하나씩 순회
    {
        uint8_t idx = (history_index + SPEED_HISTORY_LEN - 1 - i) % SPEED_HISTORY_LEN;
        // 제일 최근 것부터 거꾸로 순서대로 인덱스를 계산 (원형버퍼 역순 탐색)

        uint32_t age_ms = now - front_speed_history[idx].tick;  // 그 샘플이 얼마나 오래된 것인지(ms)

        if (age_ms >= DECEL_WINDOW_MS)       // 설정한 비교 시간창(예: 1000ms)보다 오래된 샘플을 찾으면
        {
            oldest_speed_in_window = front_speed_history[idx].speed_kmh;  // 그 샘플을 "예전 속도"로 채택
            found_old_sample = 1;             // 찾았다고 표시
            break;                            // 반복문 종료
        }
    }

    if (!found_old_sample)                    // 시간창만큼 오래된 샘플을 못 찾았으면 (아직 데이터가 부족)
    {
        return 0;                            // 판단 불가, 감속 아님으로 처리
    }

    float speed_drop = oldest_speed_in_window - front_car_speed_kmh;  // 예전속도 - 현재속도 = 감속한 정도

    if (speed_drop >= DECEL_THRESHOLD_KMH)    // 감속량이 기준치 이상이면
    {
        return 1;                             // 감속 중이라고 판정
    }

    return 0;                                 // 그 외에는 감속 아님
}

// 감속여부 + 밝기이상여부를 종합해서 최종 "제동등 이상" 판정을 내리는 함수
void UpdateAnomalyJudgement(void)
{
    is_decelerating = CheckDeceleration();    // 지금 감속 중인지 판정 결과를 저장

    uint8_t brake_light_missing = 0;          // "제동등이 안 켜진 것으로 보인다" 상태를 저장할 변수, 기본값 0(정상)

    if (pi_data_valid)                        // Pi로부터 유효한 데이터를 받은 적 있으면
    {
        brake_light_missing = (pi_anomaly_flag == 0);
        // pi_anomaly_flag가 0(정상, 즉 밝기변화 없음)이면 "제동등이 안 켜진 것"으로 해석
        // (pi_anomaly_flag=1이면 Pi가 이미 "밝아짐" 감지했다는 뜻이므로 이 경우는 문제 없음)
    }

    if (is_decelerating && brake_light_missing)  // 감속 중인데 + 제동등은 안 켜진 것으로 보이면 (모순 상황)
    {
        if (anomaly_start_tick == 0)          // 이 모순 상황이 방금 막 시작된 거라면 (아직 시작시각 기록 안 됨)
        {
            anomaly_start_tick = HAL_GetTick();  // 지금 시각을 "이상 의심 시작 시각"으로 기록
        }

        if (HAL_GetTick() - anomaly_start_tick >= ANOMALY_PERSIST_MS)
        // 이 모순 상황이 시작된 후로 설정한 지속시간(예: 500ms) 이상 계속됐으면
        {
            anomaly_confirmed = 1;            // 최종적으로 "이상"이라고 확정
        }
    }
    else                                       // 감속 중이 아니거나, 제동등이 정상으로 보이면 (모순 상황 해소)
    {
        anomaly_start_tick = 0;               // 시작시각 초기화 (다음에 새로 감지되면 처음부터 다시 카운트)
        anomaly_confirmed = 0;                // 이상 확정 상태도 해제
    }

    if (anomaly_confirmed)                    // 최종적으로 이상이 확정된 상태면
    {
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 부저를 계속 울림 (경고음)
    }
    else                                       // 정상 상태면
    {
        HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);   // 부저 정지
    }
}

// LCD를 하드웨어적으로 리셋시키는 함수
void LCD_Reset(void)
{
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_RESET);  // RES 핀을 LOW로 내려서 리셋 시작
    HAL_Delay(20);                           // 20ms 대기 (리셋 신호 유지 시간)
    HAL_GPIO_WritePin(LCD_RES_GPIO_Port, LCD_RES_Pin, GPIO_PIN_SET);    // RES 핀을 다시 HIGH로 올려서 리셋 해제
    HAL_Delay(120);                          // 120ms 대기 (리셋 후 안정화 시간, ST7789 데이터시트 권장값)
}

// LCD에 "이건 명령이다"라고 표시하며 1바이트를 전송하는 함수
void LCD_WriteCommand(uint8_t cmd)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_RESET);    // DC 핀을 LOW로 = "지금 보내는 건 명령이다"라는 신호
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);    // CS 핀을 LOW로 = "지금부터 너(LCD)한테 말할게"라는 신호(칩 선택)
    HAL_SPI_Transmit(&hspi1, &cmd, 1, HAL_MAX_DELAY);                    // SPI로 명령 1바이트 전송
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);      // CS 핀을 다시 HIGH로 = "전송 끝, 이제 너한테 말 안 해"
}

// LCD에 "이건 데이터다"라고 표시하며 1바이트를 전송하는 함수
void LCD_WriteData(uint8_t data)
{
    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);      // DC 핀을 HIGH로 = "지금 보내는 건 데이터다"라는 신호
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);    // CS 핀을 LOW로 (칩 선택)
    HAL_SPI_Transmit(&hspi1, &data, 1, HAL_MAX_DELAY);                   // SPI로 데이터 1바이트 전송
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);      // CS 핀을 다시 HIGH로 (전송 끝)
}

// ST7789 LCD를 초기화하는 함수 (여러 설정 명령을 순서대로 전송)
void LCD_Init(void)
{
    LCD_Reset();                             // 하드웨어 리셋 먼저 실행

    LCD_WriteCommand(0x01);                  // Software Reset (소프트웨어 리셋 명령 추가)
    HAL_Delay(150);                          // 150ms 대기 (리셋 완료 대기)

    LCD_WriteCommand(0x11);                  // Sleep Out (절전모드 해제)
    HAL_Delay(255);                          // 충분히 대기 (기존보다 늘림)

    LCD_WriteCommand(0x3A);                  // Interface Pixel Format
    LCD_WriteData(0x55);                     // 0x55 = 16비트 컬러(RGB565), 기존 0x05에서 값 수정
                                              // (0x55는 MCU 인터페이스/RGB 인터페이스 둘 다 16비트로 지정하는 표준값)

    LCD_WriteCommand(0x36);                  // Memory Data Access Control
    LCD_WriteData(0x00);                     // 방향 기본값

    LCD_WriteCommand(0xB2);                  // Porch Setting (포치 타이밍 설정, 클론 패널에서 필요한 경우 많음)
    LCD_WriteData(0x0C);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x00);
    LCD_WriteData(0x33);
    LCD_WriteData(0x33);

    LCD_WriteCommand(0xB7);                  // Gate Control
    LCD_WriteData(0x35);

    LCD_WriteCommand(0xBB);                  // VCOM Setting
    LCD_WriteData(0x19);

    LCD_WriteCommand(0xC0);                  // LCM Control
    LCD_WriteData(0x2C);

    LCD_WriteCommand(0xC2);                  // VDV and VRH Command Enable
    LCD_WriteData(0x01);

    LCD_WriteCommand(0xC3);                  // VRH Set
    LCD_WriteData(0x12);

    LCD_WriteCommand(0xC4);                  // VDV Set
    LCD_WriteData(0x20);

    LCD_WriteCommand(0xC6);                  // Frame Rate Control
    LCD_WriteData(0x0F);

    LCD_WriteCommand(0xD0);                  // Power Control 1
    LCD_WriteData(0xA4);
    LCD_WriteData(0xA1);

    LCD_WriteCommand(0xE0);                  // Positive Voltage Gamma Control
    LCD_WriteData(0xD0);
    LCD_WriteData(0x04);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x11);
    LCD_WriteData(0x13);
    LCD_WriteData(0x2B);
    LCD_WriteData(0x3F);
    LCD_WriteData(0x54);
    LCD_WriteData(0x4C);
    LCD_WriteData(0x18);
    LCD_WriteData(0x0D);
    LCD_WriteData(0x0B);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x23);

    LCD_WriteCommand(0xE1);                  // Negative Voltage Gamma Control
    LCD_WriteData(0xD0);
    LCD_WriteData(0x04);
    LCD_WriteData(0x0C);
    LCD_WriteData(0x11);
    LCD_WriteData(0x13);
    LCD_WriteData(0x2C);
    LCD_WriteData(0x3F);
    LCD_WriteData(0x44);
    LCD_WriteData(0x51);
    LCD_WriteData(0x2F);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x1F);
    LCD_WriteData(0x20);
    LCD_WriteData(0x23);

    LCD_WriteCommand(0x21);                  // Display Inversion On (많은 ST7789 패널이 이거 켜야 색이 정상으로 보임)

    LCD_WriteCommand(0x29);                  // Display On
    HAL_Delay(50);
}

// 화면 전체를 지정된 색상(RGB565, 16비트)으로 채우는 함수
void LCD_FillScreen(uint16_t color)
{
    LCD_WriteCommand(0x2A);                  // "Column Address Set" 명령 (가로 범위 설정 시작)
    LCD_WriteData(0x00);                     // 시작 컬럼 상위바이트
    LCD_WriteData(0x00);                     // 시작 컬럼 하위바이트 (0부터 시작)
    LCD_WriteData((LCD_WIDTH - 1) >> 8);     // 끝 컬럼 상위바이트 (239를 16비트로 나눈 상위 8비트)
    LCD_WriteData((LCD_WIDTH - 1) & 0xFF);   // 끝 컬럼 하위바이트

    LCD_WriteCommand(0x2B);                  // "Row Address Set" 명령 (세로 범위 설정 시작)
    LCD_WriteData(0x00);                     // 시작 로우 상위바이트
    LCD_WriteData(0x00);                     // 시작 로우 하위바이트
    LCD_WriteData((LCD_HEIGHT - 1) >> 8);    // 끝 로우 상위바이트
    LCD_WriteData((LCD_HEIGHT - 1) & 0xFF);  // 끝 로우 하위바이트

    LCD_WriteCommand(0x2C);                  // "Memory Write" 명령 (지금부터 보내는 데이터는 화면 픽셀 값이라는 뜻)

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);   // DC를 HIGH로 (데이터 모드)
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET); // CS를 LOW로 (칩 선택, 이번엔 대량 전송이라 한 번만 선택)

    uint8_t color_bytes[2];                  // 색상값(16비트)을 2바이트로 나눠 담을 배열
    color_bytes[0] = color >> 8;             // 색상값의 상위 8비트
    color_bytes[1] = color & 0xFF;           // 색상값의 하위 8비트

    for (uint32_t i = 0; i < (uint32_t)LCD_WIDTH * LCD_HEIGHT; i++)  // 전체 픽셀 개수(240×240)만큼 반복
    {
        HAL_SPI_Transmit(&hspi1, color_bytes, 2, HAL_MAX_DELAY);    // 같은 색상 2바이트를 픽셀 하나마다 전송
    }

    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);  // 다 끝났으면 CS를 다시 HIGH로 (전송 종료)
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
