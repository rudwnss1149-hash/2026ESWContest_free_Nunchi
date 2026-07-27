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
  HAL_UART_Receive_IT(&huart1, (uint8_t*)&radar_byte, 1);  // USART1(레이더)에서도 1바이트씩 받는 인터럽트 수신을 시작(무장)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)                                  // 전원이 켜져있는 동안 무한히 반복되는 메인 루프의 시작
  {                                           // while 루프의 코드 블록 시작 중괄호

      ELM327_RequestSpeed();                 // 매 반복(loop)마다 ELM327에게 속도 요청 명령을 한 번 전송

      HAL_Delay(200);                        // 200밀리초 동안 프로그램을 멈춰서, 블루투스 왕복+ELM327 응답 처리시간을 기다려줌

      if (line_ready)                        // line_ready 깃발이 1(완성됨)인지 확인하는 조건문 시작
      {                                       // if문 코드 블록 시작 중괄호

          int speed = ParseSpeedResponse((const char*)rx_line);
          // rx_line에 쌓인 문자열을 파싱 함수에 넘겨서 결과를 speed라는 지역변수에 저장

          if (speed >= 0) // 파싱 결과가 0 이상(=에러 아님, 유효한 값)인지 확인하는 조건문
          {  // 이 조건이 참일 때 실행될 블록 시작 중괄호

              last_speed_kmh = speed;         // 유효한 값이므로 전역변수 last_speed_kmh에 이번 속도값을 저장(갱신)

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RES_Pin|LCD_DC_Pin|LCD_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : LCD_RES_Pin LCD_DC_Pin LCD_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RES_Pin|LCD_DC_Pin|LCD_CS_Pin;
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
        char c = (char)rx_byte;             // 받은 1바이트를 문자로 변환

        if (c == '\r' || c == '>')          // 줄바꿈 또는 프롬프트 문자인지 확인
        {
            rx_line[rx_index] = '\0';       // 문자열 끝에 널문자 추가
            line_ready = 1;                 // 한 줄 완성 깃발 올림
        }
        else if (rx_index < RX_LINE_MAX - 1)  // 버퍼에 공간이 남아있으면
        {
            rx_line[rx_index++] = c;        // 문자 저장하고 인덱스 증가
        }

        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_byte, 1);  // USART3 다음 바이트 수신 재무장
    }                                        // USART3 처리 블록 끝

    else if (huart->Instance == USART1)      // USART1(레이더)에서 발생한 인터럽트인 경우
    {
        radar_frame[radar_index++] = radar_byte;  // 받은 바이트를 프레임 배열에 저장하고 인덱스 증가

        if (radar_index >= RADAR_FRAME_LEN)  // 정해진 프레임 길이(7바이트)만큼 다 받았는지 확인
        {
            radar_frame_ready = 1;           // 프레임 완성 깃발 올림
        }

        HAL_UART_Receive_IT(&huart1, (uint8_t*)&radar_byte, 1);  // USART1 다음 바이트 수신 재무장
    }                                        // USART1 처리 블록 끝
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
