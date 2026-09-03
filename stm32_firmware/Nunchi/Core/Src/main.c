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
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
#define RX_LINE_MAX 32                     // 응답 한 줄 버퍼 최대 길이
volatile uint8_t rx_byte;                  // 인터럽트로 한 바이트씩 받아오는 임시 변수
                                            // (volatile: 인터럽트 안에서 바뀌는 값이라 컴파일러 최적화 방지용)
volatile char rx_line[RX_LINE_MAX];        // 받은 바이트를 이어붙여 한 줄을 완성해가는 버퍼
volatile uint8_t rx_index = 0;             // rx_line에서 지금까지 채운 위치, 시작값 0
volatile uint8_t line_ready = 0;           // 한 줄 완성 신호 (0=대기, 1=완성)
volatile int16_t last_speed_kmh = -1;      // 가장 최근 파싱 성공한 속도값(km/h), -1=아직 유효값 없음
volatile uint8_t elm_data_valid = 0;       // ELM327 응답을 한 번이라도 정상 파싱했는지 (0=아직, 1=있음)

// ============================================================================
// ===== 레이더(HLK-LD2451) 수신 관련 =====
// 출처: 하이링크 공식 문서 "HLK-LD2451 Serial Communication Protocol v1.03"
//   헤더 4바이트: F4 F3 F2 F1
//   길이필드 2바이트 (페이로드 길이, 리틀엔디안 가정 — 실측 재확인 필요)
//   페이로드: 타겟개수(1바이트) + 타겟마다 6바이트(경보정보/각도/거리/속도방향/속도/SNR) * 최대 5개
//   테일 4바이트: F8 F7 F6 F5 / 체크섬 없음 / 기본 baudrate 115200
// ============================================================================
#define RADAR_HDR_LEN       4     // 헤더 바이트 개수
#define RADAR_LEN_FIELD_LEN 2     // 길이필드 바이트 개수
#define RADAR_TAIL_LEN      4     // 테일 바이트 개수
#define RADAR_MAX_TARGET    5     // 한 프레임에 최대 몇 개 타겟까지 오는지(문서 기준)
#define RADAR_TARGET_SZ     6     // 타겟 1개당 바이트 수: 경보정보1+각도1+거리1+속도방향1+속도1+SNR1
#define RADAR_MAX_PAYLOAD   (1 + RADAR_MAX_TARGET * RADAR_TARGET_SZ)  // 타겟개수(1)+타겟들(최대30)=31바이트

static const uint8_t RADAR_HEADER[RADAR_HDR_LEN] = {0xF4, 0xF3, 0xF2, 0xF1};  // 정해진 헤더 시퀀스
static const uint8_t RADAR_TAIL[RADAR_TAIL_LEN]  = {0xF8, 0xF7, 0xF6, 0xF5};  // 정해진 테일 시퀀스

// 레이더 프레임을 바이트 단위로 조립해나가는 상태를 나타내는 열거형(상태기계)
typedef enum {
    RADAR_WAIT_HEADER = 0,   // 아직 헤더 4바이트를 다 못 찾은 상태(동기화 안 됨) — 기본 시작 상태
    RADAR_WAIT_LENGTH,       // 헤더는 찾았고, 길이필드 2바이트를 받는 중
    RADAR_WAIT_PAYLOAD,      // 길이필드까지 받았고, 페이로드를 길이만큼 받는 중
    RADAR_WAIT_TAIL          // 페이로드까지 받았고, 테일 4바이트가 맞는지 확인하는 중
} RadarRxState;

volatile RadarRxState radar_state = RADAR_WAIT_HEADER;     // 지금 몇 번째 단계인지 기억, 시작은 헤더대기
volatile uint8_t  radar_header_match = 0;                  // 헤더 몇 바이트째까지 연속으로 일치했는지(0~4)
volatile uint16_t radar_payload_len  = 0;                  // 길이필드에서 읽어낸 페이로드 길이(바이트수)
volatile uint8_t  radar_payload_idx  = 0;                  // 페이로드를 지금 몇 바이트째 받는 중인지
volatile uint8_t  radar_tail_idx     = 0;                  // 테일을 지금 몇 바이트째 확인 중인지
volatile uint8_t  radar_payload_buf[RADAR_MAX_PAYLOAD];    // 페이로드만 따로 모아 담는 버퍼
volatile uint8_t  radar_byte;                              // (DMA 방식으로 바뀌면서 더 이상 사용 안 함, 호환성 위해 남겨둠)
volatile uint8_t  radar_frame_ready = 0;                   // 프레임 하나가 완전히(헤더+길이+페이로드+테일 검증) 도착했다는 신호
volatile uint32_t radar_raw_byte_count = 0;                 // 프로토콜 해석과 무관하게 USART1로 들어온 바이트 총 개수

// STM32가 USART1(레이더)에서 받은 raw 바이트를 그대로 모아 Pi로 중계하는 버퍼
// (CH340으로 PC에서 본 것과 STM32가 실제로 받는 게 같은지 비교용)
#define RADAR_DEBUG_BUF_LEN 64                                    // 한 번에 최대 몇 바이트까지 모아서 보낼지
volatile uint8_t radar_debug_buf[RADAR_DEBUG_BUF_LEN];            // raw 바이트를 그대로 쌓아두는 버퍼
volatile uint8_t radar_debug_len = 0;                             // 지금까지 몇 바이트나 쌓였는지

// ===== 레이더 DMA 순환수신 =====
// 인터럽트로 한 바이트씩 받으면 레이더가 빠르게 연달아 쏠 때 CPU가 못 따라가 overrun이 잦았음
// → DMA가 하드웨어적으로 버퍼에 채워넣고, 메인루프에서 여유 있을 때 꺼내 처리하는 방식으로 변경
#define RADAR_DMA_BUF_SIZE 2048                                    // DMA 순환버퍼 크기(바이트)
volatile uint8_t radar_dma_buf[RADAR_DMA_BUF_SIZE];                // DMA가 계속 채워넣는 순환버퍼
volatile uint16_t radar_dma_last_pos = 0;                          // 마지막으로 처리한 위치(인덱스)
// ===== DMA 관련 끝 =====

volatile float radar_distance_m  = -1.0f;   // 파싱된 최신 거리값(m), -1.0=유효값 없음 (median 스무딩 적용된 값)
volatile float radar_speed_kmh   = 0.0f;    // 파싱된 최신 상대속도값(km/h), 양수=멀어짐/음수=가까워짐 (median 스무딩 적용된 값)
volatile int8_t radar_angle_deg  = 0;       // 파싱된 최신 각도값(도)

// 실차 테스트에서 거리값이 프레임마다 완전히 다른 물체로 튀는 문제(예: 2m→47m)가 있어서 추가한 트랙 상태
volatile float radar_track_distance_m = -1.0f;  // 추적 중인 차의 최근 원시 거리값 (다음 프레임에 같은 차인지 판단하는 기준), -1=트랙 없음
#define RADAR_TRACK_GATE_M 6.0f                   // 이 거리(m) 이내 타겟만 같은 차로 인정, 벗어나면 다른 물체로 보고 재탐색
// 속도(S)도 거리처럼 프레임마다 튀는 게 확인돼서 동일하게 median-of-5로 스무딩
static float radar_dist_hist[5]  = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 최근 5프레임 원시 거리값 (median 스무딩용)
static float radar_speed_hist[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // 최근 5프레임 원시 속도값
static uint8_t radar_dist_hist_idx    = 0;        // 거리 버퍼 다음 쓰기 위치(0~4 순환)
static uint8_t radar_dist_hist_count  = 0;        // 거리 버퍼에 쌓인 개수 (5 미만이면 최신값 그대로 사용)
static uint8_t radar_hist_idx    = 0;             // 속도 버퍼 다음 쓰기 위치(0~4 순환)
static uint8_t radar_hist_count  = 0;             // 속도 버퍼에 쌓인 개수
#define RADAR_ANGLE_GATE_DEG 8    // 트랙 연속성 없을 때 재탐색 시, 이 각도(도) 밖 타겟은 후보에서 제외 (부채꼴 오탐 방지)
// ===== 레이더 관련 끝 =====

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
// Pi가 실시간으로 보내는 카메라 검출 상태 (이상감지 로직과 무관, LCD 표시 전용)
volatile uint8_t cam_detected = 0;         // 0=미검출(또는 아직 한 번도 못 받음), 1=검출됨
// Pi<->STM32 통신이 실제로 되는지 확인용 카운터
volatile uint32_t pi_total_lines = 0;      // USART2로 완성된 줄을 총 몇 개나 받았는지 (B, C 상관없이 다 셈)
volatile uint32_t pi_cam_msgs = 0;         // 그중 "C," 파싱에 성공한 횟수
// DECEL/부저 불일치 디버깅용 — 두 판정조건이 실제로 얼마나 겹치는지 LCD로 확인 (UpdateAnomalyJudgement에서 매 루프 갱신)
volatile uint8_t dbg_brake_missing = 0;    // 이번 판정에서 제동등 꺼짐으로 봤는지 (0=정상, 1=꺼짐)
volatile uint32_t dbg_anomaly_ms = 0;      // 지금 "감속+제동등꺼짐" 모순 상황이 몇 ms째 지속중인지 (0이면 모순 아님)
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
#define DECEL_THRESHOLD_KMH 4.0f            // 앞차 감속 판정 임계값(km/h), 실차 테스트로 5.0→4.0 조정
#define DECEL_EXIT_THRESHOLD_KMH 2.5f       // 히스테리시스: OFF로 복귀하는 기준을 진입기준보다 낮게 잡아
                                             // 경계값 근처 노이즈로 감속판정이 깜빡이는 걸 방지
#define DECEL_WINDOW_MS 1000                // 이 시간(ms) 동안의 속도 변화를 비교해서 감속 여부 판단
#define ANOMALY_PERSIST_MS 500              // 이상 상황이 이 시간(ms) 이상 지속돼야 최종 "이상"으로 확정
#define ANOMALY_GRACE_MS 300                // 조건이 잠깐(이 시간 이내) 깨져도 지속 카운트를 리셋하지 않고 유지
                                             // (프레임 한두 개만 노이즈로 벗어나도 카운트가 매번 리셋되던 문제 완화)
volatile uint8_t is_decelerating = 0;       // 지금 앞차가 감속 중인지 여부 (0=아니오, 1=예)
volatile uint32_t anomaly_start_tick = 0;   // "이상 의심" 상황이 시작된 시각 (지속시간 측정용)
volatile uint32_t anomaly_last_true_tick = 0;  // 마지막으로 조건이 참이었던 시각 (grace period 판단용)
volatile uint8_t anomaly_confirmed = 0;     // 최종적으로 "이상"이 확정됐는지 (0.5초 지속 필터 통과 여부)
#define LCD_WIDTH  240                      // 디스플레이 가로 픽셀 수
#define LCD_HEIGHT 240                      // 디스플레이 세로 픽셀 수
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void ELM327_RequestSpeed(void);            // ELM327에게 속도 요청 명령을 보내는 함수
int ParseSpeedResponse(const char *raw);   // ELM327 응답 문자열을 파싱해서 속도값으로 변환 (실패시 -1)
void ParseRadarFrame(void);                // 완성된 레이더 프레임을 해석해서 거리/속도/각도로 변환하는 함수
void RadarProcessByte(uint8_t b);          // 레이더 바이트 하나를 상태기계에 넣어 처리 (DMA버퍼 처리용)
void RadarProcessDmaBuffer(void);          // DMA 순환버퍼에 새로 쌓인 바이트들을 꺼내 처리하는 함수
void Pi_SendRadarData(void);                // STM32가 Pi에게 레이더 데이터(거리, 상대속도)를 보내는 함수
int Pi_ParseBrightnessResponse(const char *raw);  // Pi가 보낸 응답 문자열을 파싱하는 함수
int Pi_ParseCameraStatus(const char *raw);          // Pi가 보낸 "C,검출여부" 응답을 파싱하는 함수
void UpdateFrontCarSpeed(void);             // 레이더+내차속도를 결합해서 앞차 절대속도를 계산하고 이력에 저장하는 함수
uint8_t CheckDeceleration(void);            // 최근 이력을 보고 "지금 감속 중인지" 판정하는 함수
void UpdateAnomalyJudgement(void);          // 감속여부+밝기이상여부를 종합해서 최종 이상감지 판정을 내리는 함수
void LCD_Reset(void);                       // LCD를 하드웨어적으로 리셋시키는 함수
void LCD_WriteCommand(uint8_t cmd);         // LCD에 명령을 보내는 함수
void LCD_WriteData(uint8_t data);           // LCD에 데이터를 보내는 함수
void LCD_Init(void);                        // LCD 초기화 함수
void LCD_FillScreen(uint16_t color);        // 화면 전체를 특정 색으로 채우는 함수
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg_color, uint16_t bg_color, uint8_t size);  // 문자 하나 그리는 함수
void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg_color, uint16_t bg_color, uint8_t size);  // 문자열 그리는 함수
void LCD_UpdateStatus(void);                // 거리/속도/경고상태를 화면에 표시하는 함수
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)radar_dma_buf, RADAR_DMA_BUF_SIZE);  // USART1(레이더) DMA 순환수신 시작
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);  // 부저 PWM 출력 시작 (전원 인가 알림음)
  HAL_Delay(250);                           // 250ms 대기
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_1);   // 부저 정지
  HAL_UART_Receive_IT(&huart2, (uint8_t*)&pi_rx_byte, 1);  // USART2(Pi)에서도 1바이트씩 받는 인터럽트 수신 시작
  HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_byte, 1);      // USART3(ELM327/HC-05) 인터럽트 수신 시작
                                                              // (콜백에서는 재무장만 하므로 최초 1회 무장이 꼭 필요함)
  LCD_Init();                                 // LCD 초기 설정 명령어들을 순서대로 전송
  HAL_Delay(100);                             // 초기화 안정화를 위해 잠깐 대기
  LCD_FillScreen(0x0000);                     // 검정 배경으로 시작 (이후 그 위에 거리/속도/경고 텍스트를 그림)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)                                  // 메인 루프
  {
      RadarProcessDmaBuffer();               // DMA 순환버퍼에 새로 들어온 레이더 바이트를 매 루프마다 즉시 처리
      ELM327_RequestSpeed();                 // 매 반복마다 ELM327에게 속도 요청 명령 전송
      for (uint8_t _w = 0; _w < 20; _w++)    // 10ms씩 20번(총 200ms) 나눠서 기다림 (블루투스 왕복+ELM327 응답 처리시간 확보)
      {
          HAL_Delay(10);                       // 10ms 대기
          RadarProcessDmaBuffer();             // 그 사이사이 레이더 DMA버퍼도 계속 비워줌 (안 그러면 200ms 동안 못 비워서 놓칠 수 있음)
      }
      if (line_ready)                        // 한 줄 완성됐으면
      {
          int speed = ParseSpeedResponse((const char*)rx_line);
          if (speed >= 0) // 파싱 성공(유효값)이면
          {
              last_speed_kmh = speed;         // 전역변수에 이번 속도값 저장
              elm_data_valid = 1;             // ELM327 응답을 한 번이라도 정상 파싱했다는 표시
          }
          // (ELM327 응답마다 삑거리던 확인용 부저는 뺐음 — UpdateAnomalyJudgement()의 이상감지 부저와 겹쳐서 헷갈림)
          line_ready = 0;                    // 처리 끝, 깃발 내림
          rx_index = 0;                      // 다음 응답을 위해 인덱스 초기화
      }
      if (radar_frame_ready)                 // 레이더 프레임이 완성됐으면
          {
              ParseRadarFrame();                 // 완성된 프레임을 해석하는 함수 호출 (결과는 전역변수에 저장됨)
              radar_frame_ready = 0;             // 처리 끝났으니 깃발을 다시 내림
          }
      // ===== Pi와 주기적으로 데이터 주고받기 =====
          if (HAL_GetTick() - last_pi_send_tick >= 200)          // 200ms마다 한 번씩 (레이더 데이터 갱신 주기와 맞춤)
          {
              Pi_SendRadarData();                                 // Pi한테 지금 레이더 값(거리, 상대속도) 전송
              last_pi_send_tick = HAL_GetTick();                  // 마지막 전송 시각 갱신
          }
          if (pi_line_ready)                                      // Pi로부터 한 줄(응답)이 다 도착했으면
          {
              pi_total_lines++;                                   // 한 줄 받을 때마다 카운트 (통신 확인용)
              // Pi가 두 종류 메시지를 보냄 → 첫 글자로 구분
              //   "B,..." = 밝기델타/이상여부 (이상감지 로직에 실제로 쓰임)
              //   "C,..." = 지금 카메라가 차를 검출했는지 여부 (LCD 표시 전용, 판정 로직엔 영향 없음)
              if (pi_rx_line[0] == 'B')
              {
                  Pi_ParseBrightnessResponse((const char*)pi_rx_line); // 파싱해서 pi_brightness_delta, pi_anomaly_flag에 저장
              }
              else if (pi_rx_line[0] == 'C')
              {
                  Pi_ParseCameraStatus((const char*)pi_rx_line);       // 카메라 검출상태 파싱해서 cam_detected에 저장
              }
              pi_line_ready = 0;                                  // 처리 끝났으니 깃발 내림
              pi_rx_index = 0;                                    // 다음 줄 받을 준비
          }
          // ===== 판정 로직 실행 =====
          UpdateFrontCarSpeed();                                  // 레이더+내차속도로 앞차 절대속도 계산 및 이력 저장
          UpdateAnomalyJudgement();                                // 감속여부+밝기이상여부 종합해서 최종 판정 (이상 감지시 부저 울림)

          // ===== 디스플레이에 거리/속도/경고상태 표시 =====
          {
              static uint32_t last_lcd_update_tick = 0;             // 마지막으로 화면 갱신한 시각(ms)
              if (HAL_GetTick() - last_lcd_update_tick >= 100)        // 100ms마다 갱신 (메인루프 한 바퀴가 최소 200ms라 매 루프 빠짐없이 갱신됨)
              {
                  LCD_UpdateStatus();                                   // 거리/속도/경고문구를 화면에 그림
                  last_lcd_update_tick = HAL_GetTick();
              }
          }

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
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  huart1.Init.BaudRate = 115200;           // HLK-LD2451 실제 baudrate
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
// UART 에러(overrun 등)가 나면 HAL이 인터럽트 수신을 자동 재시작하지 않아서
// 에러 콜백에서 직접 재무장(re-arm)해줘야 함. 안 그러면 에러 한 번에 그 채널 수신이 영구히 멈춤.
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    __HAL_UART_CLEAR_OREFLAG(huart);   // Overrun 에러 플래그 클리어
    __HAL_UART_CLEAR_NEFLAG(huart);    // Noise 에러 플래그 클리어
    __HAL_UART_CLEAR_FEFLAG(huart);    // Framing 에러 플래그 클리어
    __HAL_UART_CLEAR_PEFLAG(huart);    // Parity 에러 플래그 클리어

    if (huart->Instance == USART1)                              // 레이더 채널이면 (DMA 방식이라 재시작도 DMA로)
    {
        radar_state = RADAR_WAIT_HEADER;                          // 상태기계도 처음부터 다시 시작하도록 리셋
        radar_header_match = 0;
        radar_dma_last_pos = 0;                                    // DMA 버퍼 읽기 위치도 초기화
        HAL_UART_Receive_DMA(&huart1, (uint8_t*)radar_dma_buf, RADAR_DMA_BUF_SIZE);  // DMA 순환수신 재시작
    }
    else if (huart->Instance == USART2)                         // Pi 채널이면
    {
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&pi_rx_byte, 1);   // 인터럽트 수신 재무장
    }
    else if (huart->Instance == USART3)                         // ELM327/블루투스 채널이면
    {
        HAL_UART_Receive_IT(&huart3, (uint8_t*)&rx_byte, 1);      // 인터럽트 수신 재무장
    }
}

void ELM327_RequestSpeed(void)             // 속도 요청 명령을 보내는 함수
{
    uint8_t cmd[] = "010D\r"; // OBD-II Mode01+PID0D(속도 요청)+캐리지리턴
    HAL_UART_Transmit(&huart3, cmd, sizeof(cmd) - 1, 100);  // sizeof(cmd)-1: 널문자 제외한 길이, 100: 타임아웃(ms)
}
int ParseSpeedResponse(const char *raw)    // ELM327 응답 문자열을 파싱해서 속도값(정수)으로 변환, 실패시 -1
{
    const char *p = strstr(raw, "41 0D");  // "41 0D"(공백 포함) 위치 탐색
    if (p == NULL)
    {
        p = strstr(raw, "410D");           // 공백 없이 붙어서 오는 버전도 확인
        if (p == NULL) return -1;          // 둘 다 없으면 속도 데이터 없는 응답
        p += 4;                            // "410D" 다음(값 시작 위치)으로 이동
    }
    else
    {
        p += 5;                            // "41 0D" 다음(값 시작 위치)으로 이동
    }
    while (*p == ' ') p++;                 // 공백 건너뛰기
    unsigned int value;
    if (sscanf(p, "%2x", &value) != 1) return -1;  // 16진수 2자리 파싱 실패시 -1
    return (int)value;
}

// 레이더 바이트 하나를 상태기계에 넣어서 처리하는 함수 (DMA버퍼를 메인루프가 읽으며 호출)
void RadarProcessByte(uint8_t b)
{
    radar_raw_byte_count++;              // 헤더 매칭 여부와 무관하게 바이트가 들어올 때마다 1 증가
    if (radar_debug_len < RADAR_DEBUG_BUF_LEN)   // 버퍼에 아직 공간이 남아있으면
    {
        radar_debug_buf[radar_debug_len++] = b;    // 받은 바이트를 그대로 디버그 버퍼에 적재
    }
    switch (radar_state)                 // 지금 어느 단계인지에 따라 분기
    {
        case RADAR_WAIT_HEADER:                            // 헤더 4바이트를 찾는 중인 단계
            if (b == RADAR_HEADER[radar_header_match])      // 기대하는 다음 헤더 바이트와 일치하면
            {
                radar_header_match++;                        // 연속 일치 카운트를 1 증가
                if (radar_header_match >= RADAR_HDR_LEN)      // 헤더 4바이트가 전부 순서대로 일치했으면
                {
                    radar_state = RADAR_WAIT_LENGTH;           // 다음 단계(길이필드 읽기)로 전환
                    radar_payload_idx = 0;                     // 길이필드 임시 인덱스로 재사용(0=하위바이트 차례)
                }
            }
            else                                             // 기대한 바이트가 아니면(동기화가 깨진 경우)
            {
                radar_header_match = (b == RADAR_HEADER[0]) ? 1 : 0;
            }
            break;

        case RADAR_WAIT_LENGTH:                            // 길이필드 2바이트를 받는 중인 단계
            if (radar_payload_idx == 0)                      // 길이필드의 첫 바이트(하위바이트, 리틀엔디안 가정)
            {
                radar_payload_len = b;                         // 하위바이트를 그대로 저장
                radar_payload_idx = 1;                         // 다음 차례는 상위바이트
            }
            else                                               // 길이필드의 두 번째 바이트(상위바이트)
            {
                radar_payload_len |= ((uint16_t)b << 8);        // 상위바이트를 8비트 왼쪽으로 밀어서 합침
                if (radar_payload_len > RADAR_MAX_PAYLOAD)       // 비정상적으로 큰 길이값이면(노이즈로 헤더가 우연히 맞은 경우 등)
                {
                    radar_state = RADAR_WAIT_HEADER;              // 이 프레임은 버리고 처음부터 헤더 재탐색
                    radar_header_match = 0;
                }
                else                                              // 정상 범위의 길이값이면
                {
                    radar_state = RADAR_WAIT_PAYLOAD;              // 페이로드 수신 단계로 전환
                    radar_payload_idx = 0;                         // 페이로드를 채울 인덱스 초기화
                }
            }
            break;

        case RADAR_WAIT_PAYLOAD:                            // 페이로드를 길이필드만큼 받는 중인 단계
            radar_payload_buf[radar_payload_idx++] = b;        // 받은 바이트를 페이로드 버퍼에 저장하고 인덱스 증가
            if (radar_payload_idx >= radar_payload_len)         // 정해진 길이만큼 다 받았으면
            {
                radar_state = RADAR_WAIT_TAIL;                   // 테일 확인 단계로 전환
                radar_tail_idx = 0;                              // 테일 인덱스 초기화
            }
            break;

        case RADAR_WAIT_TAIL:                               // 테일 4바이트가 맞는지 확인하는 단계
            if (b == RADAR_TAIL[radar_tail_idx])                // 기대하는 테일 바이트와 일치하면
            {
                radar_tail_idx++;                                // 일치 카운트 증가
                if (radar_tail_idx >= RADAR_TAIL_LEN)             // 테일 4바이트가 전부 확인됐으면
                {
                    radar_frame_ready = 1;                         // 완전한 프레임 하나 완성! 메인루프에 신호
                    radar_state = RADAR_WAIT_HEADER;               // 다음 프레임을 위해 처음 상태로 복귀
                    radar_header_match = 0;
                }
            }
            else                                                 // 테일이 기대값과 다르면(깨진 프레임)
            {
                radar_state = RADAR_WAIT_HEADER;                    // 이번 프레임은 버리고 헤더 재탐색으로 복귀
                radar_header_match = (b == RADAR_HEADER[0]) ? 1 : 0; // 이 바이트가 다음 프레임의 시작일 수도 있으니 체크
            }
            break;
    }
}

// DMA 순환버퍼에 새로 쌓인 바이트들을 메인루프에서 꺼내 하나씩 RadarProcessByte()에 넘겨 처리하는 함수
// DMA가 계속 radar_dma_buf를 채우므로, 여기선 지금까지 채워진 위치만 계산해서 아직 처리 안 한 구간만 꺼내 씀
// (원형버퍼라 끝에 도달하면 다시 처음으로 돌아감)
void RadarProcessDmaBuffer(void)
{
    uint16_t dma_write_pos = RADAR_DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    // __HAL_DMA_GET_COUNTER: DMA가 "앞으로 몇 바이트 더 채울 수 있는지(남은 카운트)"를 알려줌
    // 전체 버퍼크기에서 그 남은 카운트를 빼면, 지금까지 DMA가 실제로 채워넣은 위치(인덱스)가 나옴

    while (radar_dma_last_pos != dma_write_pos)          // 아직 처리 안 한 새 바이트가 있는 동안 계속 반복
    {
        RadarProcessByte(radar_dma_buf[radar_dma_last_pos]);  // 그 위치의 바이트를 상태기계로 처리
        radar_dma_last_pos = (radar_dma_last_pos + 1) % RADAR_DMA_BUF_SIZE;  // 다음 위치로 이동 (끝에 닿으면 0으로 순환)
    }
}

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
    // USART1(레이더)은 DMA 순환수신 방식이라 여기서 바이트 단위로 처리하지 않음
    // (메인루프의 RadarProcessDmaBuffer()가 이미 처리하므로 이 콜백에서 USART1은 따로 할 일 없음)
    else if (huart->Instance == USART2)      // USART2(Pi)에서 발생한 인터럽트인 경우
    {
        char c = (char)pi_rx_byte;           // 받은 1바이트를 문자로 변환
        // 메인루프가 이전 줄("B,...")을 아직 다 못 읽었는데 다음 줄("C,...")이 덮어써지는 레이스 컨디션이 있었음
        // (덮어쓰이면 sscanf 파싱이 깨진 문자열을 읽어 실패하고, pi_anomaly_flag가 과거값에 멈춰 WARN이 안 뜰 수 있음)
        // → pi_line_ready==1인 동안은 새 바이트를 버퍼에 안 쓰고 그냥 버림 (한 줄 스킵이 덮어쓰기보다 안전)
        if (!pi_line_ready)
        {
            if (c == '\n' || c == '\r')          // 줄바꿈 문자를 만나면 (Pi는 파이썬이라 \n을 씀)
            {
                if (pi_rx_index > 0)             // 빈 줄이 아니라 실제로 뭔가 받은 경우에만
                {
                    pi_rx_line[pi_rx_index] = '\0';  // 문자열 끝에 널문자 추가
                    pi_line_ready = 1;            // 한 줄 완성 깃발 올림 (메인루프가 처리할 때까지 유지됨)
                    pi_rx_index = 0;              // 다음 줄 받을 준비(단, pi_line_ready가 0이 되기 전까진 위에서 걸러짐)
                }
            }
            else if (pi_rx_index < PI_RX_LINE_MAX - 1)  // 버퍼에 공간 남아있으면
            {
                pi_rx_line[pi_rx_index++] = c;   // 문자 저장하고 인덱스 증가
            }
        }
        // else: pi_line_ready==1인 동안 들어오는 바이트는 그냥 버림 (덮어쓰기 방지)
        HAL_UART_Receive_IT(&huart2, (uint8_t*)&pi_rx_byte, 1);  // USART2 다음 바이트 재무장
    }
}
// 5개 값의 중간값(median) 반환 — 연속 2프레임까지 튀는 노이즈를 걸러내기 위한 스무딩
static float Median5(float v[5])
{
    float a[5]; for (uint8_t i = 0; i < 5; i++) a[i] = v[i];    // 원본 순서 보존을 위해 복사본에서 정렬
    for (uint8_t i = 0; i < 5; i++)                             // 단순 버블정렬(5개라 충분히 빠름)
    {
        for (uint8_t j = 0; j < 4 - i; j++)
        {
            if (a[j] > a[j+1]) { float tmp = a[j]; a[j] = a[j+1]; a[j+1] = tmp; }
        }
    }
    return a[2];                                                // 정렬 후 정중앙(인덱스 2) 값
}

// 완성된 레이더 프레임(페이로드)을 해석해서 거리/속도/각도로 변환하는 함수
// ⚠ 길이필드 엔디안(리틀/빅)은 실측 후 재확인 필요
void ParseRadarFrame(void)
{
    if (radar_payload_len < 1) return;                    // 최소한 타겟개수 1바이트는 있어야 함, 없으면 그냥 종료

    uint8_t target_count = radar_payload_buf[0];           // 페이로드 첫 바이트 = 이번 프레임에 포함된 타겟 개수

    if (target_count == 0 || target_count > RADAR_MAX_TARGET)  // 타겟이 0개거나 비정상적으로 많으면
    {
        radar_distance_m = -1.0f;                            // "유효한 타겟 없음" 상태로 표시해둠
        radar_track_distance_m = -1.0f;                       // 타겟이 사라졌으니 트랙(연속성)도 리셋 — 다음 타겟은 새 차일 수 있음
        radar_hist_count = 0;                                 // median 버퍼도 리셋 (끊긴 값을 다음 차 값과 섞으면 안 되니까)
        radar_dist_hist_count = 0;                            // 거리 median 버퍼도 같이 리셋
        return;
    }

    // 필요한 총 바이트수(타겟개수 1바이트 + 타겟당 6바이트*개수)가 실제 받은 페이로드 길이와 맞는지 확인
    uint16_t expected_len = 1 + (uint16_t)target_count * RADAR_TARGET_SZ;
    if (expected_len > radar_payload_len) return;            // 길이가 안 맞으면(깨진 데이터일 가능성) 그냥 무시하고 종료

    // 타겟 선택: 매 프레임 "각도가 0에 가장 가까운 타겟"만으로 고르면, 각도가 비슷한 서로 다른 물체
    // (옆차/도로 옆 구조물) 사이에서 튀는 문제가 있었음 (실측: 거리값 2m→47m처럼 순간이동)
    // → 1순위: 이전 프레임에서 추적하던 거리값과 비슷한(±RADAR_TRACK_GATE_M 이내) 타겟을 최우선 선택 (연속성 유지)
    //    2순위: 그런 후보가 없으면(트랙 없음 또는 새 차) 각도 0에 가장 가까운 타겟으로 재탐색
    uint8_t best_idx = 0xFF;                                  // 0xFF = 아직 못 찾음
    if (radar_track_distance_m >= 0.0f)                       // 이전에 추적하던 거리값(트랙)이 있으면
    {
        float best_diff = 1e9f;
        for (uint8_t i = 0; i < target_count; i++)
        {
            const volatile uint8_t *ti = &radar_payload_buf[1 + i * RADAR_TARGET_SZ];
            float dist_i = (float)ti[2];                        // 이 타겟의 원시 거리값(m)
            float diff = dist_i - radar_track_distance_m;
            if (diff < 0) diff = -diff;                          // 절대값
            if (diff <= RADAR_TRACK_GATE_M && diff < best_diff)   // 게이트 안이면서 지금까지 중 가장 가까우면
            {
                best_diff = diff;
                best_idx = i;
            }
        }
    }
    if (best_idx == 0xFF)                                     // 연속성 있는 후보를 못 찾았으면(트랙 없음 또는 새 차) → 각도 기준 재탐색
    {
        int16_t best_abs_angle = 0x7FFF;
        for (uint8_t i = 0; i < target_count; i++)
        {
            const volatile uint8_t *ti = &radar_payload_buf[1 + i * RADAR_TARGET_SZ];
            int16_t angle_i = (int16_t)ti[1] - 0x80;
            int16_t abs_angle_i = (angle_i < 0) ? -angle_i : angle_i;
            if (abs_angle_i > RADAR_ANGLE_GATE_DEG) continue;     // 각도 게이트 — 정면에서 크게 벗어난 물체는 후보 제외 (부채꼴 오탐 방지)
            if (abs_angle_i < best_abs_angle)
            {
                best_abs_angle = abs_angle_i;
                best_idx = i;
            }
        }
    }
    if (best_idx == 0xFF)                                     // 각도 게이트 때문에 후보가 하나도 없으면(전방에 아무것도 없음)
    {                                                           // "유효한 타겟 없음"으로 처리하고 종료
        radar_distance_m = -1.0f;
        radar_track_distance_m = -1.0f;
        radar_hist_count = 0;
        radar_dist_hist_count = 0;
        return;
    }
    const volatile uint8_t *t = &radar_payload_buf[1 + best_idx * RADAR_TARGET_SZ];  // 최종 선택된 타겟 블록

    uint8_t alarm_info     = t[0];                            // 경보정보 (0x01 = 접근중 알람) — 지금은 참고용
    int16_t angle_raw      = (int16_t)t[1] - 0x80;             // 각도 = 원시값 - 0x80 (문서 기준 오프셋 인코딩)
    uint8_t distance_m_raw = t[2];                              // 거리(미터 단위, 1바이트 그대로)
    uint8_t speed_dir      = t[3];                              // 속도방향: 0x01=접근(가까워짐), 0x00=멀어짐
    uint8_t speed_raw      = t[4];                              // 속도(km/h, 최대 120)
    uint8_t snr            = t[5];                              // 신호대잡음비(0~255) — 지금은 안 쓰지만 나중에 신뢰도 필터링용

    radar_track_distance_m = (float)distance_m_raw;            // 다음 프레임 연속성 판단 기준으로 이번 원시 거리를 저장

    float raw_distance_m = (float)distance_m_raw;
    // speed_dir==0x01이면 멀어짐(양수), 아니면 가까워짐(음수) — 실측으로 부호 방향 확인해서 반영
    float raw_speed_kmh  = (speed_dir == 0x01) ? (float)speed_raw : -(float)speed_raw;
                                                                  // (UpdateFrontCarSpeed에서 "양수=멀어짐" 가정과 맞춤)

    // D(거리), S(속도) 둘 다 median-of-5로 스무딩 — 트랙 연속성+각도게이트가 큰 튐은 막아주고, 이건 잔여 노이즈 제거용
    radar_dist_hist[radar_dist_hist_idx] = raw_distance_m;
    radar_dist_hist_idx = (radar_dist_hist_idx + 1) % 5;
    if (radar_dist_hist_count < 5) radar_dist_hist_count++;

    radar_speed_hist[radar_hist_idx] = raw_speed_kmh;
    radar_hist_idx = (radar_hist_idx + 1) % 5;
    if (radar_hist_count < 5) radar_hist_count++;

    radar_distance_m = (radar_dist_hist_count < 5) ? raw_distance_m : Median5(radar_dist_hist);
    radar_speed_kmh  = (radar_hist_count < 5) ? raw_speed_kmh : Median5(radar_speed_hist);
                                                                  // 둘 다 5개 안 모였으면(막 시작/트랙 재시작 직후) 최신값 그대로, 모이면 median
    radar_angle_deg  = (int8_t)angle_raw;                       // 각도 저장 (스무딩 없이 그대로 — 표시/게이팅용으로만 쓰여서 큰 영향 없음)

    (void)alarm_info; (void)snr;                                // 아직 안 쓰는 값들이라 컴파일 경고 방지용으로 캐스팅
}
// STM32가 Pi에게 레이더 데이터를 보내는 함수
void Pi_SendRadarData(void)
{
    char msg[48];                            // 보낼 메시지를 담을 임시 문자 배열
    // 각도값도 같이 보내줌 (Pi에서 레이더 각도 → 카메라 화면 픽셀 위치로 변환해서 ROI를 좁히는 데 씀)
    int len = snprintf(msg, sizeof(msg), "R,%.1f,%.1f,%d\r\n", radar_distance_m, radar_speed_kmh, (int)radar_angle_deg);
    // "R,거리,상대속도,각도\r\n" 형식의 문자열을 만듦, snprintf는 printf처럼 포맷팅해서 문자열로 만들어줌
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
// Pi가 매 프레임 보내는 "C,검출여부"(카메라가 지금 차를 보고 있는지)를 파싱하는 함수
// (이상감지 판정 로직에는 관여하지 않고 LCD 표시 전용으로만 씀)
int Pi_ParseCameraStatus(const char *raw)
{
    if (raw[0] != 'C' || raw[1] != ',')     // "C,"로 시작하지 않으면 형식 에러
    {
        return -1;
    }
    int detected_val;                        // 파싱될 검출여부(0/1)
    int parsed = sscanf(raw + 2, "%d", &detected_val);
    if (parsed != 1)
    {
        return -1;
    }
    cam_detected = (uint8_t)detected_val;    // 전역변수에 저장 (LCD_UpdateStatus에서 표시함)
    pi_cam_msgs++;                           // C, 파싱 성공 횟수 카운트
    return 0;
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
// 히스테리시스 상태(0=OFF, 1=ON) — 함수 호출 사이에 유지되어야 하므로 static
static uint8_t decel_hysteresis_state = 0;
// 최근 이력을 보고 "지금 앞차가 감속 중인지"를 판정하는 함수 (히스테리시스 적용)
uint8_t CheckDeceleration(void)
{
    if (history_count < 2)                   // 비교할 만큼 이력이 충분히 쌓이지 않았으면
    {
        decel_hysteresis_state = 0;          // 데이터 자체가 부족하니 안전하게 OFF로 리셋
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
        decel_hysteresis_state = 0;          // 마찬가지로 데이터 부족이니 안전하게 OFF로 리셋
        return 0;                            // 판단 불가, 감속 아님으로 처리
    }
    float speed_drop = oldest_speed_in_window - front_car_speed_kmh;  // 예전속도 - 현재속도 = 감속한 정도
    if (speed_drop >= DECEL_THRESHOLD_KMH)           // 진입기준(4.0) 이상이면 확실히 ON
    {
        decel_hysteresis_state = 1;
    }
    else if (speed_drop <= DECEL_EXIT_THRESHOLD_KMH) // 해제기준(2.5) 이하로 확실히 떨어지면 OFF
    {
        decel_hysteresis_state = 0;
    }
    // else: 2.5~4.0 사이(회색지대)는 이전 상태를 그대로 유지 — 여기가 히스테리시스의 핵심
    return decel_hysteresis_state;
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
    dbg_brake_missing = brake_light_missing;  // 이번 판정 결과를 그대로 저장 (LCD 표시용)
    if (is_decelerating && brake_light_missing)  // 감속 중인데 + 제동등은 안 켜진 것으로 보이면 (모순 상황)
    {
        if (anomaly_start_tick == 0)          // 이 모순 상황이 방금 막 시작된 거라면 (아직 시작시각 기록 안 됨)
        {
            anomaly_start_tick = HAL_GetTick();  // 지금 시각을 "이상 의심 시작 시각"으로 기록
        }
        anomaly_last_true_tick = HAL_GetTick();  // 방금 조건이 참이었다고 기록 (grace period 계산 기준)
        if (HAL_GetTick() - anomaly_start_tick >= ANOMALY_PERSIST_MS)
        // 이 모순 상황이 시작된 후로 설정한 지속시간(예: 500ms) 이상 계속됐으면
        {
            anomaly_confirmed = 1;            // 최종적으로 "이상"이라고 확정
        }
    }
    else                                       // 감속 중이 아니거나, 제동등이 정상으로 보이면 (모순 상황 해소로 "보일" 수 있음)
    {
        // 바로 리셋하면 프레임 한두 개만 노이즈로 조건 벗어나도 카운트가 매번 0부터 다시 시작돼서
        // 실제 지속 이상상황도 WARN까지 못 가는 문제가 있었음
        // → 마지막으로 조건이 참이었던 시점으로부터 ANOMALY_GRACE_MS(300ms) 이내면 일시적 끊김으로 보고 카운트 유지
        if (anomaly_start_tick != 0 && (HAL_GetTick() - anomaly_last_true_tick) < ANOMALY_GRACE_MS)
        {
            // 카운트 유지 (start_tick도, confirmed 상태도 안 건드림) — grace period 안이므로 계속 지속중인 걸로 취급
        }
        else
        {
            anomaly_start_tick = 0;               // 시작시각 초기화 (다음에 새로 감지되면 처음부터 다시 카운트)
            anomaly_confirmed = 0;                // 이상 확정 상태도 해제
        }
    }
    // 지금 "감속+제동등꺼짐" 모순이 몇 ms째 지속중인지 계산해서 LCD로 표시 (0이면 모순 아님)
    dbg_anomaly_ms = (anomaly_start_tick != 0) ? (HAL_GetTick() - anomaly_start_tick) : 0;
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

// ============================================================================
// ===== 간단한 3x5 비트맵 폰트로 글자/숫자 그리기 =====
// 필요한 문자만 최소한으로 넣어둠: 숫자 0~9, '-' '.' ':' 공백, 그리고 표시 문구에 쓰이는 알파벳들
// 각 문자는 5행 x 3열 크기, rows[]의 각 바이트 하위 3비트가 그 행의 픽셀 패턴 (왼쪽부터 bit2,bit1,bit0)
// ============================================================================
typedef struct {
    char ch;             // 이 글리프가 어떤 문자를 나타내는지
    uint8_t rows[5];      // 5개 행의 비트패턴 (각 0~0b111)
} FontGlyph;

static const FontGlyph FONT_TABLE[] = {
    {'0', {0b111,0b101,0b101,0b101,0b111}},
    {'1', {0b010,0b110,0b010,0b010,0b111}},
    {'2', {0b111,0b001,0b111,0b100,0b111}},
    {'3', {0b111,0b001,0b111,0b001,0b111}},
    {'4', {0b101,0b101,0b111,0b001,0b001}},
    {'5', {0b111,0b100,0b111,0b001,0b111}},
    {'6', {0b111,0b100,0b111,0b101,0b111}},
    {'7', {0b111,0b001,0b001,0b001,0b001}},
    {'8', {0b111,0b101,0b111,0b101,0b111}},
    {'9', {0b111,0b101,0b111,0b001,0b111}},
    {'-', {0b000,0b000,0b111,0b000,0b000}},
    {'.', {0b000,0b000,0b000,0b000,0b010}},
    {':', {0b000,0b010,0b000,0b010,0b000}},
    {' ', {0b000,0b000,0b000,0b000,0b000}},
    {'D', {0b110,0b101,0b101,0b101,0b110}},
    {'S', {0b111,0b100,0b111,0b001,0b111}},
    {'W', {0b101,0b101,0b101,0b111,0b101}},
    {'A', {0b010,0b101,0b111,0b101,0b101}},
    {'R', {0b110,0b101,0b110,0b101,0b101}},
    {'N', {0b101,0b111,0b111,0b111,0b101}},
    {'O', {0b111,0b101,0b101,0b101,0b111}},
    {'K', {0b101,0b101,0b110,0b101,0b101}},
    {'M', {0b101,0b111,0b111,0b101,0b101}},
    {'H', {0b101,0b101,0b111,0b101,0b101}},
    {'E', {0b111,0b100,0b111,0b100,0b111}},  // 3번째 줄(디버그 표시)에 필요
    {'C', {0b111,0b100,0b100,0b100,0b111}},  // 3번째 줄(디버그 표시)에 필요
    {'L', {0b100,0b100,0b100,0b100,0b111}},  // 3번째 줄(디버그 표시)에 필요
    {'B', {0b110,0b101,0b111,0b101,0b110}},
};
#define FONT_TABLE_LEN (sizeof(FONT_TABLE) / sizeof(FONT_TABLE[0]))  // 폰트 테이블에 등록된 글리프 개수

// 문자 하나에 해당하는 비트패턴(5바이트)을 찾아 반환, 못 찾으면 NULL(=공백으로 취급)
static const uint8_t* FindGlyph(char c)
{
    for (uint8_t i = 0; i < FONT_TABLE_LEN; i++)     // 폰트 테이블을 처음부터 순서대로 검사
    {
        if (FONT_TABLE[i].ch == c)                     // 찾는 문자와 일치하면
        {
            return FONT_TABLE[i].rows;                   // 그 글리프의 비트패턴 5바이트를 반환
        }
    }
    return NULL;                                       // 등록 안 된 문자면 NULL (공백처럼 그려짐)
}

// 문자 하나를 (x,y) 위치(왼쪽위 기준)에 그리는 함수. size는 확대배율 (1이면 3x5픽셀, 4면 12x20픽셀)
void LCD_DrawChar(uint16_t x, uint16_t y, char c, uint16_t fg_color, uint16_t bg_color, uint8_t size)
{
    const uint8_t *glyph = FindGlyph(c);      // 이 문자의 비트패턴 찾기 (없으면 NULL=빈칸)
    uint16_t w = 3 * size;                    // 이 문자가 화면에서 차지할 가로 픽셀 수
    uint16_t h = 5 * size;                    // 이 문자가 화면에서 차지할 세로 픽셀 수

    LCD_WriteCommand(0x2A);                   // Column Address Set: 이 문자가 그려질 가로범위 지정
    LCD_WriteData(x >> 8);
    LCD_WriteData(x & 0xFF);
    LCD_WriteData((x + w - 1) >> 8);
    LCD_WriteData((x + w - 1) & 0xFF);
    LCD_WriteCommand(0x2B);                   // Row Address Set: 이 문자가 그려질 세로범위 지정
    LCD_WriteData(y >> 8);
    LCD_WriteData(y & 0xFF);
    LCD_WriteData((y + h - 1) >> 8);
    LCD_WriteData((y + h - 1) & 0xFF);
    LCD_WriteCommand(0x2C);                   // Memory Write: 지금부터 픽셀 데이터 전송 시작

    HAL_GPIO_WritePin(LCD_DC_GPIO_Port, LCD_DC_Pin, GPIO_PIN_SET);    // DC HIGH = 데이터 모드
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);  // CS LOW = 칩 선택 (문자 하나 다 그릴 때까지 유지)

    uint8_t fg_bytes[2] = { (uint8_t)(fg_color >> 8), (uint8_t)(fg_color & 0xFF) };  // 글자색 RGB565를 2바이트로
    uint8_t bg_bytes[2] = { (uint8_t)(bg_color >> 8), (uint8_t)(bg_color & 0xFF) };  // 배경색 RGB565를 2바이트로

    for (uint8_t row = 0; row < 5; row++)                    // 폰트의 5개 행을 위에서 아래로
    {
        uint8_t bits = glyph ? glyph[row] : 0;                  // 이 행의 3비트 패턴 (glyph가 NULL이면 전부 꺼짐=공백)
        for (uint8_t ry = 0; ry < size; ry++)                   // 세로로 size배 확대해서 반복
        {
            for (uint8_t col = 0; col < 3; col++)                 // 폰트의 3개 열을 왼쪽부터
            {
                uint8_t on = (bits >> (2 - col)) & 0x01;            // 그 칸이 켜져있는지(1) 꺼져있는지(0)
                for (uint8_t rx = 0; rx < size; rx++)                 // 가로로 size배 확대해서 반복
                {
                    HAL_SPI_Transmit(&hspi1, on ? fg_bytes : bg_bytes, 2, HAL_MAX_DELAY);  // 켜진 칸이면 글자색, 꺼진 칸이면 배경색으로 픽셀 전송
                }
            }
        }
    }
    HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);    // 문자 다 그렸으면 CS 다시 HIGH로
}

// 문자열을 (x,y)부터 가로로 이어서 그리는 함수
void LCD_DrawString(uint16_t x, uint16_t y, const char *str, uint16_t fg_color, uint16_t bg_color, uint8_t size)
{
    uint16_t cursor_x = x;                        // 지금 그릴 위치(가로), 처음엔 시작 x좌표
    while (*str)                                  // 문자열 끝(널문자)에 도달할 때까지 반복
    {
        LCD_DrawChar(cursor_x, y, *str, fg_color, bg_color, size);  // 현재 위치에 문자 하나 그리기
        cursor_x += (3 * size) + size;               // 다음 문자 위치로 이동 (문자폭 + 글자사이 여백 1칸)
        str++;                                       // 다음 문자로
    }
}

// 지금까지 계산된 거리/속도/경고상태를 화면에 표시하는 함수
void LCD_UpdateStatus(void)
{
    char line1[16];    // "D:123.4M" 같은 거리/속도 정보 한 줄
    char line2[8];     // "WARN" 또는 "OK  " 경고상태 한 줄
    char line3[16];    // ELM327 연결여부 + 감속판정여부 + 제동등꺼짐판정(B) 표시

    if (radar_distance_m < 0)                                     // 아직 유효한 레이더 타겟이 없으면
    {
        // 정상 라인("D:%3d.%1dM S:%4d", 15자 고정)과 자릿수를 똑같이 맞춰야 이전 프레임 숫자 잔상이 안 남음
        snprintf(line1, sizeof(line1), "D:---.-M S:----");           // 값 대신 "-"로 채우되 자릿수는 정상 라인과 동일하게
    }
    else                                                          // 유효한 타겟이 있으면
    {
        int dist_int  = (int)radar_distance_m;                       // 거리의 정수부
        int dist_frac = (int)((radar_distance_m - dist_int) * 10);   // 거리의 소수 첫째자리 (float printf 없이 직접 계산)
        if (dist_frac < 0) dist_frac = -dist_frac;                   // 혹시 음수 반올림 오차 나오면 절대값 처리
        int speed_int = (int)radar_speed_kmh;                        // 속도는 소수점 없이 정수로만 표시
        snprintf(line1, sizeof(line1), "D:%3d.%1dM S:%4d", dist_int, dist_frac, speed_int);
        // %3d, %4d로 자리수를 고정해서, 이전에 더 길게 그렸던 숫자의 잔상이 안 남게 함
    }

    uint16_t warn_color;                                          // 경고 문구 색상 (상태에 따라 초록/빨강)
    if (anomaly_confirmed)                                        // 최종 이상 확정 상태면
    {
        snprintf(line2, sizeof(line2), "WARN");
        warn_color = 0xF800;                                        // 빨강 (RGB565)
    }
    else                                                          // 정상 상태면
    {
        snprintf(line2, sizeof(line2), "OK  ");                     // "OK"도 4글자로 맞춰서 잔상 방지
        warn_color = 0x07E0;                                        // 초록 (RGB565)
    }

    LCD_DrawString(10, 40, line1, 0xFFFF, 0x0000, 3);              // 거리/속도: 흰 글씨, 검정 배경, 3배 확대

    // OK/WARN을 크게(8배), 가로 중앙 정렬
    // "OK  "/"WARN" 둘 다 4글자 고정폭이라 매번 같은 x로 중앙정렬 가능
    // (글자폭 공식: 문자 1개당 4*size픽셀, 마지막 글자 뒤 여백은 안 빼므로 폭 = 4*size*글자수 - size)
    {
        uint8_t warn_size = 8;
        uint16_t text_w = 4 * warn_size * 4 - warn_size;           // 4글자 기준 폭 계산
        uint16_t warn_x = (240 - text_w) / 2;                        // 240 = LCD 가로 해상도, 화면 중앙에 오도록 x 계산
        LCD_DrawString(warn_x, 90, line2, warn_color, 0x0000, warn_size);  // 경고문구: 상태색 글씨, 검정 배경, 8배 확대(크게), 가로 중앙
    }

    // 3번째 줄: ELM327 연결여부(E:C=연결/E:L=없음), 감속판정(DECEL/-----), 제동등 상태(B:1=켜짐/B:0=꺼짐)
    // (내부 판정 변수 dbg_brake_missing은 그대로 두고, 표시할 때만 뒤집어서 "켜지면 1"로 직관적으로 보이게 함)
    snprintf(line3, sizeof(line3), "E:%c %s B:%c", elm_data_valid ? 'C' : 'L', is_decelerating ? "DECEL" : "-----", dbg_brake_missing ? '0' : '1');
    LCD_DrawString(10, 180, line3, 0xFFE0, 0x0000, 3);             // 노란 글씨, 검정 배경, 3배 확대
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
