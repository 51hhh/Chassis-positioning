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

#include "main.h"
#include "gpio.h"
#include "dma.h"
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "YIS130.h"
#include "arm_math.h"
#include "stdio.h"
#include "AS5048.h"
#include "odom_protocol.h"
#include <math.h>
#include <string.h>

extern CAN_HandleTypeDef hcan1;
extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern UART_HandleTypeDef huart1;

void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_CAN1_Init(void);
void MX_TIM11_Init(void);
void MX_TIM13_Init(void);
void MX_TIM14_Init(void);
void MX_USART1_UART_Init(void);
void MX_SPI1_Init(void);
void MX_SPI2_Init(void);

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define USART_CR3_RXFTIE ((uint32_t)0x00008000)

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t receive_buff[8] = {0}; // init
int receivefactor[2];
// header x_low x_high y y yaw yaw footer
uint8_t tx_buff[255];


extern MPU_DATA mpu_data[4];
extern AS5048 AS5048s[AS5048_NUMBER];
float i = 0;
extern float ACCX,ACCY,ACCZ;
//float tt_x = 0;
//float tt_y = 0;
//float tt_x_real = 0;
//float tt_y_real = 0;


int add = 0;
int times = 0;

int fputc(int ch, FILE *f){
        //HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&ch, 1);
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

typedef struct struct_message
{
  uint8_t header;
  uint8_t parity;
  uint8_t data[6];
  uint8_t footer;
} DataPacket;

DataPacket DataRe;
uint8_t USART_FLAG = 0;


uint8_t USART1_RX_BUF[100]; 
uint16_t USART1_RX_STA = 0; 
uint8_t aRxBuffer1[1];            
UART_HandleTypeDef UART1_Handler; 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* RX 缓冲扩大到 64 字节以兼容旧 8 字节协议 + 新 0xAA 0x55 上行帧 (最长 25 字节) */
#define RCV_BUF_SIZE 64
uint8_t rcv_buf[RCV_BUF_SIZE] = {0};
volatile uint16_t rcv_len = 0;   /* 实际收到字节数, 由 idle 中断记录 */
int rcv_err = 3;
char mpu_buff[220];
uint16_t rxclear = 0;
volatile uint8_t rcv_flag = 0;
int rst_temp = 0;

/* ODOM protocol variables */
static uint8_t odom_seq = 0;
static uint8_t odom_frame_buf[2][ODOM_STATE_FRAME_LEN];
static uint8_t odom_frame_buf_index = 0;
static uint32_t odom_tx_drop_count = 0;
/* 上行响应使用独立缓冲, 避免与下行 ODOM_STATE 共用 */
static uint8_t odom_resp_buf[ODOM_TIME_SYNC_RESP_FRAME_LEN];
/* 累计成功归零次数 (event_counter) */
static uint32_t origin_event_counter = 0;
static float odom_dx_world_acc = 0.0f;
static float odom_dy_world_acc = 0.0f;
static float odom_dyaw_acc = 0.0f;
static uint16_t odom_vel_ticks = 0;
static volatile uint8_t encoder_sample_pending = 0;
static volatile uint8_t encoder_delta_ready = 0;
static uint16_t encoder_sample_divider = 0;
static float encoder_odom_last_yaw_rad = 0.0f;
static uint8_t encoder_odom_yaw_initialized = 0;

/* ODOM output rate: ticks between frames (ISR=20kHz, 100 ticks=5ms=200Hz) */
#define ODOM_OUTPUT_TICKS 100
/* Encoder SPI sampling rate: 20kHz / 20 = 1kHz */
#define ODOM_ENCODER_SAMPLE_TICKS 20U
/* ISR period in microseconds */
#define ODOM_ISR_PERIOD_US 50
#define ODOM_PI_F 3.1415926f
#define DEG_TO_RAD_F (3.1415926f / 180.0f)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void service_encoder_sampling(void);

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
  MX_CAN1_Init();
  MX_TIM11_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

        __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        HAL_UART_Receive_DMA(&huart1, rcv_buf, RCV_BUF_SIZE);

        AS5048_init(1,&hspi1,GPIOA,GPIO_PIN_4);
        AS5048_init(2,&hspi2,GPIOB,GPIO_PIN_12);

        mpu_data[0].cali = 1;
        mpu_data[0].vel[0] = 0;
        mpu_data[0].vel[1] = 0;
        mpu_data[0].REAL_YAW_SET = 0;
        mpu_data[0].REAL_YAW_MARK = 0;

        can_filter_init();

        HAL_TIM_Base_Start_IT(&htim13);
        HAL_TIM_Base_Start_IT(&htim14);
        HAL_TIM_Base_Start_IT(&htim11);


//      SelfCalibration();

//      HAL_Delay(100);


//  mpu_data[0].PITCH_ANGLE_BEG = mpu_data[0].PITCH_ANGLE;
//  mpu_data[0].YAW_ANGLE_BEG =   mpu_data[0].YAW_ANGLE;
//  mpu_data[0].ROLL_ANGLE_BEG =  mpu_data[0].ROLL_ANGLE;
//
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


        while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
                service_encoder_sampling();
                if(rcv_flag != 0U){
                        Rcv_DealData();
                }

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


//DMA+空闲中断接收
void Rcv_IdleCallback(void){
        /* 判断是否为空闲中断 */
        if(__HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE) == SET){
                /* 清除空闲中断标志并停止 DMA */
                __HAL_UART_CLEAR_IDLEFLAG(&huart1);
                HAL_UART_DMAStop(&huart1);
                /* 计算实际接收字节数 = 缓冲总长 - 剩余 NDTR */
                uint16_t remaining = (uint16_t)__HAL_DMA_GET_COUNTER(huart1.hdmarx);
                rcv_len = (remaining > RCV_BUF_SIZE) ? 0 : (uint16_t)(RCV_BUF_SIZE - remaining);
                /* 标记收到一帧 */
                rcv_flag = 1;
        }
}

/* 等待上一次串口 TX (DMA 或阻塞) 完成, 然后用 DMA 发送响应帧。*/
static void send_upstream_response(const uint8_t *buf, uint16_t len)
{
        uint32_t guard = 0;
        /* gState != HAL_UART_STATE_READY 表示 TX 还在进行 */
        while(huart1.gState != HAL_UART_STATE_READY){
                if(++guard > 200000U){ return; }  /* 超时直接放弃, 避免 ISR 死循环 */
        }
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len);
}

static void send_odom_state_payload(const OdomStatePayload_t *payload)
{
        uint8_t *frame_buf;
        uint16_t frame_len;

        if(huart1.gState != HAL_UART_STATE_READY){
                odom_tx_drop_count++;
                return;
        }

        frame_buf = odom_frame_buf[odom_frame_buf_index];
        frame_len = odom_pack_state(frame_buf, odom_seq, payload);
        if(HAL_UART_Transmit_DMA(&huart1, frame_buf, frame_len) == HAL_OK){
                odom_seq++;
                odom_frame_buf_index ^= 1U;
        }else{
                odom_tx_drop_count++;
        }
}

static void request_encoder_sample_from_isr(void)
{
        encoder_sample_divider++;
        if(encoder_sample_divider >= ODOM_ENCODER_SAMPLE_TICKS){
                encoder_sample_divider = 0;
                if(encoder_sample_pending < 0xFFU){
                        encoder_sample_pending++;
                }
        }
}

static uint8_t take_encoder_sample_request(void)
{
        uint8_t do_sample = 0U;

        __disable_irq();
        if(encoder_sample_pending != 0U){
                encoder_sample_pending--;
                do_sample = 1U;
        }
        __enable_irq();

        return do_sample;
}

static void service_encoder_sampling(void)
{
        if(encoder_delta_ready != 0U){
                return;
        }
        if(take_encoder_sample_request() == 0U){
                return;
        }

        AS5048_getREGValue(1);
        AS5048_dataUpdate(1);
        AS5048_getREGValue(2);
        AS5048_dataUpdate(2);
        encoder_delta_ready = 1U;
}

static uint8_t take_encoder_delta_ready(void)
{
        if(encoder_delta_ready == 0U){
                return 0U;
        }
        encoder_delta_ready = 0U;
        return 1U;
}

static float odom_wrap_pi(float angle)
{
        while(angle > ODOM_PI_F){
                angle -= 2.0f * ODOM_PI_F;
        }
        while(angle < -ODOM_PI_F){
                angle += 2.0f * ODOM_PI_F;
        }
        return angle;
}

static void reset_encoder_odom_yaw_reference(void)
{
        encoder_odom_last_yaw_rad = mpu_data[0].YAW_ANGLE * DEG_TO_RAD_F;
        encoder_odom_yaw_initialized = 1U;
}

static void integrate_orthogonal_encoder_delta(void)
{
        float yaw_rad = mpu_data[0].YAW_ANGLE * DEG_TO_RAD_F;
        float dyaw = 0.0f;
        float dx_wheel = (float)AS5048s[ODOM_X_ENCODER_INDEX].delta_dis
                       * ODOM_X_ENCODER_SIGN
                       * ODOM_X_WHEEL_METERS_PER_COUNT;
        float dy_wheel = (float)AS5048s[ODOM_Y_ENCODER_INDEX].delta_dis
                       * ODOM_Y_ENCODER_SIGN
                       * ODOM_Y_WHEEL_METERS_PER_COUNT;

        if(encoder_odom_yaw_initialized == 0U){
                encoder_odom_last_yaw_rad = yaw_rad;
                encoder_odom_yaw_initialized = 1U;
        }else{
                dyaw = odom_wrap_pi(yaw_rad - encoder_odom_last_yaw_rad);
                encoder_odom_last_yaw_rad = yaw_rad;
        }

        /* Rotation compensation for tracking wheels offset from robot center. */
        float dx_body = dx_wheel + ODOM_X_WHEEL_Y_OFFSET_M * dyaw;
        float dy_body = dy_wheel - ODOM_Y_WHEEL_X_OFFSET_M * dyaw;
        float cos_yaw = arm_cos_f32(yaw_rad);
        float sin_yaw = arm_sin_f32(yaw_rad);
        float dx_world = dx_body * cos_yaw - dy_body * sin_yaw;
        float dy_world = dx_body * sin_yaw + dy_body * cos_yaw;

        mpu_data[0].X_tt += dx_world;
        mpu_data[0].Y_tt += dy_world;
        mpu_data[0].REAL_X = mpu_data[0].X_tt;
        mpu_data[0].REAL_Y = mpu_data[0].Y_tt;

        odom_dx_world_acc += dx_world;
        odom_dy_world_acc += dy_world;
        odom_vel_ticks += ODOM_ENCODER_SAMPLE_TICKS;
}

static void reset_odom_runtime_accumulators(void)
{
        odom_dx_world_acc = 0.0f;
        odom_dy_world_acc = 0.0f;
        odom_dyaw_acc = 0.0f;
        odom_vel_ticks = 0;
        encoder_odom_yaw_initialized = 0U;
        mpu_data[0].vel[0] = 0.0;
        mpu_data[0].vel[1] = 0.0;
        mpu_data[0].vel[2] = 0.0;
}

static void reset_encoder_accumulators(void)
{
        int idx;
        for(idx = 0; idx < AS5048_NUMBER; idx++){
                AS5048s[idx].total_angle = 0;
                AS5048s[idx].cirle = 0;
                AS5048s[idx].delta_dis = 0;
                AS5048s[idx].last_angle = AS5048s[idx].angle;
                AS5048s[idx].diff_hist[0] = 0;
                AS5048s[idx].diff_hist[1] = 0;
                AS5048s[idx].diff_hist[2] = 0;
                AS5048s[idx].motion_state = 0;
        }
}

static uint8_t is_ball_present(void)
{
        if(HAL_GPIO_ReadPin(BALL_DETECT_GPIO_Port, BALL_DETECT1_Pin) == GPIO_PIN_SET){
                return 1U;
        }
        return (HAL_GPIO_ReadPin(BALL_DETECT_GPIO_Port, BALL_DETECT2_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

/* 处理上位机发来的 0xAA 0x55 帧 */
static void handle_upstream_frame(const OdomUpstreamFrame_t *fr)
{
        if(fr->msg_type == ODOM_MSG_TIME_SYNC_REQ){
                if(fr->payload_len != ODOM_TIME_SYNC_REQ_PAYLOAD_LEN) return;
                /* 提取 echoed_host_time_us */
                uint64_t host_us = 0;
                for(int b = 0; b < 8; b++){
                        host_us |= ((uint64_t)fr->payload[b]) << (8 * b);
                }
                OdomTimeSyncRespPayload_t resp;
                resp.echoed_host_time_us = host_us;
                resp.mcu_time_us = odom_isr_tick * (uint64_t)ODOM_ISR_PERIOD_US;
                uint16_t flen = odom_pack_time_sync_resp(odom_resp_buf, fr->seq, &resp);
                send_upstream_response(odom_resp_buf, flen);
        } else if(fr->msg_type == ODOM_MSG_SET_LOCAL_ORIGIN){
                if(fr->payload_len != ODOM_SET_ORIGIN_PAYLOAD_LEN) return;
                /* payload: float x, y, yaw + uint8 flags + 3 reserved */
                float new_x, new_y, new_yaw;
                uint8_t flags;
                memcpy(&new_x,   &fr->payload[0],  4);
                memcpy(&new_y,   &fr->payload[4],  4);
                memcpy(&new_yaw, &fr->payload[8],  4);
                flags = fr->payload[12];
                if(flags == 0U){
                        flags = ODOM_SET_ORIGIN_FLAG_RESET_ALL; /* 兼容旧版上位机: 0 表示整包归零 */
                }

                if((flags & ODOM_SET_ORIGIN_FLAG_RESET_XY) != 0U){
                        /* 应用 XY 原点重置 */
                        mpu_data[0].X_tt = new_x;
                        mpu_data[0].Y_tt = new_y;
                        mpu_data[0].REAL_X = new_x;
                        mpu_data[0].REAL_Y = new_y;
                }

                if((flags & ODOM_SET_ORIGIN_FLAG_RESET_YAW) != 0U){
                        /* 重置连续 yaw: 让下次 unwrap 输出 = new_yaw, prev_deg 锚定到当前角度 */
                        g_yaw_unwrap.continuous_rad = new_yaw;
                        g_yaw_unwrap.prev_deg       = mpu_data[0].YAW_ANGLE;
                        g_yaw_unwrap.initialized    = 1;
                }

                if((flags & ODOM_SET_ORIGIN_FLAG_RESET_ENCODER) != 0U){
                        reset_encoder_accumulators();
                }

                reset_odom_runtime_accumulators();
                origin_event_counter++;

                OdomSetLocalOriginAckPayload_t ack;
                ack.acked_seq     = (uint16_t)fr->seq;
                ack.result_code   = 0;
                ack.event_counter = origin_event_counter;
                uint16_t flen = odom_pack_set_origin_ack(odom_resp_buf, fr->seq, &ack);
                send_upstream_response(odom_resp_buf, flen);
        }
        /* 其它类型忽略 */
}

int Rcv_DealData(void){
        if(1==rcv_flag){
                uint16_t len = rcv_len;
                /* --- 1) 老协议: 8 字节固定包 --- */
                if(len >= 8 && 0x0F==rcv_buf[0] && 0xAA==rcv_buf[7]){
                        DATARELOAD(rcv_buf);
                }else if(len >= 8 && 0xBB==rcv_buf[0] && 0xCC==rcv_buf[7]){
                        HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_SET);
                        HAL_Delay(500);
                        HAL_GPIO_WritePin(RST_CTRL_GPIO_Port,RST_CTRL_Pin,GPIO_PIN_RESET);
                        DATARELOAD(rcv_buf);
                }else{
                        /* --- 2) 新协议: 0xAA 0x55 上行帧 --- */
                        OdomUpstreamFrame_t fr;
                        if(odom_parse_upstream(rcv_buf, len, &fr)){
                                handle_upstream_frame(&fr);
                        }
                }
                /* 清缓冲并重启 DMA */
                memset(rcv_buf, 0, RCV_BUF_SIZE);
                rcv_len  = 0;
                rcv_flag = 0;
                HAL_UART_Receive_DMA(&huart1, rcv_buf, RCV_BUF_SIZE);
                return 0;
        }else{
                return -1;
        }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&htim14)){
                        //if(rcv_err>0){
      //ClearUARTErrors(USART1);//清除串口错误标志，目前不再使用
                        //}

    }

                if (htim == (&htim13)){
//                      arm_fir_f32_lp();

                 }

                 if (htim == (&htim11)){

	                         odom_isr_tick++;
	                         request_encoder_sample_from_isr();

                         if(mpu_data[0].cali == 1){
	                                        if(times >= 500){

	                                                add++;
	                                                uint8_t has_encoder_delta = take_encoder_delta_ready();
	                                                uint32_t imu_now_ms = HAL_GetTick();
	                                                uint8_t gyro_fresh = YIS130_IsGyroFresh(imu_now_ms);
	                                                uint8_t yaw_fresh = YIS130_IsEulerFresh(imu_now_ms);
	                                                uint8_t imu_fresh = YIS130_IsImuFresh(imu_now_ms);

	                                                mpu_data[0].REAL_YAW = mpu_data[0].YAW_ANGLE;

	            if(has_encoder_delta != 0U && yaw_fresh != 0U){
	                integrate_orthogonal_encoder_delta();
	            }
#if ODOM_BINARY_MODE
                                                if(add >= ODOM_OUTPUT_TICKS){
                                                        /* 连续 yaw (rad) */
                                                        float yaw_cont = g_yaw_unwrap.continuous_rad;
                                                        if(yaw_fresh != 0U){
                                                                yaw_cont = odom_yaw_unwrap(&g_yaw_unwrap, mpu_data[0].YAW_ANGLE);
                                                        }

                                                        /* 计算 body-frame 速度 */
                                                        float dt = (float)odom_vel_ticks * (float)ODOM_ISR_PERIOD_US * 1e-6f;
                                                        float vx_world = 0.0f, vy_world = 0.0f;
                                                        if(dt > 0.0f){
                                                            vx_world = odom_dx_world_acc / dt;
                                                            vy_world = odom_dy_world_acc / dt;
                                                        }
                                                        /* 旋转到 body frame */
                                                        float cos_yaw = arm_cos_f32(yaw_cont);
                                                        float sin_yaw = arm_sin_f32(yaw_cont);
                                                        float vx_body =  vx_world * cos_yaw + vy_world * sin_yaw;
                                                        float vy_body = -vx_world * sin_yaw + vy_world * cos_yaw;
                                                        float wz = (gyro_fresh != 0U) ? (mpu_data[0].gyro[2] * DEG_TO_RAD_F) : 0.0f; /* YIS130 gyro Z: deg/s -> rad/s */

                                                        /* 时间戳 */
                                                        uint64_t t_us = odom_isr_tick * (uint64_t)ODOM_ISR_PERIOD_US;

                                                        /* 状态位 */
                                                        uint8_t enc_valid = ((AS5048s[0].valid != 0U) && (AS5048s[1].valid != 0U)) ? 1U : 0U;
                                                        uint8_t pose_valid = (enc_valid != 0U && yaw_fresh != 0U) ? 1U : 0U;
                                                        uint16_t status = 0U;
                                                        uint8_t quality = ODOM_QUALITY_NORMAL;
                                                        if(imu_fresh != 0U){
                                                                status |= ODOM_STATUS_IMU_VALID;
                                                        }
                                                        if(yaw_fresh != 0U){
                                                                status |= ODOM_STATUS_YAW_VALID;
                                                        }
                                                        if(enc_valid != 0U){
                                                                status |= ODOM_STATUS_ENC_VALID;
                                                        }
                                                        if(pose_valid != 0U && imu_fresh != 0U){
                                                                status |= ODOM_STATUS_POS_VALID | ODOM_STATUS_VEL_VALID;
                                                        }else{
                                                                status |= ODOM_STATUS_DEGRADED;
                                                                quality = ODOM_QUALITY_DEGRADED;
                                                        }
                                                                                                                if(is_ball_present() != 0U){
                                                                                                                        status |= ODOM_STATUS_BALL_PRESENT;
                                                                                                                }

                                                        /* 打包 ODOM_STATE */
                                                        OdomStatePayload_t payload;
                                                        payload.t_sample_us = t_us;
                                                        payload.x = mpu_data[0].REAL_X;
                                                        payload.y = mpu_data[0].REAL_Y;
                                                        payload.yaw = yaw_cont;
                                                        payload.vx = vx_body;
                                                        payload.vy = vy_body;
                                                        payload.wz = wz;
                                                        payload.status_bits = status;
                                                        payload.quality = quality;
                                                        payload.reserved = 0;

                                                        send_odom_state_payload(&payload);

                                                        /* 重置累积器 */
                                                        odom_dx_world_acc = 0.0f;
                                                        odom_dy_world_acc = 0.0f;
                                                        odom_vel_ticks = 0;
                                                        add = 0;
                                                }
#else
                                                if(add >= 50){
                                                        float enc_left_total_m = -AS5048s[1].total_angle * AS5048_LEFT_METERS_PER_COUNT;
                                                        float enc_right_total_m = AS5048s[0].total_angle * AS5048_RIGHT_METERS_PER_COUNT;
                                                        float enc_avg_total_m = 0.5f * (enc_left_total_m + enc_right_total_m);
                                                        memset(mpu_buff, 0, sizeof(mpu_buff));
                                                        int mpu_len = snprintf(mpu_buff, sizeof(mpu_buff), "%f,%f,%f,%f,%f,%f,%f,%ld,%ld\r\n", mpu_data[0].REAL_X, mpu_data[0].REAL_Y, mpu_data[0].REAL_YAW, mpu_data[0].ROLL_ANGLE, enc_left_total_m, enc_right_total_m, enc_avg_total_m, (long)AS5048s[1].total_angle, (long)AS5048s[0].total_angle);
                                                        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)&mpu_buff, mpu_len);
                                                        add = 0;
                                                }
#endif

	                                        }else{

	                                          if(take_encoder_delta_ready() != 0U && YIS130_IsEulerFresh(HAL_GetTick()) != 0U){
	                                                  reset_encoder_odom_yaw_reference();
	                                          }
	                                          mpu_data[0].vel[0] = 0;
	                                    mpu_data[0].vel[1] = 0;
	                                                times ++ ;
	                                        }

	                 }
                }
}

void ClearUARTErrors(USART_TypeDef *USARTx) {
    // 清除奇偶校验错误
    if (USARTx->SR & USART_SR_PE) {
        (void)USARTx->DR;
    }
    // 清除帧错误
    if (USARTx->SR & USART_SR_FE) {
        (void)USARTx->DR;
    }
    // 清除噪声错误
    if (USARTx->SR & USART_SR_NE) {
        (void)USARTx->DR;
    }
    // 清除溢出错误
    if (USARTx->SR & USART_SR_ORE) {
        (void)USARTx->DR;
    }
    // 重新使能串口
    USARTx->CR1 |= USART_CR1_UE;
    // 重新使能错误中断
    USARTx->CR3 |= USART_CR3_EIE;
    // 重新使能接收 FIFO 阈值中断
    USARTx->CR3 |= USART_CR3_RXFTIE;
                // 重新打开 DMA 接收
                HAL_UART_Receive_DMA(&huart1,rcv_buf,RCV_BUF_SIZE);
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
