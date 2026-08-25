/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Multi-motor UART remote control (3 motors)
  *
  * Controls up to 3 CubeMars motors (AK60 or AK70/80) via a single CAN bus.
  * Parameters received over UART2 at 115200 baud from Python GUI.
  *
  * Protocol: <motor_num><cmd><value>\n
  *   motor_num = 1,2,3
  *   P=position, V=velocity, K=Kp, D=Kd, T=torque, E=enable, I=CAN_ID, A=type
  *
  * AK60 uses extended CAN ID:  ExtId = CAN_ID | (8 << 8)
  *   Frame: [KP_hi][KP_lo|KD_hi][KD_lo][Pos_hi][Pos_lo][Vel_hi][Vel_lo|T_hi][T_lo]
  *
  * AK70/80 uses standard CAN ID:  StdId = CAN_ID
  *   Frame: [Pos_hi][Pos_lo][Vel_hi][Vel_lo|KP_hi][KP_lo][KD_hi][KD_lo|T_hi][T_lo]
  *
  * Hardware: STM32F446RE Nucleo + Waveshare CAN shield
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "uart_cmd.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

// AK60 parameter limits
#define AK60_P_MIN    -12.56f
#define AK60_P_MAX     12.56f
#define AK60_V_MIN    -60.0f
#define AK60_V_MAX     60.0f
#define AK60_T_MIN    -12.0f
#define AK60_T_MAX     12.0f

// AK70/80 parameter limits
#define AK70_P_MIN    -12.5f
#define AK70_P_MAX     12.5f
#define AK70_V_MIN    -30.0f
#define AK70_V_MAX     30.0f
#define AK70_T_MIN    -18.0f
#define AK70_T_MAX     18.0f

// Shared limits
#define KP_MIN    0.0f
#define KP_MAX    500.0f
#define KD_MIN    0.0f
#define KD_MAX    5.0f

#define CAN_PACKET_MIT 8

// CAN handles
CAN_TxHeaderTypeDef txHeader;
CAN_RxHeaderTypeDef rxHeader;
uint8_t  txData[8];
uint8_t  rxData[8];
uint32_t txMailbox;

uint8_t uart_rx_byte;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
static float        uint_to_float(int x_int, float x_min, float x_max, int bits);
static void         send_can_frame(void);
static void         send_ak60_cmd(MotorState *m);
static void         send_ak70_cmd(MotorState *m);
static void         send_ak70_enable(MotorState *m);
static void         unpack_ak60_reply(uint8_t motor_idx);
static void         unpack_ak70_reply(uint8_t motor_idx);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static unsigned int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
    float span = x_max - x_min;
    if (x < x_min) x = x_min;
    else if (x > x_max) x = x_max;
    return (unsigned int)((x - x_min) / span * (float)((1u << bits) - 1));
}

static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + x_min;
}

static void send_can_frame(void)
{
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox);
}

// AK60: extended CAN ID, KP/KD/Pos/Vel/Torque byte order
static void send_ak60_cmd(MotorState *m)
{
    uint16_t kp_int = float_to_uint(m->kp,  KP_MIN, KP_MAX, 12);
    uint16_t kd_int = float_to_uint(m->kd,  KD_MIN, KD_MAX, 12);
    uint16_t p_int  = float_to_uint(m->pos, AK60_P_MIN, AK60_P_MAX, 16);
    uint16_t v_int  = float_to_uint(m->vel, AK60_V_MIN, AK60_V_MAX, 12);
    uint16_t t_int  = float_to_uint(m->tff, AK60_T_MIN, AK60_T_MAX, 12);

    txData[0] = (kp_int >> 4) & 0xFF;
    txData[1] = ((kp_int & 0xF) << 4) | ((kd_int >> 8) & 0xF);
    txData[2] = kd_int & 0xFF;
    txData[3] = (p_int >> 8) & 0xFF;
    txData[4] = p_int & 0xFF;
    txData[5] = (v_int >> 4) & 0xFF;
    txData[6] = ((v_int & 0xF) << 4) | ((t_int >> 8) & 0xF);
    txData[7] = t_int & 0xFF;

    txHeader.StdId              = 0;
    txHeader.ExtId              = (uint32_t)m->can_id | ((uint32_t)CAN_PACKET_MIT << 8);
    txHeader.IDE                = CAN_ID_EXT;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK70/80: standard CAN ID, Pos/Vel/KP/KD/Torque byte order
static void send_ak70_cmd(MotorState *m)
{
    unsigned int p_int  = float_to_uint(m->pos, AK70_P_MIN, AK70_P_MAX, 16);
    unsigned int v_int  = float_to_uint(m->vel, AK70_V_MIN, AK70_V_MAX, 12);
    unsigned int kp_int = float_to_uint(m->kp,  KP_MIN, KP_MAX, 12);
    unsigned int kd_int = float_to_uint(m->kd,  KD_MIN, KD_MAX, 12);
    unsigned int t_int  = float_to_uint(m->tff, AK70_T_MIN, AK70_T_MAX, 12);

    txData[0] = p_int >> 8;
    txData[1] = p_int & 0xFF;
    txData[2] = v_int >> 4;
    txData[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    txData[4] = kp_int & 0xFF;
    txData[5] = kd_int >> 4;
    txData[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    txData[7] = t_int & 0xFF;

    txHeader.StdId              = m->can_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK70/80 enable: 0xFF x7 + 0xFC
static void send_ak70_enable(MotorState *m)
{
    txData[0] = 0xFF;
    txData[1] = 0xFF;
    txData[2] = 0xFF;
    txData[3] = 0xFF;
    txData[4] = 0xFF;
    txData[5] = 0xFF;
    txData[6] = 0xFF;
    txData[7] = 0xFC;

    txHeader.StdId              = m->can_id;
    txHeader.ExtId              = 0;
    txHeader.IDE                = CAN_ID_STD;
    txHeader.RTR                = CAN_RTR_DATA;
    txHeader.DLC                = 8;
    txHeader.TransmitGlobalTime = DISABLE;
    send_can_frame();
}

// AK60 feedback: servo-mode format
static void unpack_ak60_reply(uint8_t motor_idx)
{
    int16_t pos_int = (int16_t)((rxData[0] << 8) | rxData[1]);
    int16_t spd_int = (int16_t)((rxData[2] << 8) | rxData[3]);
    int16_t cur_int = (int16_t)((rxData[4] << 8) | rxData[5]);

    float pos = (float)pos_int * 0.1f;
    float spd = (float)spd_int * 10.0f;
    float cur = (float)cur_int * 0.01f;
    int8_t temp = (int8_t)rxData[6];
    int8_t err  = (int8_t)rxData[7];

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "[M%d] pos=%.1f deg  spd=%.0f eRPM  I=%.2f A  T=%d  err=%d\r\n",
        motor_idx + 1, (double)pos, (double)spd, (double)cur, temp, err);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}

// AK70/80 feedback: MIT mode format
static void unpack_ak70_reply(uint8_t motor_idx)
{
    int p_int = (rxData[1] << 8) | rxData[2];
    int v_int = (rxData[3] << 4) | (rxData[4] >> 4);
    int i_int = ((rxData[4] & 0xF) << 8) | rxData[5];

    float pos = uint_to_float(p_int, AK70_P_MIN, AK70_P_MAX, 16);
    float vel = uint_to_float(v_int, AK70_V_MIN, AK70_V_MAX, 12);
    float tau = uint_to_float(i_int, -AK70_T_MAX, AK70_T_MAX, 12);
    int8_t temp = (int8_t)(rxData[6] - 40);
    int8_t err  = (int8_t)rxData[7];

    char buf[96];
    int len = snprintf(buf, sizeof(buf),
        "[M%d] pos=%.3f rad  vel=%.2f  tau=%.2f  T=%d  err=%d\r\n",
        motor_idx + 1, (double)pos, (double)vel, (double)tau, temp, err);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, len, 100);
}

// Identify which motor sent feedback based on CAN ID
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
        return;

    // Try multiple ways to identify source motor:
    //  - Extended frame (AK60): motor ID in lower byte of ExtId
    //  - Standard frame (AK70): motor ID in rxData[0]
    uint8_t rx_id;
    if (rxHeader.IDE == CAN_ID_EXT)
        rx_id = (uint8_t)(rxHeader.ExtId & 0xFF);
    else
        rx_id = rxData[0];

    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        MotorState *m = uart_cmd_motor(i);
        if (m->can_id == rx_id) {
            if (m->type == MOTOR_TYPE_AK60)
                unpack_ak60_reply(i);
            else
                unpack_ak70_reply(i);
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            return;
        }
    }

    // No match — print debug line so we can see what came in
    char dbg[64];
    int len = snprintf(dbg, sizeof(dbg), "[M0] unknown CAN id=%d ide=%lu\r\n",
        (int)rx_id, (unsigned long)rxHeader.IDE);
    HAL_UART_Transmit(&huart2, (uint8_t*)dbg, len, 50);
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2) {
        uart_cmd_feed(uart_rx_byte);
        HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
    }
}

/* USER CODE END 0 */

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_CAN1_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */

    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

    uart_cmd_init();
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

    HAL_Delay(100);

    /* USER CODE END 2 */

    /* USER CODE BEGIN WHILE */
    while (1)
    {
        for (uint8_t i = 0; i < NUM_MOTORS; i++)
        {
            MotorState *m = uart_cmd_motor(i);

            // Handle pending enable (AK70/80)
            if (m->enable_pending) {
                for (int j = 0; j < 10; j++) {
                    send_ak70_enable(m);
                    HAL_Delay(5);
                }
                m->enable_pending = 0;
            }

            // Send position command
            if (m->type == MOTOR_TYPE_AK60)
                send_ak60_cmd(m);
            else
                send_ak70_cmd(m);

            HAL_Delay(2);
        }

        HAL_Delay(4);  // ~100 Hz total loop
    }
    /* USER CODE END WHILE */
}

/* -------------------------------------------------------------------------- */
/*  Peripheral init                                                           */
/* -------------------------------------------------------------------------- */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 336;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 2;
    RCC_OscInitStruct.PLL.PLLR            = 2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) Error_Handler();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) Error_Handler();
}

static void MX_CAN1_Init(void)
{
    hcan1.Instance              = CAN1;
    hcan1.Init.Prescaler        = 3;
    hcan1.Init.Mode             = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth    = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1         = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2         = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode    = DISABLE;
    hcan1.Init.AutoBusOff           = DISABLE;
    hcan1.Init.AutoWakeUp           = DISABLE;
    hcan1.Init.AutoRetransmission   = DISABLE;
    hcan1.Init.ReceiveFifoLocked    = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK) Error_Handler();

    CAN_FilterTypeDef filter;
    filter.FilterActivation     = ENABLE;
    filter.FilterBank           = 0;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterIdHigh         = 0x0000;
    filter.FilterIdLow          = 0x0000;
    filter.FilterMaskIdHigh     = 0x0000;
    filter.FilterMaskIdLow      = 0x0000;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    HAL_CAN_ConfigFilter(&hcan1, &filter);
}

static void MX_USART2_UART_Init(void)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = 115200;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

void Error_Handler(void)
{
    __disable_irq();
    while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {}
#endif
