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
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <stdint.h>



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define MPU9250_ADDR (0x68 << 1)
#define AK8963_ADDR  (0x0C << 1)

#define FRAME_MARKER 699
#define RPI_CONTROL_TIMEOUT_MS 400


// Rejestry MPU9250
#define REG_PWR_MGMT_1   0x6B
#define REG_USER_CTRL    0x6A
#define REG_INT_PIN_CFG  0x37
#define REG_ACCEL_XOUT_H 0x3B

//inne
#define INTEGRATOR_LIMIT 30.0f

// Rejestry magnetometru AK8963
#define AK8963_REG_CNTL1 0x0A
#define AK8963_REG_HXL   0x03
#define TELEMETRY_BUF_LEN 152

volatile float sin_phase = 0.0f;

#define SIN_AMPL_DEG 8.0f
#define SIN_FREQ_HZ 0.1f


#define MOTOR1_TIM      (&htim3)
#define MOTOR1_CH       TIM_CHANNEL_4

#define MOTOR2_TIM      (&htim15)
#define MOTOR2_CH       TIM_CHANNEL_2

#define MOTOR3_TIM      (&htim3)
#define MOTOR3_CH       TIM_CHANNEL_1

#define MOTOR4_TIM      (&htim15)
#define MOTOR4_CH       TIM_CHANNEL_1

#define GYRO_RATE_LIMIT 300.0f // deg/s, na start

#define PWM_MIN           1150
#define PWM_MAX           1500

#define PWM_MIN_M2           1150
#define PWM_MAX_M2           1500

#define PWM_MIN_M1        1350
#define PWM_MAX_M1        1700
#define PWM_MIN_M4         930
#define PWM_MAX_M4        1280

#define MAX_PWM_STEP_ALL  15U

#define ANGLE_DT (0.005f)  // 200 Hz



#define LEFT_FRONT_MOTOR   1
#define RIGHT_FRONT_MOTOR  2
#define RIGHT_REAR_MOTOR   3
#define LEFT_REAR_MOTOR    4


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

typedef enum {
    ANGLE_SEQ_0,
    ANGLE_SEQ_1,
    ANGLE_SEQ_2,
    ANGLE_SEQ_3
} angle_seq_t;

volatile angle_seq_t angle_seq = ANGLE_SEQ_0;

volatile float desired_roll_deg  = 0.0f;
volatile float desired_pitch_deg = 0.0f;



volatile uint8_t imu_flag = 0;
volatile uint8_t imu_initialized = 0;
volatile float gyro_off_x = 0.0f, gyro_off_y = 0.0f, gyro_off_z = 0.0f;
volatile bool czy_zgoda_na_prace = false;

volatile bool czy_zgoda_na_prace_rpi5 = false;

volatile bool tilt_kill_active = false;

const float TILT_LIMIT_DEG = 40.0f;

volatile uint8_t spi_rx_ready = 0;    // flaga: nowy komplet danych gotowy do parsowania
volatile uint8_t spi_rx_buf[24];   // bufor odbiorczy DMA/IT - volatile bo modyfikowany w callback

float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
float beta = 0.05f; // Madgwick gain

volatile uint8_t uart_tx_busy = 0;
static char telemetry_buf[2][TELEMETRY_BUF_LEN];
static uint8_t telemetry_idx = 0;

const float THROTTLE_MIN = 0.0f;
const float THROTTLE_MAX = 1.0f;
/* PID and filters */
typedef struct {
    float kp;
    float ki;
    float kd;

    float integrator;
    float last_error;
    float last_measurement;

    float output_min;
    float output_max;
} PID_t;

static inline float pid_update_rate(PID_t *pid,
                                    float setpoint,
                                    float measurement,
                                    float dt)
{
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measurement;

    pid->integrator += error * dt;

    if (pid->integrator > INTEGRATOR_LIMIT)  pid->integrator = INTEGRATOR_LIMIT;
    if (pid->integrator < -INTEGRATOR_LIMIT) pid->integrator = -INTEGRATOR_LIMIT;

    float derivative = -(measurement - pid->last_measurement) / dt;
    pid->last_measurement = measurement;

    /* PID RAW */
    float out_p = pid->kp * error;
    float out_i = pid->ki * pid->integrator;
    float out_d = pid->kd * derivative;

    float out = out_p + out_i + out_d;

    if (out > pid->output_max) {
        out = pid->output_max;
        out_d = 0.0f;
    }
    if (out < pid->output_min) {
        out = pid->output_min;
        out_d = 0.0f;
    }

    return out;
}


PID_t pid_rate_roll = {
    .kp = 0.3f,
    .ki = 0.02f,
    .kd = 0.02f,
    .integrator = 0.0f,
    .last_error = 0.0f,
    .last_measurement = 0.0f,
    .output_min = -12.0f,
    .output_max =  12.0f
};

PID_t pid_rate_pitch = {
    .kp = 0.3f,
    .ki = 0.02f,
    .kd = 0.02f,
    .integrator = 0.0f,
    .last_error = 0.0f,
    .last_measurement = 0.0f,
    .output_min = -12.0f,
    .output_max =  12.0f
};

PID_t pid_rate_yaw = {
    .kp = 0.10f,
    .ki = 0.0f,
    .kd = 0.0f,
    .integrator = 0.0f,
    .last_error = 0.0f,
    .last_measurement = 0.0f,
    .output_min = -12.0f,
    .output_max =  12.0f
};


// Angle PIDs
PID_t pid_angle_roll = {
    .kp = 2.0f,
    .ki = 0.02f,
    .kd = 0.04f,
	.integrator = 0,
	    .last_error = 0,
    .output_min = -12.0f,
    .output_max = 12.0f
};

PID_t pid_angle_pitch = {
    .kp = 2.0f,
    .ki = 0.02f,
    .kd = 0.04f,
	.integrator = 0,
	    .last_error = 0,
    .output_min = -12.0f,
    .output_max = 12.0f
};




typedef struct
{   float a;
	float last;
	int init;
} LPF_t;

void lpf_init(LPF_t *f, float a)
	{ f->a = a;
	f->last = 0.0f;
	f->init = 0; }

float lpf_apply(LPF_t *f, float x) {
    if (!f->init)
    	{
    f->last = x;
    f->init = 1;
    return x;
    	}

    f->last = f->last * (1.0f - f->a) + x * f->a;
    return f->last;
}
LPF_t lpf_roll, lpf_pitch;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static inline void DWT_Init(void)
{
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk))
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t micros(void)
{
    return (uint32_t)(DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000UL));
}

void MPU9250_Init(void) {
    uint8_t cmd;

    // wybudzenie, wyczyść sleep
    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDR, REG_PWR_MGMT_1, 1, &cmd, 1, 100);
    HAL_Delay(10);

    // wyłącz kontrolę użytkownika (FIFO itp.)
    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDR, REG_USER_CTRL, 1, &cmd, 1, 100);

    // INT_PIN_CFG ustaw
    cmd = 0x02;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDR, REG_INT_PIN_CFG, 1, &cmd, 1, 100);

    HAL_Delay(10);

    // ustawienia zakresów:
    // GYRO_CONFIG (0x1B) -> FS_SEL = 01 -> ±500 dps -> 0x08
    cmd = 0x08;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDR, 0x1B, 1, &cmd, 1, 100);

    // ACCEL_CONFIG (0x1C) -> AFS_SEL = 00 -> ±2 g -> 0x00
    cmd = 0x00;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDR, 0x1C, 1, &cmd, 1, 100);

    // ACCEL_CONFIG2 (0x1D) -> DLPF ustawione na ~44Hz (wartość 0x03)
    cmd = 0x03;
    HAL_I2C_Mem_Write(&hi2c2, MPU9250_ADDR, 0x1D, 1, &cmd, 1, 100);

    HAL_Delay(10);

    // ustawienie AK8963 (mag) pozostawione bez zmian
    cmd = 0x16;
    HAL_I2C_Mem_Write(&hi2c2, AK8963_ADDR, AK8963_REG_CNTL1, 1, &cmd, 1, 100);
}


void Read_Sensors(float *ax, float *ay, float *az, float *gx, float *gy, float *gz,
                  float *mx, float *my, float *mz) {
    uint8_t buf[14];
    HAL_I2C_Mem_Read(&hi2c2, MPU9250_ADDR, REG_ACCEL_XOUT_H, 1, buf, 14, 100);
    int16_t ax_raw = (buf[0] << 8) | buf[1];
    int16_t ay_raw = (buf[2] << 8) | buf[3];
    int16_t az_raw = (buf[4] << 8) | buf[5];
    int16_t gx_raw = (buf[8]  << 8) | buf[9];
    int16_t gy_raw = (buf[10] << 8) | buf[11];
    int16_t gz_raw = (buf[12] << 8) | buf[13];



    const float ACCEL_SCALE = 16384.0f; // ±2 g
    const float GYRO_SCALE  = 65.536f;  // ±500 °/s

    *ax = ax_raw / ACCEL_SCALE;
    *ay = ay_raw / ACCEL_SCALE;
    *az = az_raw / ACCEL_SCALE;
    *gx = gx_raw / GYRO_SCALE;
    *gy = gy_raw / GYRO_SCALE;
    *gz = gz_raw / GYRO_SCALE;


    uint8_t mag[6];
    HAL_I2C_Mem_Read(&hi2c2, AK8963_ADDR, AK8963_REG_HXL, 1, mag, 6, 100);
    int16_t mx_raw = (mag[1] << 8) | mag[0];
    int16_t my_raw = (mag[3] << 8) | mag[2];
    int16_t mz_raw = (mag[5] << 8) | mag[4];
    *mx = mx_raw * 0.15f;
    *my = my_raw * 0.15f;
    *mz = mz_raw * 0.15f;
}


void MadgwickAHRSupdateIMU_dt(float gx, float gy, float gz,
                              float ax, float ay, float az,
                              float *q1, float *q2, float *q3, float *q0,
                              float beta, float dt)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot0, qDot1, qDot2, qDot3;

    float _2q0 = 2.0f * (*q0);
    float _2q1 = 2.0f * (*q1);
    float _2q2 = 2.0f * (*q2);
    float _2q3 = 2.0f * (*q3);
    float _4q0 = 4.0f * (*q0);
    float _4q1 = 4.0f * (*q1);
    float _4q2 = 4.0f * (*q2);

    float q0q0 = (*q0) * (*q0);
    float q1q1 = (*q1) * (*q1);
    float q2q2 = (*q2) * (*q2);
    float q3q3 = (*q3) * (*q3);

    recipNorm = sqrtf(ax * ax + ay * ay + az * az);
    if (recipNorm <= 0.0f) return;
    recipNorm = 1.0f / recipNorm;
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * (*q1) - _2q0 * ay - _4q1 + _4q1 * az;
    s2 = 4.0f * q0q0 * (*q2) + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * (*q3) - _2q1 * ax + 4.0f * q2q2 * (*q3) - _2q2 * ay;

    recipNorm = 1.0f / sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

    qDot0 = 0.5f * (-(*q1)*gx - (*q2)*gy - (*q3)*gz) - beta * s0;
    qDot1 = 0.5f * ((*q0)*gx + (*q2)*gz - (*q3)*gy) - beta * s1;
    qDot2 = 0.5f * ((*q0)*gy - (*q1)*gz + (*q3)*gx) - beta * s2;
    qDot3 = 0.5f * ((*q0)*gz + (*q1)*gy - (*q2)*gx) - beta * s3;

    *q0 += qDot0 * dt;
    *q1 += qDot1 * dt;
    *q2 += qDot2 * dt;
    *q3 += qDot3 * dt;

    recipNorm = 1.0f / sqrtf((*q0)*(*q0) + (*q1)*(*q1) + (*q2)*(*q2) + (*q3)*(*q3));
    *q0 *= recipNorm; *q1 *= recipNorm; *q2 *= recipNorm; *q3 *= recipNorm;
}

int send_telemetry_dma(const char *text, size_t len)
{
    if (len == 0 || len > TELEMETRY_BUF_LEN) return 1;
    if (uart_tx_busy) return 1;
    	telemetry_idx ^= 1;
    memcpy(telemetry_buf[telemetry_idx], text, len);
    uart_tx_busy = 1;
    if (HAL_UART_Transmit_DMA(&huart2, (uint8_t*)telemetry_buf[telemetry_idx], len) != HAL_OK) {
        uart_tx_busy = 0;
        return 1;
    }
    return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uart_tx_busy = 0;

}


static inline void mixer_quad_x(float throttle, float roll_cmd, float pitch_cmd, float yaw_cmd,
                                uint32_t *out1, uint32_t *out2, uint32_t *out3, uint32_t *out4)
{

    // M1: lewy przód
       float m1 = throttle + roll_cmd - pitch_cmd + yaw_cmd;

       // M2: prawy przód
       float m2 = throttle - roll_cmd - pitch_cmd - yaw_cmd;

       // M3: prawy tył
       float m3 = throttle - roll_cmd + pitch_cmd + yaw_cmd;

       // M4: lewy tył
       float m4 = throttle + roll_cmd + pitch_cmd - yaw_cmd;
    // clamp 0..1
    if (m1 < 0.0f) m1 = 0.0f; if (m1 > 1.0f) m1 = 1.0f;
    if (m2 < 0.0f) m2 = 0.0f; if (m2 > 1.0f) m2 = 1.0f;
    if (m3 < 0.0f) m3 = 0.0f; if (m3 > 1.0f) m3 = 1.0f;
    if (m4 < 0.0f) m4 = 0.0f; if (m4 > 1.0f) m4 = 1.0f;


    const int32_t pwm_min = PWM_MIN;
    const int32_t pwm_max = PWM_MAX;
    const int32_t pwm_min_m1 = PWM_MIN_M1;
    const int32_t pwm_max_m1 = PWM_MAX_M1;

    const int32_t pwm_min_m2 = PWM_MIN_M2;
     const int32_t pwm_max_m2 = PWM_MAX_M2;

    const int32_t pwm_min_m4 = PWM_MIN_M4;
    const int32_t pwm_max_m4 = PWM_MAX_M4;


    int32_t p1 = pwm_min_m1 + (int32_t)((pwm_max_m1 - pwm_min_m1) * m1 + 0.5f);
    int32_t p2 = pwm_min_m2 + (int32_t)((pwm_max_m2 - pwm_min_m2) * m2 + 0.5f);
    int32_t p3 = pwm_min + (int32_t)((pwm_max - pwm_min) * m3 + 0.5f);
    int32_t p4 = pwm_min_m4 + (int32_t)((pwm_max_m4 - pwm_min_m4) * m4 + 0.5f);


    if (p1 < pwm_min_m1) p1 = pwm_min_m1;
    if (p1 > pwm_max_m1) p1 = pwm_max_m1;
    if (p2 < pwm_min_m2) p2 = pwm_min_m2;
    if (p2 > pwm_max_m2) p2 = pwm_max_m2;
    if (p3 < pwm_min) p3 = pwm_min;
    if (p3 > pwm_max) p3 = pwm_max;
    if (p4 < pwm_min_m4) p4 = pwm_min_m4;
    if (p4 > pwm_max_m4) p4 = pwm_max_m4;

    *out1 = (uint32_t)p1;
    *out2 = (uint32_t)p2;
    *out3 = (uint32_t)p3;
    *out4 = (uint32_t)p4;
}

void calibrate_gyro(uint16_t samples)
{
    float ax, ay, az, gx, gy, gz, mx, my, mz;
    float sx = 0, sy = 0, sz = 0;
    for (uint16_t i = 0; i < samples; ++i)
    {
        Read_Sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
        sx += gx; sy += gy; sz += gz;
        HAL_Delay(2);
    }
    gyro_off_x = sx / samples;
    gyro_off_y = sy / samples;
    gyro_off_z = sz / samples;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        HAL_TIM_Base_Stop_IT(htim);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 800);
       // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 800);
        __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 800);
        czy_zgoda_na_prace = true;
    }
    if (htim->Instance == TIM17) {
        imu_flag = 1;
    }
    if (htim->Instance == TIM16)
    {
        // krok fazy = 2π * f * dt
        // TIM16 u Ciebie: Prescaler=7999, Period=49999
        // => ~1 Hz przerwanie (co 1s)
        // więc dt ≈ 1.0f

        float dt = 0.1f;

        sin_phase += 2.0f * M_PI * SIN_FREQ_HZ * dt;
        if (sin_phase > 2.0f * M_PI)
            sin_phase -= 2.0f * M_PI;

        // SINUS
        desired_roll_deg  = SIN_AMPL_DEG * sinf(sin_phase);
        desired_pitch_deg = SIN_AMPL_DEG * sinf(sin_phase); // przesunięcie 90°

        // opcjonalnie: zeruj integratory przy wolnym ruchu
        pid_angle_roll.integrator  = 0.0f;
        pid_angle_pitch.integrator = 0.0f;
    }


}
void motors_start_with_timer(void)
{ //funkcja resetujaca sterowniki silnikow
	czy_zgoda_na_prace = false;
    uint32_t w = 900;

   // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,1200 ); //lewa gora

    // Ustaw wypełnienie początkowe
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, w);

    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, w);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, w);

    // Start TIM6 w trybie przerwania
    HAL_TIM_Base_Start_IT(&htim6);

}

static inline void set_motor_pwm_by_index(uint8_t motor_idx, uint32_t pwm_value)
{
    TIM_HandleTypeDef *tim = NULL;
    uint32_t channel = 0;

    switch(motor_idx) {
        case 1: tim = MOTOR1_TIM; channel = MOTOR1_CH; break;
        case 2: tim = MOTOR2_TIM; channel = MOTOR2_CH; break;
        case 3: tim = MOTOR3_TIM; channel = MOTOR3_CH; break;
        case 4: tim = MOTOR4_TIM; channel = MOTOR4_CH; break;
        default: return;
    }
    __HAL_TIM_SET_COMPARE(tim, channel, pwm_value);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi == &hspi1) {
        // nowy komplet danych gotowy
        spi_rx_ready = 1;
        // restart odbioru DMA (ciągły tryb)
        HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)spi_rx_buf, sizeof(spi_rx_buf));
    }
}

static inline uint8_t parse_frame_and_apply(const uint8_t *buf,
                                            float *desired_roll_deg,
                                            float *desired_pitch_deg,
                                            float *desired_yaw_rate,
                                            float *throttle,
                                            bool  *start_flag)
{
    uint32_t m;
    memcpy(&m, buf + 0, 4);
    if (m == 699U) {
        float f_pitch, f_roll, f_yaw, f_throttle, f_flag;
        memcpy(&f_pitch,    buf + 4,  4); //pitch
        memcpy(&f_roll,     buf + 8,  4); // roll
        memcpy(&f_yaw,      buf + 12, 4); // yaw
        memcpy(&f_throttle, buf + 16, 4); //throttel
        memcpy(&f_flag,     buf + 20, 4); //flag

        if (f_roll >  4.0f) f_roll =  4.0f;
        if (f_roll < -4.0f) f_roll = -4.0f;
        if (f_pitch >  4.0f) f_pitch =  4.0f;
        if (f_pitch < -4.0f) f_pitch = -4.0f;
        if (f_yaw >  30.0f) f_yaw =  30.0f;
        if (f_yaw < -30.0f) f_yaw = -30.0f;
        if (f_throttle > THROTTLE_MAX) f_throttle = THROTTLE_MAX;
        if (f_throttle < THROTTLE_MIN) f_throttle = THROTTLE_MIN;

        *desired_roll_deg  = f_roll;
        *desired_pitch_deg = f_pitch;
        *desired_yaw_rate  = f_yaw;
        *throttle          = f_throttle;
        *start_flag        = (f_flag >= 0.5f) ? true : false;

        return 1;
    }

    memcpy(&m, buf + 4, 4);
    if (m == 699U) {
        float f_pitch, f_roll, f_yaw, f_throttle, f_flag;
        memcpy(&f_pitch,    buf + 0,  4);
        memcpy(&f_roll,     buf + 8,  4);
        memcpy(&f_yaw,      buf + 12, 4);
        memcpy(&f_throttle, buf + 16, 4);
        memcpy(&f_flag,     buf + 20, 4);

        if (f_roll >  4.0f) f_roll =  4.0f;
        if (f_roll < -4.0f) f_roll = -4.0f;
        if (f_pitch >  4.0f) f_pitch =  4.0f;
        if (f_pitch < -4.0f) f_pitch = -4.0f;
        if (f_yaw >  30.0f) f_yaw =  30.0f;
        if (f_yaw < -30.0f) f_yaw = -30.0f;
        if (f_throttle > THROTTLE_MAX) f_throttle = THROTTLE_MAX;
        if (f_throttle < THROTTLE_MIN) f_throttle = THROTTLE_MIN;

        *desired_roll_deg  = f_roll;
        *desired_pitch_deg = f_pitch;
        *desired_yaw_rate  = f_yaw;
        *throttle          = f_throttle;
        *start_flag        = (f_flag >= 0.5f) ? true : false;

        return 1;
    }

    return 0;
}

static inline float constrain(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}


static uint32_t prev_pwm_1 = PWM_MIN_M1;
static uint32_t prev_pwm_2 = PWM_MIN_M2;
static uint32_t prev_pwm_3 = PWM_MIN;
static uint32_t prev_pwm_4 = PWM_MIN_M4;



static inline void apply_safe_pwm_all(uint8_t motor_idx, uint32_t target_pwm)
{


    uint32_t minv = PWM_MIN, maxv = PWM_MAX;
    uint32_t *prev = NULL;

    if (motor_idx == 1) { minv = PWM_MIN_M1; maxv = PWM_MAX_M1; prev = &prev_pwm_1; }
    else if (motor_idx == 2) { minv = PWM_MIN_M2; maxv = PWM_MAX_M2; prev = &prev_pwm_2; }
    else if (motor_idx == 3) { minv = PWM_MIN; maxv = PWM_MAX; prev = &prev_pwm_3; }
    else if (motor_idx == 4) { minv = PWM_MIN_M4; maxv = PWM_MAX_M4; prev = &prev_pwm_4; }
    else return;

    // clamp do zakresu
    if (target_pwm < minv) target_pwm = minv;
    if (target_pwm > maxv) target_pwm = maxv;

    // jeśli prev==0 to zainicjuj
    if (*prev == 0) *prev = target_pwm;

    // slew limiter
    if (target_pwm > *prev) {
        uint32_t diff = target_pwm - *prev;
        if (diff > MAX_PWM_STEP_ALL) target_pwm = *prev + MAX_PWM_STEP_ALL;
    } else if (*prev > target_pwm) {
        uint32_t diff = *prev - target_pwm;
        if (diff > MAX_PWM_STEP_ALL) target_pwm = *prev - MAX_PWM_STEP_ALL;
    }

    *prev = target_pwm;

    set_motor_pwm_by_index(motor_idx, target_pwm);
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
  MX_TIM3_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM15_Init();
  MX_TIM6_Init();
  MX_I2C2_Init();
  MX_TIM7_Init();
  MX_TIM17_Init();
  MX_TIM16_Init();
  /* USER CODE BEGIN 2 */

 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
 HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


 HAL_SPI_Receive_DMA(&hspi1, (uint8_t*)spi_rx_buf, sizeof(spi_rx_buf));

 DWT_Init();
 HAL_Delay(50);
 MPU9250_Init();
 HAL_Delay(50);
 calibrate_gyro(1000);
 HAL_Delay(1500);
 imu_initialized = 1;
 HAL_Delay(1000);


 pid_rate_roll.integrator  = 0.0f; pid_rate_roll.last_error  = 0.0f;
 pid_rate_pitch.integrator = 0.0f; pid_rate_pitch.last_error = 0.0f;
 pid_rate_yaw.integrator   = 0.0f; pid_rate_yaw.last_error   = 0.0f;
 pid_angle_roll.integrator = 0.0f; pid_angle_roll.last_error = 0.0f;
 pid_angle_pitch.integrator= 0.0f; pid_angle_pitch.last_error= 0.0f;


 static LPF_t lpf_gx, lpf_gy;

 lpf_init(&lpf_gx, 0.3f);
 lpf_init(&lpf_gy, 0.3f);

 HAL_TIM_Base_Start_IT(&htim17);

 uint32_t last_us = micros();
 uint32_t outer_counter = 0;
 const uint32_t outer_ratio = 4; // angle loop 200 Hz


// zadane wartosci roll i pitch


// potrzebne do kalibracji zyro
 static float roll_zero = 0.0f;
 static float pitch_zero = 0.0f;
 static uint8_t zero_offsets_captured = 0;

 float roll_corrected = 0.0f;
 float pitch_corrected = 0.0f;



 float desired_yaw_rate  = 0.0f;

 float throttle = 0.25f;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


 uint8_t local_buf[24];
 static uint32_t last_frame_ms = 0;   /* czas ostatniej poprawnej ramki */

 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,1200 ); //lewa gora
 __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 940); // praswy przod
	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 920); //prawy tyl
	__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 900 ); //lewy tyl

HAL_Delay(5000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,1300 ); //lewa gora
  __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 1100); // praswy przod
	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); //prawy tyl
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 950 ); //lewy tyl

HAL_Delay(2000);
__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,1000 ); //lewa gora
__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 920); // praswy przod
	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 900);
		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 900 ); //lewy tyl

HAL_Delay(5000);
float roll_rate_out  = 0.0f;
float pitch_rate_out = 0.0f;
float yaw_rate_out   = 0.0f;
static uint8_t angle_div = 0;


HAL_TIM_Base_Start_IT(&htim16);

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//if(czy_zgoda_na_prace)
//{
//    uint32_t w = 920;  // lub 0, zależnie jak Twoje ESC reagują
//	  	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100);
//	  		        // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1300);  //lewo gora
//	  		    //     __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 950); //lewy tyl
//	  		         __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 1100); // praswy przod
//}

//
//			  			     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, w); //prawy tyl i kreci sie dobrze
//
//			  					    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, w ); //lewy gora i kerci sie dobrez
//
//			  					     __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, w ); //lewy tyl i kreci sie dobrze
//
//
//			  					     __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 1000);  // prawy przdo i kreci sie dobrez ale od 1000
////			  					     HAL_Delay(1500);
//
//	  if (spi_rx_ready) {
//	          spi_rx_ready = 0;
//	          memcpy(local_buf, (const void*)spi_rx_buf, sizeof(local_buf));
//
//	         uint8_t ok = parse_frame_and_apply(local_buf,
//	                                    &desired_roll_deg,
//	                                    &desired_pitch_deg,
//	                                    &desired_yaw_rate,
//	                                    &throttle,
//	                                    &czy_zgoda_na_prace_rpi5) ;
//	         if (ok == 1) {
//	             last_frame_ms = HAL_GetTick();  // zapamiętaj czas ostatniej poprawnej ramki
//	         }
//
//	      }
//
//	  uint32_t now_ms = HAL_GetTick();
//
//	  if (last_frame_ms > 0) {  // jeśli kiedykolwiek przyszła ramka
//	      if ((now_ms - last_frame_ms) > RPI_CONTROL_TIMEOUT_MS) {
//
//
//	          uint32_t w = 300;  // bezpieczny sygnał
//	          czy_zgoda_na_prace_rpi5 = false;  // dodatkowe zabezpieczenie
//
//	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, w);
//	          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, w);
//	          __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, w);
//	          __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, w);
//
//	          throttle = 0.0f;
//	      }
//	  }
//
//	  if (!czy_zgoda_na_prace_rpi5) {
//	      uint32_t w = 200;
//	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, w);
//	      __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, w);
//	      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, w);
//	      __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, w);
//
//	      throttle = 0.0f;  }
// && czy_zgoda_na_prace_rpi5
//(czy_zgoda_na_prace &&
	  czy_zgoda_na_prace_rpi5 = true;
	  if(  imu_flag && imu_initialized) {
	         imu_flag = 0;


	         uint32_t now_us = micros();
	         float dt = (now_us - last_us) * 1e-6f;
	         if (dt <= 0.0f || dt > 0.1f) dt = 0.001f;
	         last_us = now_us;
	         float ax, ay, az, gx, gy, gz, mx, my, mz;
	         Read_Sensors(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);


				 gx -= gyro_off_x; gy -= gyro_off_y; gz -= gyro_off_z;
	         const float deg2rad = 3.14159265358979323846f / 180.0f;
	         float gx_rad = gx * deg2rad, gy_rad = gy * deg2rad, gz_rad = gz * deg2rad;

	         MadgwickAHRSupdateIMU_dt(gx_rad, gy_rad, gz_rad, ax, ay, az, &q1, &q2, &q3, &q0, beta, dt);

	         float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)) * (180.0f / M_PI);
	         float pitch = asinf(-2.0f*(q1*q3 - q0*q2)) * (180.0f / M_PI);
	         float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)) * (180.0f / M_PI);

	         // IMU obrócone o +90° wokół Z (bokiem)
	         float roll_f  =  pitch;
	         float pitch_f = -roll;

	         float gx_body =  gy;
	         float gy_body = -gx;
	         float gz_body =  gz;

	         if (gx_body >  GYRO_RATE_LIMIT) gx_body =  GYRO_RATE_LIMIT;
	         if (gx_body < -GYRO_RATE_LIMIT) gx_body = -GYRO_RATE_LIMIT;
	         if (gy_body >  GYRO_RATE_LIMIT) gy_body =  GYRO_RATE_LIMIT;
	         if (gy_body < -GYRO_RATE_LIMIT) gy_body = -GYRO_RATE_LIMIT;

	         float gx_f = lpf_apply(&lpf_gx, gx_body);
	         float gy_f = lpf_apply(&lpf_gy, gy_body);

	         #define ZERO_SAMPLES 200
	         static int zero_sample_cnt = 0;
	         static float zero_roll_acc = 0.0f;
	         static float zero_pitch_acc = 0.0f;

	         if (!zero_offsets_captured) {
	             if (czy_zgoda_na_prace_rpi5) {
	                 zero_roll_acc  += roll_f;
	                 zero_pitch_acc += pitch_f;
	                 zero_sample_cnt++;

	                 if (zero_sample_cnt >= ZERO_SAMPLES) {
	                     roll_zero  = zero_roll_acc  / (float)zero_sample_cnt;
	                     pitch_zero = zero_pitch_acc / (float)zero_sample_cnt;
	                     zero_offsets_captured = 1;

	                     char zmsg[80];
	                     int l = snprintf(zmsg, sizeof(zmsg),
	                         "Zero captured: roll_zero=%.3f pitch_zero=%.3f samples=%d\r\n",
	                         roll_zero, pitch_zero, zero_sample_cnt);
	                     if (l>0 && !uart_tx_busy) HAL_UART_Transmit(&huart2, (uint8_t*)zmsg, (uint16_t)l, 50);
	                     zero_roll_acc = 0.0f;
	                     zero_pitch_acc = 0.0f;
	                     zero_sample_cnt = 0;
	                 }
	             } else {
	                 zero_roll_acc = 0.0f;
	                 zero_pitch_acc = 0.0f;
	                 zero_sample_cnt = 0;
	             }
	         }


	         roll_corrected  = roll_f  - roll_zero;
	         pitch_corrected = pitch_f - pitch_zero;


	         if ( (fabsf(roll_corrected) > 35.0f || fabsf(pitch_corrected) > 35.0f))
	         {
	             tilt_kill_active = true;

	             czy_zgoda_na_prace_rpi5 = false;
	             throttle = 0.0f;

	             __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,100 ); //lewa gora
	             __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_2, 100); // praswy przod
	             	   __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 100);
	             		__HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, 100 );

	             pid_angle_roll.integrator  = 0.0f; pid_angle_roll.last_error  = 0.0f;
	             pid_angle_pitch.integrator = 0.0f; pid_angle_pitch.last_error = 0.0f;
	             pid_rate_roll.integrator   = 0.0f; pid_rate_roll.last_error   = 0.0f;
	             pid_rate_pitch.integrator  = 0.0f; pid_rate_pitch.last_error  = 0.0f;
	             pid_rate_yaw.integrator    = 0.0f; pid_rate_yaw.last_error    = 0.0f;

	             if (!uart_tx_busy) {
	                 char tmsg[80];
	                 int l = snprintf(tmsg, sizeof(tmsg),
	                     "TILT KILL ACTIVATED roll=%.2f pitch=%.2f\r\n",
	                     roll_corrected, pitch_corrected);
	                 send_telemetry_dma(tmsg, (size_t)l);
	             }
	             	 	 break;
	         }

	         static float roll_rate_setpoint = 0.0f;
	         static float pitch_rate_setpoint = 0.0f;
	         static float angle_dt_acc = 0.0f;
	         angle_dt_acc += dt;





	         //  ANGLE LOOP

	         if (++angle_div >= 5) {   // 1kHz / 5 = 200 Hz
	             angle_div = 0;

	             float roll_err  = desired_roll_deg  - roll_corrected;
	             float pitch_err = desired_pitch_deg - pitch_corrected;

	             roll_rate_setpoint  = pid_angle_roll.kp  * roll_err;
	             pitch_rate_setpoint = pid_angle_pitch.kp * pitch_err;

	             roll_rate_setpoint  = constrain(roll_rate_setpoint,  -10.0f, 10.0f);
	             pitch_rate_setpoint = constrain(pitch_rate_setpoint, -10.0f, 10.0f);
	         }





	         /* RATE PID */
	         roll_rate_out  = pid_update_rate(&pid_rate_roll,
	                                          roll_rate_setpoint,
	                                          gx_f,
	                                          dt);

	         pitch_rate_out = pid_update_rate(&pid_rate_pitch,
	                                          pitch_rate_setpoint,
	                                          gy_f,
	                                          dt);

	         yaw_rate_out   = pid_update_rate(&pid_rate_yaw,
	                                          desired_yaw_rate,
	                                          gz_body,
	                                          dt);



	         float roll_cmd_norm  = roll_rate_out / 12.0f;
	         float pitch_cmd_norm = pitch_rate_out / 12.0f;
	         float yaw_cmd_norm   = yaw_rate_out / 12.0f;


	         if (roll_cmd_norm >  1.0f) roll_cmd_norm =  1.0f;
	         if (roll_cmd_norm < -1.0f) roll_cmd_norm = -1.0f;
	         if (pitch_cmd_norm >  1.0f) pitch_cmd_norm =  1.0f;
	         if (pitch_cmd_norm < -1.0f) pitch_cmd_norm = -1.0f;
	         if (yaw_cmd_norm >  1.0f) yaw_cmd_norm =  1.0f;
	         if (yaw_cmd_norm < -1.0f) yaw_cmd_norm = -1.0f;

	         uint32_t m1=0, m2=0, m3=0, m4=0;

	         yaw_cmd_norm = 0.0f; // tylko do testu

	        // float max_cmd = throttle * 1.2f;   // skala momentu
	         float max_cmd = 1.0f;

	         if (roll_cmd_norm >  max_cmd) roll_cmd_norm =  max_cmd;
	         if (roll_cmd_norm < -max_cmd) roll_cmd_norm = -max_cmd;
	         if (pitch_cmd_norm >  max_cmd) pitch_cmd_norm =  max_cmd;
	         if (pitch_cmd_norm < -max_cmd) pitch_cmd_norm = -max_cmd;

	         mixer_quad_x(throttle, roll_cmd_norm, pitch_cmd_norm, yaw_cmd_norm, &m1, &m2, &m3, &m4);


	         if (m1 < PWM_MIN_M1) { m1 = PWM_MIN_M1; }
	         if (m1 > PWM_MAX_M1) { m1 = PWM_MAX_M1; }

	         if (m2 < PWM_MIN_M2) { m2 = PWM_MIN_M2; }
	         if (m2 > PWM_MAX_M2) { m2 = PWM_MAX_M2; }


	         if (m3 < PWM_MIN) { m3 = PWM_MIN; }
	         if (m3 > PWM_MAX) { m3 = PWM_MAX; }
	         if (m4 < PWM_MIN_M4) { m4 = PWM_MIN_M4; }
	         if (m4 > PWM_MAX_M4) { m4 = PWM_MAX_M4; }
	         apply_safe_pwm_all(1, m1);
	         apply_safe_pwm_all(2, m2);
	         apply_safe_pwm_all(3, m3);
	         apply_safe_pwm_all(4, m4);
	         static uint32_t tele_last = 0;
	         uint32_t now_ms = HAL_GetTick();
//
	         float m1_pct = 100.0f * ((float)m1 - PWM_MIN_M1) / (PWM_MAX_M1 - PWM_MIN_M1);
	         float m2_pct = 100.0f * ((float)m2 - PWM_MIN_M2) / (PWM_MAX_M2 - PWM_MIN_M2);
	         float m3_pct = 100.0f * ((float)m3 - PWM_MIN) / (PWM_MAX - PWM_MIN);
	         float m4_pct = 100.0f * ((float)m4 - PWM_MIN_M4) / (PWM_MAX_M4 - PWM_MIN_M4);

	         if ((now_ms - tele_last) >= 70) {
	             tele_last = now_ms;
	             char tbuf[150];

	             int len = snprintf(tbuf, sizeof(tbuf),
	                 "ANG R:%.2f P:%.2f "
	                 "sp R:%.1f P:%.1f "
	                 "me R:%.1f P:%.1f Y:%.1f "
	                 "out R:%.1f P:%.1f "
	                 "M %.2f %.2f %.2f %.2f "
	                 "DES R:%.1f P:%.1f\r\n",

	                 roll_corrected,
	                 pitch_corrected,

	                 roll_rate_setpoint,
	                 pitch_rate_setpoint,

	                 gx_f, gy_f, gz_body,

	                 roll_rate_out,
	                 pitch_rate_out,

	                 m1_pct, m2_pct, m3_pct, m4_pct,

	                 desired_roll_deg,
	                 desired_pitch_deg
	             );



	             if (len > 0 && !uart_tx_busy)
	                 send_telemetry_dma(tbuf, (size_t)len);
	         }


	     }




	  //ax, ay, az,
	  //Ax:%.2f Ay:%.2f Az:%.2f
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00B10E24;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /** I2C Fast mode Plus enable
  */
  HAL_I2CEx_EnableFastModePlus(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
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
  htim3.Init.Prescaler = 79;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 29999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 7999;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 19;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 79;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 19999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */
  HAL_TIM_MspPostInit(&htim15);

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7999;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 999;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 79;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_LINKDMA(&huart2, hdmatx, hdma_usart2_tx);

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
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
