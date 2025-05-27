/** ===============================================================
 *                  P U B L I C  I N C L U D E S                    
 * ================================================================ */
 /*< Dependencies >*/
 #include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <ernno.h>
#include <math.h>

 /*< Implemented >*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/** ===============================================================
 *          P U B L I C  D E F I N E S  &  M A C R O S              
 * ================================================================ */

 /** =============================================================
 *  @def        MPU6050_ADDR
 *  @brief      I2C 7-bit address of the MPU6050 device, shifted for HAL.
 *
 *  @details    Base address (0x68) shifted left by 1 to accommodate
 *              the R/W bit as required by HAL I2C functions.
 * ============================================================= */
#define MPU6050_ADDR            (uint32_t)(0x68 << 1U)

/** =============================================================
 *  @def        MPU6050_REG_PWR_MGMT_1
 *  @brief      Register address for power management control.
 *
 *  @details    Writing to this register clears the sleep bit
 *              and configures the clock source.
 * ============================================================= */
#define MPU6050_REG_PWR_MGMT_1  (uint16_t)(0x6B)

/** =============================================================
 *  @def        MPU6050_REG_WHO_AM_I
 *  @brief      Register address for device identification.
 *
 *  @details    Reading this register returns the fixed device ID
 *              (should be 0x68 for MPU6050).
 * ============================================================= */
#define MPU6050_REG_WHO_AM_I    (uint16_t)(0x75)

/** =============================================================
 *  @def        ACCEL_XOUT_H
 *  @brief      Starting register address for accelerometer data.
 *
 *  @details    Reads six consecutive bytes: XH, XL, YH, YL, ZH, ZL.
 * ============================================================= */
#define ACCEL_XOUT_H            (uint16_t)(0x3B)

/** =============================================================
 *  @def        ACCEL_THRESHOLD_MG
 *  @brief      Minimum delta in milli-g to consider movement.
 *
 *  @details    Threshold used to filter out noise in accelerometer
 *              readings before reporting movement.
 * ============================================================= */
#define ACCEL_THRESHOLD_MG      (uint8_t)(200U)

/** =============================================================
 *  @def        THRESHOLD_MG
 *  @brief      General-purpose movement threshold in mg.
 *
 *  @details    Used to decide when to format and transmit data
 *              over UART based on magnitude of axis change.
 * ============================================================= */
#define THRESHOLD_MG            (uint8_t)(200U)

/** =============================================================
 *  @def        LSB_PER_G
 *  @brief      Number of raw LSB units per 1g.
 *
 *  @details    For a ±2g full-scale range, MPU6050 outputs 16,384 LSB/g.
 * ============================================================= */
#define LSB_PER_G               (uint16_t)(16384U)

/** =============================================================
 *  @def        MG_PER_G
 *  @brief      Number of milli-g per 1g.
 * ============================================================= */
#define MG_PER_G                (uint16_t)(1000U)

/** =============================================================
 *  @def        GRAVITY_MS2
 *  @brief      Standard acceleration due to gravity (m/s²).
 * ============================================================= */
#define GRAVITY_MS2             (float)(9.80665f)

/** =============================================================
 *  @def        SAMPLE_PERIOD_S
 *  @brief      Sensor sampling period in seconds.
 *
 *  @details    Used for numerical integration of acceleration.
 * ============================================================= */
#define SAMPLE_PERIOD_S         (float)(0.01f)

/** =============================================================
 *  @def        DX2MM_FACTOR
 *  @brief      Scaling factor for delta-mg to millimeters conversion.
 *
 *  @details    Empirical factor to approximate displacement in mm.
 * ============================================================= */
#define DX2MM_FACTOR            (uint8_t)(49U)

/** =============================================================
 *  @def        DX2MM_ROUNDING
 *  @brief      Rounding offset for DX2MM conversion.
 * ============================================================= */
#define DX2MM_ROUNDING          (uint16_t)(500U)

/** =============================================================
 *  @def        MM2CM_DIVISOR
 *  @brief      Divisor to convert millimeters to centimeters.
 * ============================================================= */
#define MM2CM_DIVISOR           (uint8_t)(10U)

/** =============================================================
 *  @def        MAX_MSG_LEN
 *  @brief      Maximum length of UART message buffer.
 * ============================================================= */
#define MAX_MSG_LEN             (uint8_t)(2000u)

/** =============================================================
 *  @def        MAX_ACCEL_BUF
 *  @brief      Number of bytes to read raw accelerometer data.
 *
 *  @details    Three axes × two bytes each.
 * ============================================================= */
#define MAX_ACCEL_BUF           (uint8_t)(6u)

/** =============================================================
 *  @def        US_PER_S
 *  @brief      Number of microseconds in one second.
 * ============================================================= */
#define US_PER_S                (uint32_t)(1000000UL)

/** =============================================================
 *  @def        RAW2MG(v)
 *  @brief      Convert raw sensor LSB to milli-g.
 *
 *  @param  v   Raw accelerometer reading (signed 16-bit).
 *  @return     Converted value in milli-g (signed 16-bit).
 * ============================================================= */
#define RAW2MG(v)               ((int16_t)(((int32_t)(v) * MG_PER_G) / LSB_PER_G))

/** =============================================================
 *  @def        ABS_DIFF(a, b)
 *  @brief      Compute absolute difference of two values.
 *
 *  @param  a   First value.
 *  @param  b   Second value.
 *  @return     Absolute difference (signed 16-bit).
 * ============================================================= */
#define ABS_DIFF(a, b)          ((int16_t)(((a) > (b)) ? ((a) - (b)) : ((b) - (a))))

/** =============================================================
 *  @def        DX2MM(dx)
 *  @brief      Approximate displacement in millimeters.
 *
 *  @param  dx  Delta in milli-g.
 *  @return     Displacement in millimeters (signed 16-bit).
 * ============================================================= */
#define DX2MM(dx)               ((int16_t)(((int32_t)(dx) * DX2MM_FACTOR + DX2MM_ROUNDING) / MG_PER_G))

/** =============================================================
 *  @def        MM2CM(mm)
 *  @brief      Convert millimeters to whole centimeters.
 *
 *  @param  mm  Displacement in millimeters.
 *  @return     Integer centimeters (signed 16-bit).
 * ============================================================= */
#define MM2CM(mm)               ((int16_t)((mm) / MM2CM_DIVISOR))

/** =============================================================
 *  @def        MM2CM_REM(mm)
 *  @brief      Remainder after converting mm to cm.
 *
 *  @param  mm  Displacement in millimeters.
 *  @return     Fractional millimeters (0–9).
 * ============================================================= */
#define MM2CM_REM(mm)           ((uint8_t)((mm) % MM2CM_DIVISOR))

/** =============================================================
 *  @def        MG2MS2(mg)
 *  @brief      Convert milli-g to meters per second squared.
 *
 *  @param  mg  Acceleration in milli-g.
 *  @return     Acceleration in m/s².
 * ============================================================= */
#define MG2MS2(mg)              ((float)(mg) * GRAVITY_MS2 / (float)MG_PER_G)

/** =============================================================
 *  @def        INTEGRATE(v, a)
 *  @brief      Integrate acceleration over fixed sample period.
 *
 *  @param  v   Previous velocity (m/s).
 *  @param  a   Acceleration (m/s²).
 *  @return     Updated velocity after one sample period.
 * ============================================================= */
#define INTEGRATE(v, a)         ((v) + (a) * SAMPLE_PERIOD_S)

/** =============================================================
 *  @def        DWT_INIT()
 *  @brief      Initialize DWT cycle counter for profiling.
 *
 *  @details    Enables trace, resets cycle count, and activates
 *              the cycle counter.
 * ============================================================= */
#define DWT_INIT()                                      \
  do                                                    \
  {                                                     \
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;     \
    DWT->CYCCNT = 0U;                                   \
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;               \
  } while (0)

/** =============================================================
 *  @def        PROFILE_START()
 *  @brief      Reset DWT cycle counter to zero.
 * ============================================================= */
#define PROFILE_START()         (DWT->CYCCNT = 0U)

/** =============================================================
 *  @def        PROFILE_STOP(us_var)
 *  @brief      Compute elapsed time in microseconds from cycle count.
 *
 *  @param[out] us_var  Variable to receive elapsed time (µs).
 * ============================================================= */
#define PROFILE_STOP(us_var)                                    \
  do                                                            \
  {                                                             \
    uint32_t _cycles = DWT->CYCCNT;                             \
    (us_var) = _cycles / (HAL_RCC_GetHCLKFreq() / US_PER_S);    \
  } while (0)

/** ===============================================================
 *          P U B L I C  S T R U C T U R E S  &  T Y P E S          
 * ================================================================ */

/** =============================================================
 *  @struct     SensorData
 *  @typedef    sensor_data_t
 *  @brief      Represents processed accelerometer sensor data.
 *
 *  @details    This structure holds raw readings from the MPU6050,
 *              intermediate unit conversions, computed deltas,
 *              approximate displacements, and derived physical
 *              quantities such as acceleration and velocity.
 * ============================================================= */
 typedef struct __attribute__((packed, aligned(4u))) SensorData
 {
     int16_t raw_ax, raw_ay, raw_az;         /**< Raw accelerometer readings (LSB) */
     int16_t mg_x, mg_y, mg_z;               /**< Converted readings in milli-g (mg) */
     int16_t dx, dy, dz;                     /**< Absolute variation between samples (mg) */
     int16_t mm_x, mm_y, mm_z;               /**< Approximate displacement in millimeters (mm) */
     int16_t cm_x, cm_y, cm_z;               /**< Integer part of displacement in centimeters (cm) */
     uint8_t cm_x_rem, cm_y_rem, cm_z_rem;   /**< Fractional part of displacement in millimeters (0–9) */
     float   ax, ay, az;                     /**< Acceleration in meters per second squared (m/s²) */
     float   vx, vy, vz;                     /**< Velocity in meters per second (m/s) */
 } sensor_data_t;

/** ===============================================================
 *       P U B L I C  F U N C T I O N S  P R O T O T Y P E S        
 * ================================================================ */

void SystemClock_Config(void);

static void MPU6050_initSensor(void);
static void MPU6050_readAccelRaw(int16_t* ax, int16_t* ay, int16_t* az);
static int MPU6050_sensorUpdate(sensor_data_t *sensor, const sensor_data_t *last);
static int MPU6050_formatBuffer(char *buf, size_t len, const sensor_data_t *sensor);

/** ===============================================================
 *       P U B L I C  F U N C T I O N S  D E F I N I T I O N S     
 * ================================================================ */

int main(void)
{
  char msg[MAX_MSG_LEN] =
    { [0 ... (MAX_MSG_LEN - 1u)] = ' ' };

  HAL_Init();

  SystemClock_Config();

  DWT_INIT();

  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM10_Init();

  HAL_TIM_Base_Start_IT(&htim10);

  MPU6050_initSensor();
  snprintf(msg,sizeof(msg),"MPU6050 initialized successfully!\r\n");
  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, strlen(msg));
}

/** ============================================================================
 *  @fn         SystemClock_Config
 *  @brief      Configures the system clocks: HSI oscillator, PLL, and bus prescalers.
 *
 *  @details    - Enables power control clock and sets voltage scaling  
 *              - Turns on the HSI oscillator and configures the PLL (M=16, N=336, P=4, Q=4)  
 *              - Sets SYSCLK source to PLL output  
 *              - Configures AHB, APB1, and APB2 prescalers  
 *              - Sets flash latency  
 *
 *  @note       Calls Error_Handler() in case of failure.
 * ========================================================================== */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/** ============================================================================
 *  @fn         HAL_TIM_PeriodElapsedCallback
 *  @brief      Timer interrupt callback, called on TIM10 update event.
 *
 *  @param[in]  htim  Pointer to the Timer handle that triggered the callback.
 *
 *  @details    - On first call, zeroes out `last` and `sensor` data buffers.  
 *              - Calls MPU6050_sensorUpdate() to compute new sensor metrics.  
 *              - Measures ISR execution time via DWT.  
 *              - If any axis delta exceeds threshold, formats message and 
 *                transmits it over UART using DMA.  
 *              - Copies current sensor state into `last` for next iteration.
 * ========================================================================== */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  static sensor_data_t last;
  static sensor_data_t sensor;

  static bool first_init = true;

  uint32_t isr_time_us = 0u;
  UNUSED(isr_time_us);

  char msg[MAX_MSG_LEN] =
    { [0 ... (MAX_MSG_LEN - 1u)] = ' ' };

  if (htim->Instance == TIM10)
  {
    PROFILE_START();

    if (first_init)
    {
      memset(&last,   0, sizeof(sensor_data_t));
      memset(&sensor, 0, sizeof(sensor_data_t));

      first_init = false;
    }

    MPU6050_sensorUpdate(&sensor, &last);

    PROFILE_STOP(isr_time_us);

    if (sensor.dx > THRESHOLD_MG || sensor.dy > THRESHOLD_MG || sensor.dz > THRESHOLD_MG)
    {
      MPU6050_formatBuffer(msg, sizeof(msg), &sensor);

      HAL_UART_Transmit_DMA(&huart2, (uint8_t*)msg, strlen(msg));       
    }
    
    memcpy(&last, &sensor, sizeof(sensor_data_t));
  }
}

/** ============================================================================
 *  @fn         MPU6050_initSensor
 *  @brief      Wakes up the MPU6050 and reads its WHO_AM_I register.
 *
 *  @details    - Writes 0x00 to PWR_MGMT_1 to clear sleep bit.  
 *              - Reads WHO_AM_I register for device identification.
 * ========================================================================== */
static void MPU6050_initSensor(void)
{
  uint8_t wake_cmd = 0x00;
  uint8_t who = 0;

 HAL_I2C_Mem_Write(&hi2c1,
                    MPU6050_ADDR,
                    MPU6050_REG_PWR_MGMT_1,
                    I2C_MEMADD_SIZE_8BIT,
                    &wake_cmd,
                    1,
                    HAL_MAX_DELAY);

  HAL_I2C_Mem_Read(&hi2c1,
                    MPU6050_ADDR,
                    MPU6050_REG_WHO_AM_I,
                    I2C_MEMADD_SIZE_8BIT,
                    &who,
                    1,
                    HAL_MAX_DELAY);
}

/** ============================================================================
 *  @fn         MPU6050_readAccelRaw
 *  @brief      Reads raw accelerometer data (X, Y, Z) from MPU6050.
 *
 *  @param[out] ax  Pointer to store raw X-axis reading.
 *  @param[out] ay  Pointer to store raw Y-axis reading.
 *  @param[out] az  Pointer to store raw Z-axis reading.
 *
 *  @details    - Reads 6 bytes starting at ACCEL_XOUT_H via I2C.  
 *              - Combines high and low bytes for each axis.
 * ========================================================================== */
static MPU6050_readAccelRaw(int16_t* ax, int16_t* ay, int16_t* az)
{
  uint8_t accel_buf[MAX_ACCEL_BUF] =
    { [0 ... (MAX_ACCEL_BUF - 1u)] = 0u };

  int16_t* axes[3] = { ax, ay, az };

  HAL_I2C_Mem_Read(&hi2c1,
                   MPU6050_ADDR,
                   ACCEL_XOUT_H,
                   I2C_MEMADD_SIZE_8BIT,
                   accel_buf,
                   6,
                   HAL_MAX_DELAY);

    for (size_t i = 0; i < 3; ++i)
    {
        *axes[i] = (int16_t)((accel_buf[2u*i] << 8u) | accel_buf[2u*i + 1u]);
    }
}

/** ============================================================================
 *  @fn         MPU6050_sensorUpdate
 *  @brief      Updates sensor_data_t based on raw readings and previous state.
 *
 *  @param[out] sensor  Pointer to current sensor data structure to populate.
 *  @param[in]  last    Pointer to previous sensor data for delta calculations.
 *
 *  @details    - Reads raw accelerometer data.  
 *              - Converts to mg, computes absolute deltas.  
 *              - Approximates displacement in mm and cm (with remainder).  
 *              - Converts mg to m/s² for acceleration.  
 *              - Integrates acceleration over fixed sample period to compute velocity.  
 *              - Copies results into the provided `sensor` structure.
 * ========================================================================== */
static int MPU6050_sensorUpdate(sensor_data_t *sensor, const sensor_data_t *last)
{
  int ret = 0u;

  if (sensor == NULL || last == NULL)
  {
    ret = -EINVAL;
    goto function_output;
  }

  sensor_data_t tmp;
  sensor_data_t tmp_last;

  memset(&tmp, 0, sizeof(tmp));
  memcpy(&tmp_last, last, sizeof(tmp_last));

  MPU9250_readAccelRaw(&tmp.raw_ax, &tmp.raw_ay, &tmp.raw_az);

  tmp.mg_x = RAW2MG(tmp.raw_ax);
  tmp.mg_y = RAW2MG(tmp.raw_ay);
  tmp.mg_z = RAW2MG(tmp.raw_az);

  tmp.dx = ABS_DIFF(tmp.mg_x, tmp_last.mg_x);
  tmp.dy = ABS_DIFF(tmp.mg_y, tmp_last.mg_y);
  tmp.dz = ABS_DIFF(tmp.mg_z, tmp_last.mg_z);

  tmp.mm_x = DX2MM(tmp.dx);
  tmp.mm_y = DX2MM(tmp.dy);
  tmp.mm_z = DX2MM(tmp.dz);

  tmp.cm_x     = MM2CM(tmp.mm_x);
  tmp.cm_x_rem = MM2CM_REM(tmp.mm_x);
  tmp.cm_y     = MM2CM(tmp.mm_y);
  tmp.cm_y_rem = MM2CM_REM(tmp.mm_y);
  tmp.cm_z     = MM2CM(tmp.mm_z);
  tmp.cm_z_rem = MM2CM_REM(tmp.mm_z);

  tmp.ax = MG2MS2(tmp.mg_x);
  tmp.ay = MG2MS2(tmp.mg_y);
  tmp.az = MG2MS2(tmp.mg_z);

  tmp.vx = INTEGRATE(tmp_last.vx, tmp.ax);
  tmp.vy = INTEGRATE(tmp_last.vy, tmp.ay);
  tmp.vz = INTEGRATE(tmp_last.vz, tmp.az);

  memcpy(sensor, &tmp, sizeof(*sensor));

function_output:
  return ret;
}

/** ============================================================================
 *  @fn         MPU6050_formatBuffer
 *  @brief      Formats sensor data into a human-readable message buffer.
 *
 *  @param[out] buf     Character buffer to receive formatted message.
 *  @param[in]  len     Length of the buffer.
 *  @param[in]  sensor  Pointer to populated sensor_data_t structure.
 *
 *  @details    - Copies sensor data locally, then uses snprintf() to build a
 *                multi-line message with deltas, displacements, acceleration,
 *                and velocity for each axis.  
 *              - Clears output buffer and copies formatted bytes into it.
 * ========================================================================== */
static int MPU6050_formatBuffer(char *buf, size_t len, const sensor_data_t *sensor)
{
  int ret = 0u;

  sensor_data_t tmp_sensor;
  char tmp_buf[MAX_MSG_LEN];

  if (sensor == NULL || len <= 0)
  {
    ret = -EINVAL;
    goto function_output;
  }
  
  memset(tmp_buf, 0, len);
  memcpy(&tmp_sensor, sensor, sizeof(tmp_sensor));

  snprintf(tmp_buf, len,
      "\r\n Movimento! "
      "ΔX=%d mg (%d.%d cm), aX=%.2f m/s², vX=%.2f m/s;\n"
      "ΔY=%d mg (%d.%d cm), aY=%.2f m/s², vY=%.2f m/s;\n"
      "ΔZ=%d mg (%d.%d cm), aZ=%.2f m/s², vZ=%.2f m/s;\n"
      "\r\n",
      tmp_sensor.dx, tmp_sensor.cm_x,   tmp_sensor.cm_x_rem, tmp_sensor.ax, tmp_sensor.vx,
      tmp_sensor.dy, tmp_sensor.cm_y,   tmp_sensor.cm_y_rem, tmp_sensor.ay, tmp_sensor.vy,
      tmp_sensor.dz, tmp_sensor.cm_z,   tmp_sensor.cm_z_rem, tmp_sensor.az, tmp_sensor.vz
    );

  memset(buf, 0, len);
  memcpy(buf, tmp_buf, len);

function_output:
  return ret;
}
