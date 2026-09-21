/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Sensor state-machine + STM32-to-Zybo UART + OLED demo
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include "VL53L1X_api.h"
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* Bus assignment */
#define TOF_I2C_HANDLE              hi2c2
#define MUX_I2C_HANDLE              hi2c4

/* VL53L1X ToF */
#define VL53L1X_ADDR_7BIT           0x29
#define VL53L1X_ADDR_8BIT           0x52

/*
 * Updated ToF wiring:
 * PB9 = XSHUT / SHUT output
 * PC6 = INT / GPIO1 input
 */
#define TOF_XSHUT_PORT              GPIOB
#define TOF_XSHUT_PIN               GPIO_PIN_9

#define TOF_INT_PORT                GPIOC
#define TOF_INT_PIN                 GPIO_PIN_6

#define TOF_NEAR_THRESHOLD_MM       150

/* PCA9548A / TCA9548A mux */
#define MUX_ADDR_MIN_7BIT           0x70
#define MUX_ADDR_MAX_7BIT           0x77

#define BME280_MUX_CH               0
#define SHT41_MUX_CH                1
#define OLED1_MUX_CH                2
#define OLED2_MUX_CH                3
#define OLED3_MUX_CH                4
#define OLED4_MUX_CH                5

/* Downstream I2C addresses */
#define BME280_ADDR_A               0x76
#define BME280_ADDR_B               0x77
#define BME280_REG_ID               0xD0
#define BME280_EXPECTED_ID          0x60

#define BME280_REG_CALIB_TP_START   0x88
#define BME280_REG_CALIB_TP_LEN     26
#define BME280_REG_CALIB_H1         0xA1
#define BME280_REG_CALIB_H_START    0xE1
#define BME280_REG_CALIB_H_LEN      7

#define BME280_REG_CTRL_HUM         0xF2
#define BME280_REG_STATUS           0xF3
#define BME280_REG_CTRL_MEAS        0xF4
#define BME280_REG_CONFIG           0xF5
#define BME280_REG_DATA_START       0xF7
#define BME280_REG_DATA_LEN         8

#define SHT41_ADDR_7BIT             0x44
#define SHT41_ADDR_8BIT             (SHT41_ADDR_7BIT << 1)

#define OLED_ADDR                   0x3C

/* SSD1306 OLED displays on mux CH2-CH5 */
#define SSD1306_ADDR_7BIT           0x3C
#define SSD1306_ADDR_8BIT           (SSD1306_ADDR_7BIT << 1)
#define SSD1306_PAGES               8

#define OLED_UPDATE_MS              1000

/* Environment thresholds */
#define TEMP_ALERT_CENTI            3000    /* 30.00 C */
#define HUM_ALERT_CENTI             7000    /* 70.00 % */

/* LSM6DS3 SPI */
#define LSM6DS3_CS_PORT             GPIOD
#define LSM6DS3_CS_PIN              GPIO_PIN_14

#define LSM6DS3_WHO_AM_I_REG        0x0F
#define LSM6DS3_EXPECTED_ID         0x69

#define LSM6DS3_CTRL1_XL            0x10
#define LSM6DS3_CTRL3_C             0x12

#define LSM6DS3_OUTX_L_XL           0x28
#define LSM6DS3_OUTX_H_XL           0x29
#define LSM6DS3_OUTY_L_XL           0x2A
#define LSM6DS3_OUTY_H_XL           0x2B
#define LSM6DS3_OUTZ_L_XL           0x2C
#define LSM6DS3_OUTZ_H_XL           0x2D

/*
 * Motion tuning.
 * If idle still false-triggers, raise this to 16000 or 20000.
 */
#define MOTION_DELTA_THRESHOLD       12000
#define MOTION_HIT_COUNT_REQUIRED    3
#define MOTION_QUIET_COUNT_REQUIRED  12

/* Poll intervals */
#define TOF_POLL_MS                 100
#define ENV_POLL_MS                 1000
#define MOTION_POLL_MS              200
#define STATUS_PRINT_MS             3000

/* Zybo command bytes */
#define CMD_DISTANCE_NEAR           'D'
#define CMD_DISTANCE_NORMAL         'N'
#define CMD_ENV_ALERT               'E'
#define CMD_ENV_NORMAL              'e'
#define CMD_MOTION_DETECTED         'M'
#define CMD_MOTION_NORMAL           'm'
#define CMD_CLEAR_ALL               'C'
#define CMD_HEARTBEAT               'H'

#define HEARTBEAT_TX_MS             1000

/* USER CODE END PD */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c4;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static uint8_t mux_addr_found = 0;

static uint8_t tof_initialized = 0;
static uint8_t tof_near_state = 0;
static uint16_t latest_distance_mm = 0;

static uint8_t env_alert_state = 0;
static int32_t latest_temp_centi = 0;
static int32_t latest_hum_centi = 0;

static uint8_t bme280_detected = 0;
static uint8_t bme280_addr_7bit = 0;
static int32_t latest_sht_temp_centi = 0;
static int32_t latest_sht_hum_centi = 0;
static int32_t latest_bme_temp_centi = 0;
static int32_t latest_bme_hum_centi = 0;

typedef struct
{
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;

  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4;
  int16_t  dig_H5;
  int8_t   dig_H6;

  int32_t  t_fine;
} BME280_Calib_t;

static BME280_Calib_t bme280_calib;

static uint8_t lsm_detected = 0;
static uint8_t motion_state = 0;
static int16_t prev_ax = 0;
static int16_t prev_ay = 0;
static int16_t prev_az = 0;
static uint8_t motion_baseline_valid = 0;

static uint8_t motion_hit_count = 0;
static uint8_t motion_quiet_count = 0;
static int32_t latest_motion_delta = 0;

static uint8_t oled1_ok = 0;
static uint8_t oled2_ok = 0;
static uint8_t oled3_ok = 0;
static uint8_t oled4_ok = 0;

static uint8_t latest_zybo_cmd = CMD_CLEAR_ALL;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C4_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);

/* USER CODE BEGIN PFP */
static void uart2_print(const char *msg);
static void uart2_printf(const char *fmt, ...);
static void zybo_send(uint8_t cmd);
static void zybo_send_heartbeat(void);

static void scan_i2c_bus(I2C_HandleTypeDef *hi2c, const char *label);

static uint8_t find_mux_on_i2c4(void);
static HAL_StatusTypeDef mux_select(uint8_t mux_addr_7bit, uint8_t channel);
static void mux_disable_all(uint8_t mux_addr_7bit);
static void scan_mux_channel(uint8_t mux_addr_7bit, uint8_t channel, const char *label);

static HAL_StatusTypeDef oled_write_command(uint8_t cmd);
static HAL_StatusTypeDef oled_write_data(uint8_t *data, uint16_t size);
static int oled_init_on_channel(uint8_t mux_addr_7bit, uint8_t channel);
static void oled_clear(void);
static void oled_set_cursor(uint8_t page, uint8_t col);
static void oled_write_char(char c);
static void oled_write_string(const char *str);
static void oled_write_line(uint8_t page, const char *str);
static void oled_init_all_displays(void);
static void oled_update_all_displays(void);

static void tof_xshut_low(void);
static void tof_xshut_high(void);
static int tof_init_sensor(void);
static void tof_poll_state(void);

static int bme280_read_id(uint8_t mux_addr_7bit);
static int bme280_init_sensor(uint8_t mux_addr_7bit);
static int bme280_read_calibration(uint8_t mux_addr_7bit);
static int bme280_read_temp_humidity(uint8_t mux_addr_7bit, int32_t *temp_centi, int32_t *hum_centi);

static uint8_t sht41_crc8(uint8_t *data, uint8_t len);
static int sht41_read_temp_humidity(uint8_t mux_addr_7bit, int32_t *temp_centi, int32_t *hum_centi);
static void env_poll_state(void);

static void lsm6ds3_cs_low(void);
static void lsm6ds3_cs_high(void);
static uint8_t lsm6ds3_read_reg(uint8_t reg);
static void lsm6ds3_write_reg(uint8_t reg, uint8_t value);
static int lsm6ds3_init(void);
static void lsm6ds3_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az);
static void motion_poll_state(void);

static void print_system_status(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

static void uart2_print(const char *msg)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), HAL_MAX_DELAY);
}

static void uart2_printf(const char *fmt, ...)
{
  char buffer[192];

  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);

  uart2_print(buffer);
}

static void zybo_send(uint8_t cmd)
{
  latest_zybo_cmd = cmd;
  HAL_UART_Transmit(&huart3, &cmd, 1, HAL_MAX_DELAY);
}

static void zybo_send_heartbeat(void)
{
  uint8_t cmd = (uint8_t)CMD_HEARTBEAT;

  /*
   * Heartbeat is intentionally sent without updating latest_zybo_cmd.
   * This keeps OLED4 showing the latest meaningful alert command
   * instead of constantly showing CMD H.
   */
  HAL_UART_Transmit(&huart3, &cmd, 1, HAL_MAX_DELAY);
}

/*
 * Minimal 5x7 font from ASCII 0x20 to 0x5A.
 * Lowercase letters are converted to uppercase.
 */
static const uint8_t font5x7[][5] =
{
  {0x00,0x00,0x00,0x00,0x00}, /* space */
  {0x00,0x00,0x5F,0x00,0x00}, /* ! */
  {0x00,0x07,0x00,0x07,0x00}, /* " */
  {0x14,0x7F,0x14,0x7F,0x14}, /* # */
  {0x24,0x2A,0x7F,0x2A,0x12}, /* $ */
  {0x23,0x13,0x08,0x64,0x62}, /* % */
  {0x36,0x49,0x55,0x22,0x50}, /* & */
  {0x00,0x05,0x03,0x00,0x00}, /* ' */
  {0x00,0x1C,0x22,0x41,0x00}, /* ( */
  {0x00,0x41,0x22,0x1C,0x00}, /* ) */
  {0x14,0x08,0x3E,0x08,0x14}, /* * */
  {0x08,0x08,0x3E,0x08,0x08}, /* + */
  {0x00,0x50,0x30,0x00,0x00}, /* , */
  {0x08,0x08,0x08,0x08,0x08}, /* - */
  {0x00,0x60,0x60,0x00,0x00}, /* . */
  {0x20,0x10,0x08,0x04,0x02}, /* / */
  {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
  {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
  {0x42,0x61,0x51,0x49,0x46}, /* 2 */
  {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
  {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
  {0x27,0x45,0x45,0x45,0x39}, /* 5 */
  {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
  {0x01,0x71,0x09,0x05,0x03}, /* 7 */
  {0x36,0x49,0x49,0x49,0x36}, /* 8 */
  {0x06,0x49,0x49,0x29,0x1E}, /* 9 */
  {0x00,0x36,0x36,0x00,0x00}, /* : */
  {0x00,0x56,0x36,0x00,0x00}, /* ; */
  {0x08,0x14,0x22,0x41,0x00}, /* < */
  {0x14,0x14,0x14,0x14,0x14}, /* = */
  {0x00,0x41,0x22,0x14,0x08}, /* > */
  {0x02,0x01,0x51,0x09,0x06}, /* ? */
  {0x32,0x49,0x79,0x41,0x3E}, /* @ */
  {0x7E,0x11,0x11,0x11,0x7E}, /* A */
  {0x7F,0x49,0x49,0x49,0x36}, /* B */
  {0x3E,0x41,0x41,0x41,0x22}, /* C */
  {0x7F,0x41,0x41,0x22,0x1C}, /* D */
  {0x7F,0x49,0x49,0x49,0x41}, /* E */
  {0x7F,0x09,0x09,0x09,0x01}, /* F */
  {0x3E,0x41,0x49,0x49,0x7A}, /* G */
  {0x7F,0x08,0x08,0x08,0x7F}, /* H */
  {0x00,0x41,0x7F,0x41,0x00}, /* I */
  {0x20,0x40,0x41,0x3F,0x01}, /* J */
  {0x7F,0x08,0x14,0x22,0x41}, /* K */
  {0x7F,0x40,0x40,0x40,0x40}, /* L */
  {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
  {0x7F,0x04,0x08,0x10,0x7F}, /* N */
  {0x3E,0x41,0x41,0x41,0x3E}, /* O */
  {0x7F,0x09,0x09,0x09,0x06}, /* P */
  {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
  {0x7F,0x09,0x19,0x29,0x46}, /* R */
  {0x46,0x49,0x49,0x49,0x31}, /* S */
  {0x01,0x01,0x7F,0x01,0x01}, /* T */
  {0x3F,0x40,0x40,0x40,0x3F}, /* U */
  {0x1F,0x20,0x40,0x20,0x1F}, /* V */
  {0x7F,0x20,0x18,0x20,0x7F}, /* W */
  {0x63,0x14,0x08,0x14,0x63}, /* X */
  {0x07,0x08,0x70,0x08,0x07}, /* Y */
  {0x61,0x51,0x49,0x45,0x43}  /* Z */
};

static void scan_i2c_bus(I2C_HandleTypeDef *hi2c, const char *label)
{
  uint8_t found_count = 0;

  uart2_printf("\r\n--- I2C scan: %s ---\r\n", label);

  for (uint8_t addr = 1; addr < 128; addr++)
  {
    if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 20) == HAL_OK)
    {
      found_count++;

      uart2_printf("Found I2C device at 0x%02X", addr);

      if (addr == VL53L1X_ADDR_7BIT)
      {
        uart2_print("  <-- VL53L1X ToF");
      }

      if ((addr >= MUX_ADDR_MIN_7BIT) && (addr <= MUX_ADDR_MAX_7BIT))
      {
        uart2_print("  <-- I2C mux range");
      }

      if ((addr == BME280_ADDR_A) || (addr == BME280_ADDR_B))
      {
        uart2_print("  <-- possible BME280");
      }

      if (addr == SHT41_ADDR_7BIT)
      {
        uart2_print("  <-- possible SHT41");
      }

      if (addr == OLED_ADDR)
      {
        uart2_print("  <-- possible OLED");
      }

      uart2_print("\r\n");
    }
  }

  uart2_printf("Scan complete. Devices found: %u\r\n", found_count);
}

static uint8_t find_mux_on_i2c4(void)
{
  uart2_print("\r\n--- Checking mux address range 0x70-0x77 on I2C4 ---\r\n");

  for (uint8_t addr = MUX_ADDR_MIN_7BIT; addr <= MUX_ADDR_MAX_7BIT; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&MUX_I2C_HANDLE, (uint16_t)(addr << 1), 3, 50) == HAL_OK)
    {
      uart2_printf("MUX ACK at 0x%02X on I2C4\r\n", addr);
      BSP_LED_On(LED_GREEN);
      return addr;
    }
  }

  uart2_print("No mux detected at 0x70-0x77 on I2C4.\r\n");
  BSP_LED_On(LED_RED);
  return 0;
}

static HAL_StatusTypeDef mux_select(uint8_t mux_addr_7bit, uint8_t channel)
{
  if (channel > 7)
  {
    return HAL_ERROR;
  }

  uint8_t select_data = (uint8_t)(1U << channel);
  uint16_t mux_addr_8bit = (uint16_t)(mux_addr_7bit << 1);

  return HAL_I2C_Master_Transmit(&MUX_I2C_HANDLE, mux_addr_8bit, &select_data, 1, 100);
}

static void mux_disable_all(uint8_t mux_addr_7bit)
{
  uint16_t mux_addr_8bit = (uint16_t)(mux_addr_7bit << 1);
  uint8_t disable_all = 0x00;

  HAL_I2C_Master_Transmit(&MUX_I2C_HANDLE, mux_addr_8bit, &disable_all, 1, 100);
}

static void scan_mux_channel(uint8_t mux_addr_7bit, uint8_t channel, const char *label)
{
  uint8_t found_count = 0;

  uart2_printf("\r\n--- Scanning mux %s ---\r\n", label);

  if (mux_select(mux_addr_7bit, channel) != HAL_OK)
  {
    uart2_printf("Failed to select mux channel %u.\r\n", channel);
    return;
  }

  HAL_Delay(10);

  for (uint8_t addr = 1; addr < 128; addr++)
  {
    if (addr == mux_addr_7bit)
    {
      continue;
    }

    if (HAL_I2C_IsDeviceReady(&MUX_I2C_HANDLE, (uint16_t)(addr << 1), 2, 20) == HAL_OK)
    {
      found_count++;
      uart2_printf("  Found downstream device at 0x%02X\r\n", addr);
    }
  }

  if (found_count == 0)
  {
    uart2_print("  No downstream devices found on this channel.\r\n");
  }
}

/* ---------------- OLED / SSD1306 driver ---------------- */

static HAL_StatusTypeDef oled_write_command(uint8_t cmd)
{
  uint8_t buffer[2];

  buffer[0] = 0x00;
  buffer[1] = cmd;

  return HAL_I2C_Master_Transmit(&MUX_I2C_HANDLE,
                                 SSD1306_ADDR_8BIT,
                                 buffer,
                                 2,
                                 100);
}

static HAL_StatusTypeDef oled_write_data(uint8_t *data, uint16_t size)
{
  uint8_t buffer[17];

  while (size > 0)
  {
    uint8_t chunk = (size > 16U) ? 16U : (uint8_t)size;

    buffer[0] = 0x40;

    for (uint8_t i = 0; i < chunk; i++)
    {
      buffer[i + 1U] = data[i];
    }

    if (HAL_I2C_Master_Transmit(&MUX_I2C_HANDLE,
                                SSD1306_ADDR_8BIT,
                                buffer,
                                (uint16_t)(chunk + 1U),
                                100) != HAL_OK)
    {
      return HAL_ERROR;
    }

    data += chunk;
    size = (uint16_t)(size - chunk);
  }

  return HAL_OK;
}

static void oled_set_cursor(uint8_t page, uint8_t col)
{
  oled_write_command((uint8_t)(0xB0 + page));
  oled_write_command((uint8_t)(0x00 + (col & 0x0F)));
  oled_write_command((uint8_t)(0x10 + ((col >> 4) & 0x0F)));
}

static void oled_clear(void)
{
  uint8_t zeros[16];

  memset(zeros, 0x00, sizeof(zeros));

  for (uint8_t page = 0; page < SSD1306_PAGES; page++)
  {
    oled_set_cursor(page, 0);

    for (uint8_t block = 0; block < 8; block++)
    {
      oled_write_data(zeros, sizeof(zeros));
    }
  }
}

static int oled_init_on_channel(uint8_t mux_addr_7bit, uint8_t channel)
{
  if (mux_select(mux_addr_7bit, channel) != HAL_OK)
  {
    return -1;
  }

  HAL_Delay(20);

  if (HAL_I2C_IsDeviceReady(&MUX_I2C_HANDLE, SSD1306_ADDR_8BIT, 2, 100) != HAL_OK)
  {
    return -1;
  }

  oled_write_command(0xAE);
  oled_write_command(0x20);
  oled_write_command(0x00);
  oled_write_command(0xB0);
  oled_write_command(0xC8);
  oled_write_command(0x00);
  oled_write_command(0x10);
  oled_write_command(0x40);
  oled_write_command(0x81);
  oled_write_command(0x7F);
  oled_write_command(0xA1);
  oled_write_command(0xA6);
  oled_write_command(0xA8);
  oled_write_command(0x3F);
  oled_write_command(0xA4);
  oled_write_command(0xD3);
  oled_write_command(0x00);
  oled_write_command(0xD5);
  oled_write_command(0x80);
  oled_write_command(0xD9);
  oled_write_command(0xF1);
  oled_write_command(0xDA);
  oled_write_command(0x12);
  oled_write_command(0xDB);
  oled_write_command(0x40);
  oled_write_command(0x8D);
  oled_write_command(0x14);
  oled_write_command(0xAF);

  oled_clear();

  return 0;
}

static void oled_write_char(char c)
{
  uint8_t data[6];

  if (c >= 'a' && c <= 'z')
  {
    c = (char)(c - 32);
  }

  if (c < ' ' || c > 'Z')
  {
    c = ' ';
  }

  uint8_t index = (uint8_t)(c - ' ');

  data[0] = font5x7[index][0];
  data[1] = font5x7[index][1];
  data[2] = font5x7[index][2];
  data[3] = font5x7[index][3];
  data[4] = font5x7[index][4];
  data[5] = 0x00;

  oled_write_data(data, sizeof(data));
}

static void oled_write_string(const char *str)
{
  while (*str)
  {
    oled_write_char(*str);
    str++;
  }
}

static void oled_write_line(uint8_t page, const char *str)
{
  oled_set_cursor(page, 0);
  oled_write_string(str);
}

static void oled_init_all_displays(void)
{
  if (!mux_addr_found)
  {
    uart2_print("OLED init skipped: mux not found.\r\n");
    return;
  }

  uart2_print("\r\n--- OLED live-display init ---\r\n");

  oled1_ok = (oled_init_on_channel(mux_addr_found, OLED1_MUX_CH) == 0) ? 1U : 0U;
  uart2_printf("OLED1 CH2 %s\r\n", oled1_ok ? "OK" : "FAILED");

  oled2_ok = (oled_init_on_channel(mux_addr_found, OLED2_MUX_CH) == 0) ? 1U : 0U;
  uart2_printf("OLED2 CH3 %s\r\n", oled2_ok ? "OK" : "FAILED");

  oled3_ok = (oled_init_on_channel(mux_addr_found, OLED3_MUX_CH) == 0) ? 1U : 0U;
  uart2_printf("OLED3 CH4 %s\r\n", oled3_ok ? "OK" : "FAILED");

  oled4_ok = (oled_init_on_channel(mux_addr_found, OLED4_MUX_CH) == 0) ? 1U : 0U;
  uart2_printf("OLED4 CH5 %s\r\n", oled4_ok ? "OK" : "FAILED");

  mux_disable_all(mux_addr_found);
}

static void oled_update_all_displays(void)
{
  char line[24];

  if (!mux_addr_found)
  {
    return;
  }

  if (oled1_ok)
  {
    mux_select(mux_addr_found, OLED1_MUX_CH);
    oled_clear();

    oled_write_line(0, "TOF DISTANCE");

    snprintf(line, sizeof(line), "%u MM", latest_distance_mm);
    oled_write_line(2, line);

    oled_write_line(4, tof_near_state ? "STATE NEAR" : "STATE NORMAL");
  }

  if (oled2_ok)
  {
    mux_select(mux_addr_found, OLED2_MUX_CH);
    oled_clear();

    oled_write_line(0, "ENV AVG");

    snprintf(line, sizeof(line), "T %ld.%02ld C",
             (long)(latest_temp_centi / 100),
             labs((long)(latest_temp_centi % 100)));
    oled_write_line(2, line);

    snprintf(line, sizeof(line), "H %ld.%02ld %%",
             (long)(latest_hum_centi / 100),
             labs((long)(latest_hum_centi % 100)));
    oled_write_line(4, line);

    oled_write_line(6, env_alert_state ? "STATE ALERT" : "STATE NORMAL");
  }

  if (oled3_ok)
  {
    mux_select(mux_addr_found, OLED3_MUX_CH);
    oled_clear();

    oled_write_line(0, "MOTION");

    oled_write_line(2, motion_state ? "STATE MOTION" : "STATE NORMAL");

    snprintf(line, sizeof(line), "DELTA %ld", (long)latest_motion_delta);
    oled_write_line(4, line);
  }

  if (oled4_ok)
  {
    mux_select(mux_addr_found, OLED4_MUX_CH);
    oled_clear();

    oled_write_line(0, "SYSTEM SUMMARY");

    snprintf(line, sizeof(line), "CMD %c", (char)latest_zybo_cmd);
    oled_write_line(2, line);

    snprintf(line, sizeof(line), "D%d E%d M%d",
             tof_near_state ? 1 : 0,
             env_alert_state ? 1 : 0,
             motion_state ? 1 : 0);
    oled_write_line(4, line);

    oled_write_line(6, "STM32 TO ZYBO OK");
  }

  mux_disable_all(mux_addr_found);
}

/* ---------------- ToF ---------------- */

static void tof_xshut_low(void)
{
  HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_RESET);
}

static void tof_xshut_high(void)
{
  HAL_GPIO_WritePin(TOF_XSHUT_PORT, TOF_XSHUT_PIN, GPIO_PIN_SET);
  HAL_Delay(50);
}

static int tof_init_sensor(void)
{
  uint8_t boot_state = 0;
  uint32_t timeout_count = 0;
  int8_t status = 0;

  uart2_print("\r\n--- VL53L1X ToF init ---\r\n");

  tof_xshut_low();
  HAL_Delay(10);
  tof_xshut_high();

  while (boot_state == 0)
  {
    status = VL53L1X_BootState(VL53L1X_ADDR_8BIT, &boot_state);

    if (status != 0)
    {
      uart2_printf("VL53L1X_BootState error: %d\r\n", status);
      return -1;
    }

    HAL_Delay(10);
    timeout_count++;

    if (timeout_count > 100)
    {
      uart2_print("VL53L1X boot timeout.\r\n");
      return -1;
    }
  }

  uart2_print("VL53L1X booted.\r\n");

  status = VL53L1X_SensorInit(VL53L1X_ADDR_8BIT);
  if (status != 0)
  {
    uart2_printf("VL53L1X_SensorInit error: %d\r\n", status);
    return -1;
  }

  status = VL53L1X_SetDistanceMode(VL53L1X_ADDR_8BIT, 2);
  if (status != 0)
  {
    uart2_printf("VL53L1X_SetDistanceMode error: %d\r\n", status);
    return -1;
  }

  status = VL53L1X_SetTimingBudgetInMs(VL53L1X_ADDR_8BIT, 50);
  if (status != 0)
  {
    uart2_printf("VL53L1X_SetTimingBudgetInMs error: %d\r\n", status);
    return -1;
  }

  status = VL53L1X_SetInterMeasurementInMs(VL53L1X_ADDR_8BIT, 100);
  if (status != 0)
  {
    uart2_printf("VL53L1X_SetInterMeasurementInMs error: %d\r\n", status);
    return -1;
  }

  status = VL53L1X_StartRanging(VL53L1X_ADDR_8BIT);
  if (status != 0)
  {
    uart2_printf("VL53L1X_StartRanging error: %d\r\n", status);
    return -1;
  }

  tof_initialized = 1;
  uart2_print("VL53L1X ranging started.\r\n");
  return 0;
}

static void tof_poll_state(void)
{
  uint8_t data_ready = 0;
  uint16_t distance_mm = 0;
  int8_t status = 0;

  if (!tof_initialized)
  {
    return;
  }

  status = VL53L1X_CheckForDataReady(VL53L1X_ADDR_8BIT, &data_ready);
  if (status != 0)
  {
    uart2_printf("VL53L1X CheckForDataReady error: %d\r\n", status);
    return;
  }

  if (!data_ready)
  {
    return;
  }

  status = VL53L1X_GetDistance(VL53L1X_ADDR_8BIT, &distance_mm);
  if (status != 0)
  {
    uart2_printf("VL53L1X GetDistance error: %d\r\n", status);
    VL53L1X_ClearInterrupt(VL53L1X_ADDR_8BIT);
    return;
  }

  latest_distance_mm = distance_mm;

  uint8_t near_now = (distance_mm < TOF_NEAR_THRESHOLD_MM) ? 1U : 0U;

  if (near_now != tof_near_state)
  {
    tof_near_state = near_now;

    if (tof_near_state)
    {
      uart2_printf("STATE CHANGE: TOF NEAR, %u mm. Sent D.\r\n", distance_mm);
      zybo_send((uint8_t)CMD_DISTANCE_NEAR);
    }
    else
    {
      uart2_printf("STATE CHANGE: TOF NORMAL, %u mm. Sent N.\r\n", distance_mm);
      zybo_send((uint8_t)CMD_DISTANCE_NORMAL);
    }
  }

  VL53L1X_ClearInterrupt(VL53L1X_ADDR_8BIT);
}

/* ---------------- BME/SHT ---------------- */

static int bme280_read_id(uint8_t mux_addr_7bit)
{
  uint8_t id = 0;
  uint8_t bme_addrs[2] = {BME280_ADDR_A, BME280_ADDR_B};

  if (mux_select(mux_addr_7bit, BME280_MUX_CH) != HAL_OK)
  {
    return -1;
  }

  HAL_Delay(10);

  for (uint8_t i = 0; i < 2; i++)
  {
    uint8_t addr7 = bme_addrs[i];
    uint16_t addr8 = (uint16_t)(addr7 << 1);

    if (HAL_I2C_Mem_Read(&MUX_I2C_HANDLE,
                         addr8,
                         BME280_REG_ID,
                         I2C_MEMADD_SIZE_8BIT,
                         &id,
                         1,
                         100) == HAL_OK)
    {
      uart2_printf("BME280 ID check: addr=0x%02X ID=0x%02X\r\n", addr7, id);

      if (id == BME280_EXPECTED_ID)
      {
        bme280_addr_7bit = addr7;
        return 0;
      }
    }
  }

  uart2_print("BME280 ID read failed or ID mismatch.\r\n");
  return -1;
}

static uint16_t bme280_u16_le(uint8_t lsb, uint8_t msb)
{
  return (uint16_t)(((uint16_t)msb << 8) | lsb);
}

static int16_t bme280_s16_le(uint8_t lsb, uint8_t msb)
{
  return (int16_t)(((uint16_t)msb << 8) | lsb);
}

static int bme280_read_calibration(uint8_t mux_addr_7bit)
{
  uint8_t tp[BME280_REG_CALIB_TP_LEN];
  uint8_t h1 = 0;
  uint8_t h[BME280_REG_CALIB_H_LEN];
  uint16_t addr8 = 0;

  if (!bme280_addr_7bit)
  {
    return -1;
  }

  if (mux_select(mux_addr_7bit, BME280_MUX_CH) != HAL_OK)
  {
    return -1;
  }

  HAL_Delay(10);

  addr8 = (uint16_t)(bme280_addr_7bit << 1);

  if (HAL_I2C_Mem_Read(&MUX_I2C_HANDLE,
                       addr8,
                       BME280_REG_CALIB_TP_START,
                       I2C_MEMADD_SIZE_8BIT,
                       tp,
                       BME280_REG_CALIB_TP_LEN,
                       100) != HAL_OK)
  {
    uart2_print("BME280 temp calibration read failed.\r\n");
    return -1;
  }

  if (HAL_I2C_Mem_Read(&MUX_I2C_HANDLE,
                       addr8,
                       BME280_REG_CALIB_H1,
                       I2C_MEMADD_SIZE_8BIT,
                       &h1,
                       1,
                       100) != HAL_OK)
  {
    uart2_print("BME280 H1 calibration read failed.\r\n");
    return -1;
  }

  if (HAL_I2C_Mem_Read(&MUX_I2C_HANDLE,
                       addr8,
                       BME280_REG_CALIB_H_START,
                       I2C_MEMADD_SIZE_8BIT,
                       h,
                       BME280_REG_CALIB_H_LEN,
                       100) != HAL_OK)
  {
    uart2_print("BME280 humidity calibration read failed.\r\n");
    return -1;
  }

  bme280_calib.dig_T1 = bme280_u16_le(tp[0], tp[1]);
  bme280_calib.dig_T2 = bme280_s16_le(tp[2], tp[3]);
  bme280_calib.dig_T3 = bme280_s16_le(tp[4], tp[5]);

  bme280_calib.dig_H1 = h1;
  bme280_calib.dig_H2 = bme280_s16_le(h[0], h[1]);
  bme280_calib.dig_H3 = h[2];

  bme280_calib.dig_H4 = (int16_t)(((int16_t)h[3] << 4) | (h[4] & 0x0F));
  if (bme280_calib.dig_H4 & 0x0800)
  {
    bme280_calib.dig_H4 |= 0xF000;
  }

  bme280_calib.dig_H5 = (int16_t)(((int16_t)h[5] << 4) | (h[4] >> 4));
  if (bme280_calib.dig_H5 & 0x0800)
  {
    bme280_calib.dig_H5 |= 0xF000;
  }

  bme280_calib.dig_H6 = (int8_t)h[6];

  uart2_print("BME280 calibration loaded.\r\n");
  return 0;
}

static int bme280_init_sensor(uint8_t mux_addr_7bit)
{
  uint16_t addr8 = 0;
  uint8_t value = 0;

  if (bme280_read_id(mux_addr_7bit) != 0)
  {
    bme280_detected = 0;
    return -1;
  }

  if (bme280_read_calibration(mux_addr_7bit) != 0)
  {
    bme280_detected = 0;
    return -1;
  }

  if (mux_select(mux_addr_7bit, BME280_MUX_CH) != HAL_OK)
  {
    bme280_detected = 0;
    return -1;
  }

  HAL_Delay(10);

  addr8 = (uint16_t)(bme280_addr_7bit << 1);

  value = 0x01;
  if (HAL_I2C_Mem_Write(&MUX_I2C_HANDLE,
                        addr8,
                        BME280_REG_CTRL_HUM,
                        I2C_MEMADD_SIZE_8BIT,
                        &value,
                        1,
                        100) != HAL_OK)
  {
    uart2_print("BME280 ctrl_hum write failed.\r\n");
    bme280_detected = 0;
    return -1;
  }

  value = 0x27;
  if (HAL_I2C_Mem_Write(&MUX_I2C_HANDLE,
                        addr8,
                        BME280_REG_CTRL_MEAS,
                        I2C_MEMADD_SIZE_8BIT,
                        &value,
                        1,
                        100) != HAL_OK)
  {
    uart2_print("BME280 ctrl_meas write failed.\r\n");
    bme280_detected = 0;
    return -1;
  }

  value = 0xA0;
  HAL_I2C_Mem_Write(&MUX_I2C_HANDLE,
                    addr8,
                    BME280_REG_CONFIG,
                    I2C_MEMADD_SIZE_8BIT,
                    &value,
                    1,
                    100);

  HAL_Delay(100);

  bme280_detected = 1;
  uart2_print("BME280 temp/humidity measurement enabled.\r\n");
  return 0;
}

static int bme280_read_temp_humidity(uint8_t mux_addr_7bit, int32_t *temp_centi, int32_t *hum_centi)
{
  uint8_t data[BME280_REG_DATA_LEN];
  uint16_t addr8 = 0;
  int32_t adc_T = 0;
  int32_t adc_H = 0;
  int32_t var1 = 0;
  int32_t var2 = 0;
  int32_t temp = 0;
  int32_t v_x1_u32r = 0;
  uint32_t hum_q22 = 0;

  if (!bme280_detected || !bme280_addr_7bit)
  {
    return -1;
  }

  if (mux_select(mux_addr_7bit, BME280_MUX_CH) != HAL_OK)
  {
    return -1;
  }

  HAL_Delay(10);
  addr8 = (uint16_t)(bme280_addr_7bit << 1);

  if (HAL_I2C_Mem_Read(&MUX_I2C_HANDLE,
                       addr8,
                       BME280_REG_DATA_START,
                       I2C_MEMADD_SIZE_8BIT,
                       data,
                       BME280_REG_DATA_LEN,
                       100) != HAL_OK)
  {
    return -1;
  }

  adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
  adc_H = ((int32_t)data[6] << 8) | data[7];

  var1 = ((((adc_T >> 3) - ((int32_t)bme280_calib.dig_T1 << 1))) *
          ((int32_t)bme280_calib.dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)bme280_calib.dig_T1))) >> 12) *
          ((int32_t)bme280_calib.dig_T3)) >> 14;

  bme280_calib.t_fine = var1 + var2;

  temp = (bme280_calib.t_fine * 5 + 128) >> 8;
  *temp_centi = temp;

  v_x1_u32r = bme280_calib.t_fine - 76800;

  v_x1_u32r =
      (((((adc_H << 14) -
          (((int32_t)bme280_calib.dig_H4) << 20) -
          (((int32_t)bme280_calib.dig_H5) * v_x1_u32r)) +
         16384) >> 15) *
       (((((((v_x1_u32r * ((int32_t)bme280_calib.dig_H6)) >> 10) *
            (((v_x1_u32r * ((int32_t)bme280_calib.dig_H3)) >> 11) + 32768)) >> 10) +
          2097152) *
             ((int32_t)bme280_calib.dig_H2) +
         8192) >> 14));

  v_x1_u32r =
      v_x1_u32r -
      (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
        ((int32_t)bme280_calib.dig_H1)) >> 4);

  if (v_x1_u32r < 0)
  {
    v_x1_u32r = 0;
  }

  if (v_x1_u32r > 419430400)
  {
    v_x1_u32r = 419430400;
  }

  hum_q22 = (uint32_t)(v_x1_u32r >> 12);
  *hum_centi = (int32_t)((hum_q22 * 100U) / 1024U);

  if (*hum_centi < 0)
  {
    *hum_centi = 0;
  }

  if (*hum_centi > 10000)
  {
    *hum_centi = 10000;
  }

  return 0;
}

static uint8_t sht41_crc8(uint8_t *data, uint8_t len)
{
  uint8_t crc = 0xFF;

  for (uint8_t i = 0; i < len; i++)
  {
    crc ^= data[i];

    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x80)
      {
        crc = (uint8_t)((crc << 1) ^ 0x31);
      }
      else
      {
        crc <<= 1;
      }
    }
  }

  return crc;
}

static int sht41_read_temp_humidity(uint8_t mux_addr_7bit, int32_t *temp_centi, int32_t *hum_centi)
{
  uint8_t cmd = 0xFD;
  uint8_t rx[6];

  if (mux_select(mux_addr_7bit, SHT41_MUX_CH) != HAL_OK)
  {
    return -1;
  }

  HAL_Delay(10);

  if (HAL_I2C_Master_Transmit(&MUX_I2C_HANDLE, SHT41_ADDR_8BIT, &cmd, 1, 100) != HAL_OK)
  {
    return -1;
  }

  HAL_Delay(10);

  if (HAL_I2C_Master_Receive(&MUX_I2C_HANDLE, SHT41_ADDR_8BIT, rx, 6, 100) != HAL_OK)
  {
    return -1;
  }

  if ((sht41_crc8(&rx[0], 2) != rx[2]) || (sht41_crc8(&rx[3], 2) != rx[5]))
  {
    return -1;
  }

  uint16_t raw_t = ((uint16_t)rx[0] << 8) | rx[1];
  uint16_t raw_h = ((uint16_t)rx[3] << 8) | rx[4];

  *temp_centi = -4500 + ((17500L * raw_t) / 65535L);
  *hum_centi = -600 + ((12500L * raw_h) / 65535L);

  if (*hum_centi < 0)
  {
    *hum_centi = 0;
  }

  if (*hum_centi > 10000)
  {
    *hum_centi = 10000;
  }

  return 0;
}

static void env_poll_state(void)
{
  int32_t sht_temp_centi = 0;
  int32_t sht_hum_centi = 0;
  int32_t bme_temp_centi = 0;
  int32_t bme_hum_centi = 0;
  uint8_t bme_valid = 0;

  if (!mux_addr_found)
  {
    return;
  }

  if (sht41_read_temp_humidity(mux_addr_found, &sht_temp_centi, &sht_hum_centi) != 0)
  {
    uart2_print("SHT41 read failed.\r\n");
    return;
  }

  latest_sht_temp_centi = sht_temp_centi;
  latest_sht_hum_centi = sht_hum_centi;

  if (bme280_read_temp_humidity(mux_addr_found, &bme_temp_centi, &bme_hum_centi) == 0)
  {
    bme_valid = 1;
    latest_bme_temp_centi = bme_temp_centi;
    latest_bme_hum_centi = bme_hum_centi;
  }

  if (bme_valid)
  {
    latest_temp_centi = (sht_temp_centi + bme_temp_centi) / 2;
    latest_hum_centi = (sht_hum_centi + bme_hum_centi) / 2;
  }
  else
  {
    latest_temp_centi = sht_temp_centi;
    latest_hum_centi = sht_hum_centi;
  }

  uint8_t alert_now = ((latest_temp_centi >= TEMP_ALERT_CENTI) ||
                       (latest_hum_centi >= HUM_ALERT_CENTI)) ? 1U : 0U;

  if (alert_now != env_alert_state)
  {
    env_alert_state = alert_now;

    if (env_alert_state)
    {
      uart2_printf("STATE CHANGE: ENV ALERT, AVG T=%ld.%02ld C AVG H=%ld.%02ld%%. Sent E.\r\n",
                   latest_temp_centi / 100, labs(latest_temp_centi % 100),
                   latest_hum_centi / 100, labs(latest_hum_centi % 100));
      zybo_send((uint8_t)CMD_ENV_ALERT);
    }
    else
    {
      uart2_printf("STATE CHANGE: ENV NORMAL, AVG T=%ld.%02ld C AVG H=%ld.%02ld%%. Sent e.\r\n",
                   latest_temp_centi / 100, labs(latest_temp_centi % 100),
                   latest_hum_centi / 100, labs(latest_hum_centi % 100));
      zybo_send((uint8_t)CMD_ENV_NORMAL);
    }
  }
}

/* ---------------- LSM6DS3 ---------------- */

static void lsm6ds3_cs_low(void)
{
  HAL_GPIO_WritePin(LSM6DS3_CS_PORT, LSM6DS3_CS_PIN, GPIO_PIN_RESET);

  for (volatile uint32_t i = 0; i < 100; i++)
  {
  }
}

static void lsm6ds3_cs_high(void)
{
  for (volatile uint32_t i = 0; i < 100; i++)
  {
  }

  HAL_GPIO_WritePin(LSM6DS3_CS_PORT, LSM6DS3_CS_PIN, GPIO_PIN_SET);
}

static uint8_t lsm6ds3_read_reg(uint8_t reg)
{
  uint8_t tx[2];
  uint8_t rx[2];

  tx[0] = reg | 0x80;
  tx[1] = 0x00;

  rx[0] = 0x00;
  rx[1] = 0x00;

  lsm6ds3_cs_low();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
  lsm6ds3_cs_high();

  return rx[1];
}

static void lsm6ds3_write_reg(uint8_t reg, uint8_t value)
{
  uint8_t tx[2];

  tx[0] = reg & 0x7F;
  tx[1] = value;

  lsm6ds3_cs_low();
  HAL_SPI_Transmit(&hspi1, tx, 2, 100);
  lsm6ds3_cs_high();
}

static int lsm6ds3_init(void)
{
  uint8_t id = 0;
  uint8_t ctrl1 = 0;
  uint8_t ctrl3 = 0;

  uart2_print("\r\n--- LSM6DS3 init ---\r\n");

  lsm6ds3_cs_high();
  HAL_Delay(50);

  id = lsm6ds3_read_reg(LSM6DS3_WHO_AM_I_REG);
  uart2_printf("LSM6DS3 WHO_AM_I = 0x%02X\r\n", id);

  if (id != LSM6DS3_EXPECTED_ID)
  {
    uart2_print("LSM6DS3 not detected.\r\n");
    lsm_detected = 0;
    return -1;
  }

  lsm6ds3_write_reg(LSM6DS3_CTRL3_C, 0x44);
  HAL_Delay(20);

  lsm6ds3_write_reg(LSM6DS3_CTRL1_XL, 0x40);
  HAL_Delay(100);

  ctrl1 = lsm6ds3_read_reg(LSM6DS3_CTRL1_XL);
  ctrl3 = lsm6ds3_read_reg(LSM6DS3_CTRL3_C);

  uart2_printf("CTRL1_XL readback = 0x%02X, expected 0x40\r\n", ctrl1);
  uart2_printf("CTRL3_C  readback = 0x%02X, expected 0x44\r\n", ctrl3);

  if ((ctrl1 != 0x40) || (ctrl3 != 0x44))
  {
    uart2_print("WARNING: LSM control register readback mismatch.\r\n");
    uart2_print("Check PD14 CS wiring and SPI mode if accel values are unstable.\r\n");
  }

  lsm_detected = 1;
  motion_baseline_valid = 0;
  motion_hit_count = 0;
  motion_quiet_count = 0;
  latest_motion_delta = 0;

  uart2_print("LSM6DS3 detected and accelerometer enabled.\r\n");

  return 0;
}

static void lsm6ds3_read_accel_raw(int16_t *ax, int16_t *ay, int16_t *az)
{
  uint8_t tx[7] = {0};
  uint8_t rx[7] = {0};

  tx[0] = LSM6DS3_OUTX_L_XL | 0x80;

  lsm6ds3_cs_low();
  HAL_SPI_TransmitReceive(&hspi1, tx, rx, 7, 100);
  lsm6ds3_cs_high();

  uint8_t xl = rx[1];
  uint8_t xh = rx[2];
  uint8_t yl = rx[3];
  uint8_t yh = rx[4];
  uint8_t zl = rx[5];
  uint8_t zh = rx[6];

  *ax = (int16_t)(((uint16_t)xh << 8) | xl);
  *ay = (int16_t)(((uint16_t)yh << 8) | yl);
  *az = (int16_t)(((uint16_t)zh << 8) | zl);
}

static void motion_poll_state(void)
{
  int16_t ax = 0;
  int16_t ay = 0;
  int16_t az = 0;

  if (!lsm_detected)
  {
    return;
  }

  lsm6ds3_read_accel_raw(&ax, &ay, &az);

  if (!motion_baseline_valid)
  {
    prev_ax = ax;
    prev_ay = ay;
    prev_az = az;
    motion_baseline_valid = 1;
    motion_hit_count = 0;
    motion_quiet_count = 0;
    latest_motion_delta = 0;
    return;
  }

  int32_t dx = labs((int32_t)ax - prev_ax);
  int32_t dy = labs((int32_t)ay - prev_ay);
  int32_t dz = labs((int32_t)az - prev_az);

  int32_t delta = dx + dy + dz;
  latest_motion_delta = delta;

  prev_ax = (int16_t)(((int32_t)prev_ax * 7 + ax) / 8);
  prev_ay = (int16_t)(((int32_t)prev_ay * 7 + ay) / 8);
  prev_az = (int16_t)(((int32_t)prev_az * 7 + az) / 8);

  if (delta > MOTION_DELTA_THRESHOLD)
  {
    if (motion_hit_count < 255)
    {
      motion_hit_count++;
    }

    motion_quiet_count = 0;
  }
  else
  {
    if (motion_quiet_count < 255)
    {
      motion_quiet_count++;
    }

    motion_hit_count = 0;
  }

  if ((motion_state == 0) && (motion_hit_count >= MOTION_HIT_COUNT_REQUIRED))
  {
    motion_state = 1;
    motion_hit_count = 0;
    motion_quiet_count = 0;

    uart2_printf("STATE CHANGE: MOTION DETECTED, delta=%ld. Sent M.\r\n", delta);
    zybo_send((uint8_t)CMD_MOTION_DETECTED);
  }

  if ((motion_state == 1) && (motion_quiet_count >= MOTION_QUIET_COUNT_REQUIRED))
  {
    motion_state = 0;
    motion_hit_count = 0;
    motion_quiet_count = 0;

    uart2_printf("STATE CHANGE: MOTION NORMAL, delta=%ld. Sent m.\r\n", delta);
    zybo_send((uint8_t)CMD_MOTION_NORMAL);
  }
}

/* ---------------- Debug status ---------------- */

static void print_system_status(void)
{
  uart2_print("\r\n--- System status ---\r\n");

  uart2_printf("Distance: %u mm | state=%s\r\n",
               latest_distance_mm,
               tof_near_state ? "NEAR" : "NORMAL");

  uart2_printf("Environment: T=%ld.%02ld C H=%ld.%02ld %% | state=%s\r\n",
               latest_temp_centi / 100, labs(latest_temp_centi % 100),
               latest_hum_centi / 100, labs(latest_hum_centi % 100),
               env_alert_state ? "ALERT" : "NORMAL");

  uart2_printf("Motion: state=%s | delta=%ld | hits=%u | quiet=%u\r\n",
               motion_state ? "MOTION" : "NORMAL",
               latest_motion_delta,
               motion_hit_count,
               motion_quiet_count);

  uart2_printf("OLED: CH2=%u CH3=%u CH4=%u CH5=%u | Last CMD=%c\r\n",
               oled1_ok,
               oled2_ok,
               oled3_ok,
               oled4_ok,
               (char)latest_zybo_cmd);
}

/* USER CODE END 0 */

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C4_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();

  BSP_LED_Init(LED_GREEN);
  BSP_LED_Init(LED_YELLOW);
  BSP_LED_Init(LED_RED);

  BSP_LED_Off(LED_GREEN);
  BSP_LED_Off(LED_YELLOW);
  BSP_LED_Off(LED_RED);

  lsm6ds3_cs_high();
  tof_xshut_low();

  HAL_Delay(300);

  uart2_print("\r\n================================\r\n");
  uart2_print("SENSOR STATE MACHINE + OLED START\r\n");
  uart2_print("USART2 debug @ 115200\r\n");
  uart2_print("USART3 sends commands to Zybo\r\n");
  uart2_print("ToF: I2C2, XSHUT=PB9, INT=PC6\r\n");
  uart2_print("LSM6DS3: SPI1, CS=PD14\r\n");
  uart2_print("OLEDs: I2C4 mux CH2-CH5, SSD1306 0x3C\r\n");
  uart2_print("Commands: D/N=distance, E/e=environment, M/m=motion, C=clear, H=heartbeat\r\n");
  uart2_print("================================\r\n");

  scan_i2c_bus(&hi2c2, "I2C2 - ToF bus");
  scan_i2c_bus(&hi2c4, "I2C4 - Mux bus");

  mux_addr_found = find_mux_on_i2c4();

  if (mux_addr_found)
  {
    scan_mux_channel(mux_addr_found, BME280_MUX_CH, "CH0 - BME280");
    scan_mux_channel(mux_addr_found, SHT41_MUX_CH, "CH1 - SHT41");
    scan_mux_channel(mux_addr_found, OLED1_MUX_CH, "CH2 - OLED1");
    scan_mux_channel(mux_addr_found, OLED2_MUX_CH, "CH3 - OLED2");
    scan_mux_channel(mux_addr_found, OLED3_MUX_CH, "CH4 - OLED3");
    scan_mux_channel(mux_addr_found, OLED4_MUX_CH, "CH5 - OLED4");

    if (bme280_init_sensor(mux_addr_found) != 0)
    {
      uart2_print("WARNING: BME280 temp/humidity init failed. Using SHT41 only.\r\n");
    }

    mux_disable_all(mux_addr_found);
  }

  oled_init_all_displays();

  if (tof_init_sensor() != 0)
  {
    uart2_print("WARNING: ToF init failed.\r\n");
  }

  if (lsm6ds3_init() != 0)
  {
    uart2_print("WARNING: LSM6DS3 init failed.\r\n");
  }

  env_poll_state();
  oled_update_all_displays();

  zybo_send((uint8_t)CMD_CLEAR_ALL);
  uart2_print("Sent C to Zybo: clear all states.\r\n");

  zybo_send_heartbeat();
  uart2_print("Sent H to Zybo: heartbeat initialized.\r\n");

  uint32_t last_tof_poll = 0;
  uint32_t last_env_poll = 0;
  uint32_t last_motion_poll = 0;
  uint32_t last_oled_update = 0;
  uint32_t last_status_print = 0;
  uint32_t last_heartbeat_tx = HAL_GetTick();

  while (1)
  {
    uint32_t now = HAL_GetTick();

    if ((now - last_heartbeat_tx) >= HEARTBEAT_TX_MS)
    {
      last_heartbeat_tx = now;
      zybo_send_heartbeat();
    }

    if ((now - last_tof_poll) >= TOF_POLL_MS)
    {
      last_tof_poll = now;
      tof_poll_state();
    }

    if ((now - last_env_poll) >= ENV_POLL_MS)
    {
      last_env_poll = now;
      env_poll_state();

      if (mux_addr_found)
      {
        mux_disable_all(mux_addr_found);
      }
    }

    if ((now - last_motion_poll) >= MOTION_POLL_MS)
    {
      last_motion_poll = now;
      motion_poll_state();
    }

    if ((now - last_oled_update) >= OLED_UPDATE_MS)
    {
      last_oled_update = now;
      oled_update_all_displays();
    }

    if ((now - last_status_print) >= STATUS_PRINT_MS)
    {
      last_status_print = now;
      BSP_LED_Toggle(LED_YELLOW);
      print_system_status();
    }
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 9;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 1;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOMEDIUM;
  RCC_OscInitStruct.PLL.PLLFRACN = 3072;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                              | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2
                              | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C2_Init(void)
{
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10707DBC;
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

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C4_Init(void)
{
  hi2c4.Instance = I2C4;
  hi2c4.Init.Timing = 0x10707DBC;
  hi2c4.Init.OwnAddress1 = 0;
  hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c4.Init.OwnAddress2 = 0;
  hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

  if (HAL_I2C_Init(&hi2c4) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*
   * Initial output states before GPIO init.
   * PD14 = LSM CS must idle HIGH.
   * PB9 = ToF XSHUT starts LOW for reset.
   */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /* Optional project output */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

  /* PB9 = ToF XSHUT output */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* PC6 = ToF INT input */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* PD14 = LSM6DS3 CS output, idle HIGH */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* PA15 optional output */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();

  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
