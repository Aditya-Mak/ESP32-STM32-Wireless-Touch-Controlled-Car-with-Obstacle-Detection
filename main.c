/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Dual IBT-2 motor control + B1 toggle
  *                   MODE_DRIVE: touch/I2C forward/back
  *                   MODE_ULTRASONIC: go straight; stop <=ULTRA_STOP_CM; resume >=ULTRA_RESUME_CM
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint16_t x;
  uint16_t y;
  uint8_t  valid;
  uint8_t  touching;
} touch_sample_t;

typedef enum {
  DRIVE_STOP = 0,
  DRIVE_FORWARD,
  DRIVE_BACKWARD,
  DRIVE_LEFT,
  DRIVE_RIGHT
} drive_mode_t;

typedef enum { MODE_DRIVE = 0, MODE_ULTRASONIC = 1 } AppMode;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* ---- Ultrasonic pin macros (fallback if not defined in gpio.h) ---- */
#ifndef Front_Trig_Pin
  #define Front_Trig_Pin   GPIO_PIN_0
  #define Front_Trig_Port  GPIOA
  #define Front_Echo_Pin   GPIO_PIN_1
  #define Front_Echo_Port  GPIOA

  #define Right_Trig_Pin   GPIO_PIN_5
  #define Right_Trig_Port  GPIOC
  #define Right_Echo_Pin   GPIO_PIN_2
  #define Right_Echo_Port  GPIOC

  #define Left_Trig_Pin    GPIO_PIN_0
  #define Left_Trig_Port   GPIOC
  #define Left_Echo_Pin    GPIO_PIN_1
  #define Left_Echo_Port   GPIOC
#endif

#define TOUCH_ADDR_7BIT  0x28
#define TOUCH_ADDR_8BIT  (TOUCH_ADDR_7BIT << 1)

/* PWM / motor mapping */
#define PWM_PERIOD        1000

/* Motor 1 (MC1) -> TIM3 on PA6/PA7 */
#define PWM1_TIMER        (&htim3)
#define RPWM1_CH          TIM_CHANNEL_1
#define LPWM1_CH          TIM_CHANNEL_2
#define R_EN1_Port        GPIOB
#define R_EN1_Pin         GPIO_PIN_10
#define L_EN1_Port        GPIOB
#define L_EN1_Pin         GPIO_PIN_4

/* Motor 2 (MC2) -> TIM1 on PA8/PA9 */
#define PWM2_TIMER        (&htim1)
#define RPWM2_CH          TIM_CHANNEL_1
#define LPWM2_CH          TIM_CHANNEL_2
#define R_EN2_Port        GPIOB
#define R_EN2_Pin         GPIO_PIN_5
#define L_EN2_Port        GPIOB
#define L_EN2_Pin         GPIO_PIN_0

/* Logical direction inversion if one side is flipped */
#define LEFT_INVERT   1
#define RIGHT_INVERT  0

/* Touch calibration points */
#define PT_RIGHT_TOP_X   530u
#define PT_RIGHT_TOP_Y   1220u
#define PT_LEFT_TOP_X    1350u
#define PT_LEFT_TOP_Y    1370u
#define PT_RIGHT_DOWN_X  670u
#define PT_RIGHT_DOWN_Y  847u
#define PT_LEFT_DOWN_X   1255u
#define PT_LEFT_DOWN_Y   750u

#define NEAR_RADIUS_PX   300u
#define NEAR_RADIUS2     ((uint32_t)NEAR_RADIUS_PX * (uint32_t)NEAR_RADIUS_PX)
#define HYSTERESIS_PX    30u
#define HYSTERESIS2      ((uint32_t)HYSTERESIS_PX * (uint32_t)HYSTERESIS_PX)

/* Base speed (~80% duty). Increase/decrease to change speed. */
#define DUTY_CONST       (PWM_PERIOD * 5 / 10)

/* Ultrasonic behavior (use hysteresis: resume must be > stop) */
#define ULTRA_STOP_CM     30u   /* stop when <= this */
#define ULTRA_RESUME_CM   35u   /* resume when >= this */

/* ---- Direction-aware wheel trim (percent) ----
   Apply ONLY during FORWARD to fix right drift; keep reverse as-is.
   Positive speeds that side up; negative slows it down. */
#define FWD_LEFT_TRIM_PCT     (0)    /* keep left baseline */
#define FWD_RIGHT_TRIM_PCT    (+12)   /* boost right ~8% to correct right drift */

#define REV_LEFT_TRIM_PCT     (0)    /* reverse was straight—leave 0 */
#define REV_RIGHT_TRIM_PCT    (0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
static inline uint16_t clamp16(int32_t v){ if (v < 0) return 0; if (v > PWM_PERIOD) return PWM_PERIOD; return (uint16_t)v; }

/* Forward trims */
static inline uint16_t scale_left_fwd(uint16_t duty){  return clamp16((int32_t)duty * (100 + FWD_LEFT_TRIM_PCT)  / 100); }
static inline uint16_t scale_right_fwd(uint16_t duty){ return clamp16((int32_t)duty * (100 + FWD_RIGHT_TRIM_PCT) / 100); }

/* Reverse trims (0 by default) */
static inline uint16_t scale_left_rev(uint16_t duty){  return clamp16((int32_t)duty * (100 + REV_LEFT_TRIM_PCT)  / 100); }
static inline uint16_t scale_right_rev(uint16_t duty){ return clamp16((int32_t)duty * (100 + REV_RIGHT_TRIM_PCT) / 100); }
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern UART_HandleTypeDef huart2;

volatile AppMode g_mode = MODE_DRIVE;
volatile uint8_t mode_changed = 0;

static uint32_t last_fresh_frame_ms = 0;
static touch_sample_t last_frame = { .x=0xFFFF, .y=0xFFFF, .valid=0, .touching=0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static inline void mc1_enable(void);
static inline void mc1_disable(void);
static inline void mc1_brake(void);
static inline void mc1_forward(uint16_t duty);
static inline void mc1_reverse(uint16_t duty);

static inline void mc2_enable(void);
static inline void mc2_disable(void);
static inline void mc2_brake(void);
static inline void mc2_forward(uint16_t duty);
static inline void mc2_reverse(uint16_t duty);

static inline void left_forward(uint16_t duty);
static inline void left_backward(uint16_t duty);
static inline void right_forward(uint16_t duty);
static inline void right_backward(uint16_t duty);
static inline void motors_stop(void);

static uint8_t touch_crc6(const uint8_t *f);
static touch_sample_t read_touch(void);
static inline uint32_t dist2_u16(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);
static drive_mode_t decide_mode_4pt(uint16_t x, uint16_t y);

static void delay_us(uint16_t us);
static uint16_t measure_distance(GPIO_TypeDef *trigPort, uint16_t trigPin,
                                 GPIO_TypeDef *echoPort, uint16_t echoPin);
static void run_ultrasonic_mode(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* ===== Motor helpers ===== */
static inline void mc1_enable(void){
  HAL_GPIO_WritePin(R_EN1_Port, R_EN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(L_EN1_Port, L_EN1_Pin, GPIO_PIN_SET);
}
static inline void mc1_disable(void){
  HAL_GPIO_WritePin(R_EN1_Port, R_EN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L_EN1_Port, L_EN1_Pin, GPIO_PIN_RESET);
}
static inline void mc1_brake(void){
  __HAL_TIM_SET_COMPARE(PWM1_TIMER, RPWM1_CH, 0);
  __HAL_TIM_SET_COMPARE(PWM1_TIMER, LPWM1_CH, 0);
}
static inline void mc1_forward(uint16_t duty){
  if (duty > PWM_PERIOD) duty = PWM_PERIOD;
  __HAL_TIM_SET_COMPARE(PWM1_TIMER, RPWM1_CH, duty);
  __HAL_TIM_SET_COMPARE(PWM1_TIMER, LPWM1_CH, 0);
}
static inline void mc1_reverse(uint16_t duty){
  if (duty > PWM_PERIOD) duty = PWM_PERIOD;
  __HAL_TIM_SET_COMPARE(PWM1_TIMER, RPWM1_CH, 0);
  __HAL_TIM_SET_COMPARE(PWM1_TIMER, LPWM1_CH, duty);
}

static inline void mc2_enable(void){
  HAL_GPIO_WritePin(R_EN2_Port, R_EN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(L_EN2_Port, L_EN2_Pin, GPIO_PIN_SET);
}
static inline void mc2_disable(void){
  HAL_GPIO_WritePin(R_EN2_Port, R_EN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(L_EN2_Port, L_EN2_Pin, GPIO_PIN_RESET);
}
static inline void mc2_brake(void){
  __HAL_TIM_SET_COMPARE(PWM2_TIMER, RPWM2_CH, 0);
  __HAL_TIM_SET_COMPARE(PWM2_TIMER, LPWM2_CH, 0);
}
static inline void mc2_forward(uint16_t duty){
  if (duty > PWM_PERIOD) duty = PWM_PERIOD;
  __HAL_TIM_SET_COMPARE(PWM2_TIMER, RPWM2_CH, duty);
  __HAL_TIM_SET_COMPARE(PWM2_TIMER, LPWM2_CH, 0);
}
static inline void mc2_reverse(uint16_t duty){
  if (duty > PWM_PERIOD) duty = PWM_PERIOD;
  __HAL_TIM_SET_COMPARE(PWM2_TIMER, RPWM2_CH, 0);
  __HAL_TIM_SET_COMPARE(PWM2_TIMER, LPWM2_CH, duty);
}

/* Logical wheel directions */
static inline void left_forward(uint16_t duty){
  #if LEFT_INVERT
    mc1_reverse(duty);
  #else
    mc1_forward(duty);
  #endif
}
static inline void left_backward(uint16_t duty){
  #if LEFT_INVERT
    mc1_forward(duty);
  #else
    mc1_reverse(duty);
  #endif
}
static inline void right_forward(uint16_t duty){
  #if RIGHT_INVERT
    mc2_reverse(duty);
  #else
    mc2_forward(duty);
  #endif
}
static inline void right_backward(uint16_t duty){
  #if RIGHT_INVERT
    mc2_forward(duty);
  #else
    mc2_reverse(duty);
  #endif
}

static inline void motors_stop(void){
  mc1_brake();
  mc2_brake();
  mc1_disable();
  mc2_disable();
}

/* ===== Touch ===== */
static uint8_t touch_crc6(const uint8_t *f) {
  uint16_t s = 0;
  for (int i = 0; i < 5; ++i) s += f[i];
  return (uint8_t)(s & 0xFF);
}

static touch_sample_t read_touch(void)
{
  uint8_t f[6] = {0};
  touch_sample_t out = { .x=0xFFFF, .y=0xFFFF, .valid=0, .touching=0 };

  if (HAL_I2C_Master_Receive(&hi2c1, TOUCH_ADDR_8BIT, f, sizeof(f), 10) != HAL_OK)
    return out;
  if (f[0] != 0xAA) return out;
  if (touch_crc6(f) != f[5]) return out;

  uint16_t x = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
  uint16_t y = (uint16_t)f[3] | ((uint16_t)f[4] << 8);

  out.valid = 1;
  out.x = x;
  out.y = y;
  out.touching = !((x == 0xFFFF) && (y == 0xFFFF));
  return out;
}

typedef struct { uint16_t x, y; drive_mode_t mode; } touch_target_t;
static const touch_target_t kTargets[4] = {
  { PT_RIGHT_TOP_X,  PT_RIGHT_TOP_Y,  DRIVE_RIGHT    }, // top-right -> turn right
  { PT_RIGHT_DOWN_X, PT_RIGHT_DOWN_Y, DRIVE_BACKWARD }, // bottom-right -> reverse
  { PT_LEFT_TOP_X,   PT_LEFT_TOP_Y,   DRIVE_FORWARD  }, // top-left -> forward
  { PT_LEFT_DOWN_X,  PT_LEFT_DOWN_Y,  DRIVE_LEFT     }  // bottom-left -> turn left
};

static inline uint32_t dist2_u16(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  int32_t dx = (int32_t)x1 - (int32_t)x2;
  int32_t dy = (int32_t)y1 - (int32_t)y2;
  return (uint32_t)(dx*dx + dy*dy);
}

static drive_mode_t decide_mode_4pt(uint16_t x, uint16_t y)
{
  static drive_mode_t last_mode = DRIVE_STOP;
  static uint16_t last_cx = 0, last_cy = 0;

  uint32_t best_d2 = 0xFFFFFFFFu;
  int best_idx = -1;
  for (int i = 0; i < 4; ++i) {
    uint32_t d2 = dist2_u16(x, y, kTargets[i].x, kTargets[i].y);
    if (d2 < best_d2) { best_d2 = d2; best_idx = i; }
  }

  if (best_idx < 0 || best_d2 > NEAR_RADIUS2) {
    last_mode = DRIVE_STOP;
    return DRIVE_STOP;
  }

  if (last_mode != DRIVE_STOP) {
    uint32_t d2_prev = dist2_u16(x, y, last_cx, last_cy);
    if (d2_prev <= (best_d2 + HYSTERESIS2)) {
      return last_mode; // stick with previous mode
    }
  }

  last_mode = kTargets[best_idx].mode;
  last_cx   = kTargets[best_idx].x;
  last_cy   = kTargets[best_idx].y;
  return last_mode;
}

/* ===== Microsecond delay & ultrasonic measure ===== */
static void delay_us(uint16_t us)
{
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  while (__HAL_TIM_GET_COUNTER(&htim2) < us);
}

static uint16_t measure_distance(GPIO_TypeDef *trigPort, uint16_t trigPin,
                                 GPIO_TypeDef *echoPort, uint16_t echoPin)
{
  uint32_t startTick;
  uint32_t t1 = 0, t2 = 0;

  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);
  delay_us(2);
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_SET);
  delay_us(10);
  HAL_GPIO_WritePin(trigPort, trigPin, GPIO_PIN_RESET);

  startTick = HAL_GetTick();
  while (!HAL_GPIO_ReadPin(echoPort, echoPin)) {
    if ((HAL_GetTick() - startTick) > 10) return 0; // timeout -> invalid
  }
  t1 = __HAL_TIM_GET_COUNTER(&htim2);

  startTick = HAL_GetTick();
  while (HAL_GPIO_ReadPin(echoPort, echoPin)) {
    if ((HAL_GetTick() - startTick) > 50) return 0; // timeout -> invalid
  }
  t2 = __HAL_TIM_GET_COUNTER(&htim2);

  uint32_t dt = (t2 >= t1) ? (t2 - t1) : (65535u - t1 + t2);
  /* distance cm ≈ (dt * 0.034) / 2  -> dt*34/2000 */
  return (uint16_t)((dt * 34u) / 2000u);
}

/* ===== ULTRASONIC MODE: go straight; stop <=ULTRA_STOP_CM; resume >=ULTRA_RESUME_CM ===== */
static void run_ultrasonic_mode(void)
{
  static uint8_t moving = 0;  // 0=stopped, 1=forward
  uint16_t front = measure_distance(Front_Trig_Port, Front_Trig_Pin, Front_Echo_Port, Front_Echo_Pin);
  HAL_Delay(30); // small spacing for sensor stability

  // Optional: side sensors (printed; not used for control)
  uint16_t right = measure_distance(Right_Trig_Port, Right_Trig_Pin, Right_Echo_Port, Right_Echo_Pin);
  HAL_Delay(30);
  uint16_t left  = measure_distance(Left_Trig_Port,  Left_Trig_Pin,  Left_Echo_Port,  Left_Echo_Pin);

  // Safety: invalid read -> stop
  if (front == 0) {
    motors_stop();
    moving = 0;
  } else {
    if (moving) {
      if (front <= ULTRA_STOP_CM) {
        motors_stop();
        moving = 0;
      } else {
        // keep moving forward WITH FORWARD-ONLY TRIM
        mc1_enable(); mc2_enable();
        left_forward(  scale_left_fwd(DUTY_CONST)  );
        right_forward( scale_right_fwd(DUTY_CONST) );
      }
    } else {
      if (front >= ULTRA_RESUME_CM) {
        mc1_enable(); mc2_enable();
        left_forward(  scale_left_fwd(DUTY_CONST)  );
        right_forward( scale_right_fwd(DUTY_CONST) );
        moving = 1;
      } else {
        motors_stop();
      }
    }
  }

  // Status print
  char msg[96];
  int len = snprintf(msg, sizeof(msg),
                     "Front=%u cm  Right=%u cm  Left=%u cm  MOVE=%u\r\n",
                     front, right, left, (unsigned)moving);
  HAL_UART_Transmit(&huart2, (uint8_t*)msg, len, HAL_MAX_DELAY);

  HAL_Delay(70); // total ~130ms loop for ULTRA mode
}

/* ===== EXTI callback for B1 toggle ===== */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_13) {
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last) > 80) {  // debounce ~80 ms
      g_mode = (g_mode == MODE_DRIVE) ? MODE_ULTRASONIC : MODE_DRIVE;
      mode_changed = 1;
      motors_stop(); // safety stop immediately on toggle
      const char *m = (g_mode == MODE_DRIVE) ? "MODE=DRIVE\r\n" : "MODE=ULTRASONIC\r\n";
      HAL_UART_Transmit(&huart2, (uint8_t*)m, strlen(m), 10);
    }
    last = now;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();

  /* Start PWMs and TIM2 base for µs timing */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start(&htim2);

  /* Ensure motor driver enables low at boot */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_0, GPIO_PIN_RESET);

  /* Infinite loop */
  while (1)
  {
    if (mode_changed) {
      mode_changed = 0;
      motors_stop();
    }

    if (g_mode == MODE_DRIVE)
    {
      /* ===== Drive logic (I2C/touch) ===== */
      touch_sample_t s = read_touch();
      uint32_t now = HAL_GetTick();

      bool fresh = false;
      if (s.valid) {
        if (s.x != last_frame.x || s.y != last_frame.y) {
          fresh = true;
          last_fresh_frame_ms = now;
          last_frame = s;
        } else {
          if ((now - last_fresh_frame_ms) <= 80) {
            fresh = true;
          }
        }
      }

      if (!fresh || (now - last_fresh_frame_ms) > 120) {
        motors_stop();
        HAL_Delay(10);
        continue;
      }

      uint16_t x = last_frame.x, y = last_frame.y;
      drive_mode_t dmode = decide_mode_4pt(x, y);

      mc1_enable();
      mc2_enable();

      switch (dmode) {
        case DRIVE_FORWARD:
          left_forward(  scale_left_fwd(DUTY_CONST)  );
          right_forward( scale_right_fwd(DUTY_CONST) );
          break;
        case DRIVE_BACKWARD:
          left_backward(  scale_left_rev(DUTY_CONST)  );
          right_backward( scale_right_rev(DUTY_CONST) );
          break;
        case DRIVE_LEFT:
          left_backward(  scale_left_rev(DUTY_CONST)  );
          right_forward( scale_right_rev(DUTY_CONST) );
          break;
        case DRIVE_RIGHT:
           left_forward(  scale_left_rev(DUTY_CONST)  );
           right_backward( scale_right_rev(DUTY_CONST) );
          break;
         default:
          motors_stop();
          break;
      }

      // Optional: debug print on fresh frames
      if (fresh) {
        char dbg[64];
        uint16_t duty_print = (dmode == DRIVE_STOP) ? 0 : DUTY_CONST;
        int len = snprintf(dbg, sizeof(dbg), "X=%u Y=%u mode=%d duty=%u\r\n",
                           x, y, (int)dmode, (unsigned)duty_print);
        HAL_UART_Transmit(&huart2, (uint8_t*)dbg, len, 10);
      }

      HAL_Delay(10);
    }
    else  /* MODE_ULTRASONIC */
    {
      run_ultrasonic_mode();
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

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) { Error_Handler(); }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) { Error_Handler(); }
}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) { }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
}
#endif /* USE_FULL_ASSERT */
