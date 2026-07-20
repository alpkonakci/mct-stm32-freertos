/*
 * STM32F103C8T6 + DRV8876 PH/EN motor node - native FreeRTOS version
 *
 * Hardware:
 *   PA11 CAN_RX, PA12 CAN_TX, 500 kbit/s
 *   PA3  ADC1_IN3, DRV8876 IPROPI through 2.2k to GND
 *   PA6  DRV8876 nFAULT, active low
 *   PB1  TIM3_CH4, DRV8876 EN PWM
 *   PB10 DRV8876 PH direction
 *   PB11 DRV8876 nSLEEP
 *   PMODE is tied to GND: PH/EN mode
 *
 * FreeRTOS ownership:
 *   MotorControlTask (priority 5): sole owner of motor state and GPIO/PWM
 *   CanTxTask       (priority 4): sole owner of CAN transmission/recovery
 *   TelemetryTask   (priority 2): fixed 500 ms telemetry producer
 *
 * CAN IDs are unchanged:
 *   RX  0x101 command, 0x202 sensor telemetry
 *   TX  0x181 ACK, 0x201 basic, 0x203 speed, 0x204 counters,
 *       0x211 status, 0x221 fault
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

ADC_HandleTypeDef hadc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim3;
static TIM_HandleTypeDef htim2HalTick;

#define DRV_EN_Pin                         GPIO_PIN_1
#define DRV_EN_GPIO_Port                   GPIOB
#define DRV_PH_Pin                         GPIO_PIN_10
#define DRV_PH_GPIO_Port                   GPIOB
#define DRV_NSLEEP_Pin                     GPIO_PIN_11
#define DRV_NSLEEP_GPIO_Port               GPIOB
#define DRV_NFAULT_Pin                     GPIO_PIN_6
#define DRV_NFAULT_GPIO_Port               GPIOA

#define ADC_VREF_MV                        3300U
#define ADC_MAX_COUNTS                     4095U
#define RIPROPI_OHMS                       2200U
#define AIPROPI_UA_PER_A                   1000U

#define CURRENT_LIMIT_MA                   500U
#define OVERCURRENT_STOP_MS                1000U
#define DRIVER_FAULT_IGNORE_AFTER_WAKE_MS  100U
#define DRIVER_FAULT_DEBOUNCE_MS           50U
#define CONTROL_PERIOD_MS                  5U
#define TELEMETRY_PERIOD_MS                500U

#define SENSOR_VIB_STOP_PCT                80U
#define SENSOR_TEMP_STOP_X100              4000

#define CAN_ID_CMD_RX                      0x101U
#define CAN_ID_ACK_TX                      0x181U
#define CAN_ID_TELEM_TX                    0x201U
#define CAN_ID_SPEED_TX                    0x203U
#define CAN_ID_COUNTER_TX                  0x204U
#define CAN_ID_STATUS_TX                   0x211U
#define CAN_ID_FAULT_TX                    0x221U
#define SENSOR_TELEM_ID_RX                 0x202U

#define START_MOTOR_ON_BOOT                1
#define MOTOR_FORWARD_ON_BOOT              1
#define MOTOR_DEFAULT_SPEED_PERCENT        80U
#define MOTOR_SPEED_STEP_PERCENT           20U

#define PWM_TIMER_CHANNEL                  TIM_CHANNEL_4
#define PWM_TIMER_PERIOD                   3599U
#define PWM_TIMER_COUNTS                   (PWM_TIMER_PERIOD + 1U)

#define CAN_RX_QUEUE_LENGTH                16U
#define CAN_TX_QUEUE_LENGTH                16U
#define CAN_TX_RETRY_LIMIT                 2U
#define CAN_MAILBOX_WAIT_MS                3U
#define CAN_TX_DONE_WAIT_MS                10U
#define CAN_RESTART_COOLDOWN_MS            100U
#define CAN_ESR_BOFF_FLAG                  0x00000004U

#define MOTOR_TASK_PRIORITY                5U
#define CAN_TX_TASK_PRIORITY               4U
#define TELEMETRY_TASK_PRIORITY            2U
#define MOTOR_TASK_STACK_WORDS             320U
#define CAN_TX_TASK_STACK_WORDS            256U
#define TELEMETRY_TASK_STACK_WORDS         192U

#define SAT_U16_INC(x) do { if ((x) < UINT16_MAX) { (x)++; } } while (0)

typedef enum {
  MOTOR_STOPPED = 0,
  MOTOR_FORWARD = 1,
  MOTOR_REVERSE = 2
} MotorState_t;

typedef enum {
  DIR_STOPPED = 0,
  DIR_FORWARD = 1,
  DIR_REVERSE = 2
} MotorDirection_t;

typedef enum {
  FAULT_NONE = 0,
  FAULT_OVERCURRENT = 1,
  FAULT_DRIVER = 2,
  FAULT_SENSOR_VIBRATION = 3,
  FAULT_REMOTE = 4,
  FAULT_TEST = 5,
  FAULT_SENSOR_TEMPERATURE = 6
} FaultCode_t;

typedef enum {
  CMD_NONE = 0,
  CMD_A,
  CMD_B,
  CMD_X,
  CMD_D,
  CMD_R,
  CMD_T,
  CMD_F,
  CMD_I,
  CMD_S,
  CMD_P,
  CMD_SPEED_UP,
  CMD_SPEED_DOWN
} Command_t;

typedef struct {
  uint16_t current_mA;
  MotorState_t motorState;
  MotorDirection_t direction;
  FaultCode_t faultCode;
  bool driverFault;
  bool overcurrentActive;
  bool motorStoppedByFault;
  bool testMode;
  uint8_t speedPercent;
  uint8_t targetSpeedPercent;
  uint8_t pwmDutyPercent;
  uint16_t pwmDutyRaw;
  uint32_t runTimeMs;
  uint16_t startStopCount;
  uint16_t faultCount;
  uint32_t testStartMs;
  uint32_t lastRunTickMs;
} MotorContext_t;

typedef struct {
  uint16_t stdId;
  uint8_t dlc;
  uint8_t data[8];
} CanFrame_t;

typedef struct {
  CanFrame_t frame;
  uint8_t retries;
  bool urgent;
} CanTxItem_t;

typedef struct {
  Command_t id;
  uint8_t value;
  bool hasValue;
} CommandEntry_t;

static MotorContext_t motor = {
  .current_mA = 0U,
  .motorState = MOTOR_STOPPED,
  .direction = DIR_STOPPED,
  .faultCode = FAULT_NONE,
  .driverFault = false,
  .overcurrentActive = false,
  .motorStoppedByFault = false,
  .testMode = false,
  .speedPercent = 0U,
  .targetSpeedPercent = MOTOR_DEFAULT_SPEED_PERCENT,
  .pwmDutyPercent = 0U,
  .pwmDutyRaw = 0U,
  .runTimeMs = 0U,
  .startStopCount = 0U,
  .faultCount = 0U,
  .testStartMs = 0U,
  .lastRunTickMs = 0U
};

static uint32_t overcurrentStartMs;
static uint32_t driverFaultIgnoreUntilMs;
static uint32_t driverFaultLowStartMs;
static uint32_t lastCanRestartMs;

static volatile uint32_t dbgCanRxQueued;
static volatile uint32_t dbgCanRxDropped;
static volatile uint32_t dbgCanTxQueued;
static volatile uint32_t dbgCanTxDropped;
static volatile uint32_t dbgCanTxOk;
static volatile uint32_t dbgCanTxFail;
static volatile uint32_t dbgCanLastError;
static volatile uint32_t dbgCanLastTxId;

static QueueHandle_t canRxQueue;
static QueueHandle_t canTxQueue;
static QueueHandle_t motorSnapshotQueue;

static StaticQueue_t canRxQueueControl;
static StaticQueue_t canTxQueueControl;
static StaticQueue_t motorSnapshotQueueControl;
static uint8_t canRxQueueStorage[CAN_RX_QUEUE_LENGTH * sizeof(CanFrame_t)];
static uint8_t canTxQueueStorage[CAN_TX_QUEUE_LENGTH * sizeof(CanTxItem_t)];
static uint8_t motorSnapshotQueueStorage[sizeof(MotorContext_t)];

static TaskHandle_t motorTaskHandle;
static TaskHandle_t canTxTaskHandle;
static TaskHandle_t telemetryTaskHandle;
static StaticTask_t motorTaskControl;
static StaticTask_t canTxTaskControl;
static StaticTask_t telemetryTaskControl;
static StackType_t motorTaskStack[MOTOR_TASK_STACK_WORDS];
static StackType_t canTxTaskStack[CAN_TX_TASK_STACK_WORDS];
static StackType_t telemetryTaskStack[TELEMETRY_TASK_STACK_WORDS];

static StaticTask_t idleTaskControl;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM3_Init(void);
static void CAN_ConfigFilters(void);
static void RTOS_CreateObjects(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer);

static void MotorControlTask(void *argument);
static void CanTxTask(void *argument);
static void TelemetryTask(void *argument);

static uint16_t ADC_ReadCurrentRaw(void);
static uint16_t Current_mA_FromRaw(uint16_t raw);
static uint8_t ClampPercent(uint8_t percent);
static void PWM_SetComparePercent(uint8_t percent);
static void Motor_DriveBrake(void);
static void Motor_SelectDirection(MotorDirection_t direction);
static void Motor_Wake(void);
static void Motor_Start(MotorDirection_t direction);
static void Motor_StopBrake(void);
static void Motor_SetSpeedPercent(uint8_t percent);
static void Motor_AdjustSpeedPercent(int8_t delta);
static void Motor_ApplyTargetSpeed(void);
static void Motor_LatchFault(FaultCode_t fault);
static void Motor_ResetFaults(void);
static void Motor_UpdateSafety(uint32_t nowMs);
static void Motor_UpdateCurrent(uint32_t nowMs);
static void Motor_UpdateDriverFault(uint32_t nowMs);
static void Motor_UpdateTestFault(uint32_t nowMs);
static void Motor_UpdateRuntime(uint32_t nowMs);
static void Motor_PublishSnapshot(void);

static char ToUpperAscii(char value);
static bool Payload_HasTextPrefix(const uint8_t *payload, uint8_t dlc,
                                  const char *prefix);
static Command_t Command_FromPayload(const uint8_t *payload, uint8_t dlc);
static char Command_ToChar(Command_t command);
static CommandEntry_t Command_Decode(const CanFrame_t *frame);
static void Command_Process(CommandEntry_t entry);
static void Control_HandleRxFrame(const CanFrame_t *frame);

static bool CAN_QueueFrame(uint16_t stdId, const uint8_t *data, uint8_t dlc,
                           bool urgent);
static bool CAN_EnsureStarted(void);
static bool CAN_TransmitFrame(const CanFrame_t *frame);
static void CAN_SendAck(Command_t command, bool accepted, uint8_t reason);
static void CAN_SendFault(FaultCode_t fault);
static void CAN_SendStatusFrom(const MotorContext_t *snapshot, bool urgent);
static void CAN_SendBasicFrom(const MotorContext_t *snapshot);
static void CAN_SendSpeedFrom(const MotorContext_t *snapshot, bool urgent);
static void CAN_SendCountersFrom(const MotorContext_t *snapshot, bool urgent);
static void CAN_NotifyTxTaskFromISR(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_CAN_Init();

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Start(&htim3, PWM_TIMER_CHANNEL) != HAL_OK) {
    Error_Handler();
  }

  __HAL_TIM_SET_COMPARE(&htim3, PWM_TIMER_CHANNEL, 0U);
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_RESET);

  CAN_ConfigFilters();
  RTOS_CreateObjects();

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(
        &hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
    Error_Handler();
  }

  vTaskStartScheduler();
  Error_Handler();
  return 0;
}

static void RTOS_CreateObjects(void)
{
  canRxQueue = xQueueCreateStatic(
    CAN_RX_QUEUE_LENGTH, sizeof(CanFrame_t), canRxQueueStorage,
    &canRxQueueControl);
  canTxQueue = xQueueCreateStatic(
    CAN_TX_QUEUE_LENGTH, sizeof(CanTxItem_t), canTxQueueStorage,
    &canTxQueueControl);
  motorSnapshotQueue = xQueueCreateStatic(
    1U, sizeof(MotorContext_t), motorSnapshotQueueStorage,
    &motorSnapshotQueueControl);

  configASSERT(canRxQueue != NULL);
  configASSERT(canTxQueue != NULL);
  configASSERT(motorSnapshotQueue != NULL);

  vQueueAddToRegistry(canRxQueue, "CanRx");
  vQueueAddToRegistry(canTxQueue, "CanTx");
  vQueueAddToRegistry(motorSnapshotQueue, "MotorState");

  motorTaskHandle = xTaskCreateStatic(
    MotorControlTask, "MotorCtrl", MOTOR_TASK_STACK_WORDS, NULL,
    MOTOR_TASK_PRIORITY, motorTaskStack, &motorTaskControl);
  canTxTaskHandle = xTaskCreateStatic(
    CanTxTask, "CanTx", CAN_TX_TASK_STACK_WORDS, NULL,
    CAN_TX_TASK_PRIORITY, canTxTaskStack, &canTxTaskControl);
  telemetryTaskHandle = xTaskCreateStatic(
    TelemetryTask, "Telemetry", TELEMETRY_TASK_STACK_WORDS, NULL,
    TELEMETRY_TASK_PRIORITY, telemetryTaskStack, &telemetryTaskControl);

  configASSERT(motorTaskHandle != NULL);
  configASSERT(canTxTaskHandle != NULL);
  configASSERT(telemetryTaskHandle != NULL);
}

static void MotorControlTask(void *argument)
{
  TickType_t nextControlTick = xTaskGetTickCount();
  const TickType_t controlPeriod = pdMS_TO_TICKS(CONTROL_PERIOD_MS);
  CanFrame_t frame;

  (void)argument;

  Motor_DriveBrake();
  Motor_StopBrake();
  Motor_Wake();

#if START_MOTOR_ON_BOOT
  Motor_Start(MOTOR_FORWARD_ON_BOOT ? DIR_FORWARD : DIR_REVERSE);
#endif

  Motor_PublishSnapshot();
  CAN_SendStatusFrom(&motor, true);

  for (;;) {
    TickType_t nowTick = xTaskGetTickCount();
    TickType_t waitTicks = 0U;

    if ((int32_t)(nextControlTick - nowTick) > 0) {
      waitTicks = nextControlTick - nowTick;
    }

    if (xQueueReceive(canRxQueue, &frame, waitTicks) == pdPASS) {
      Control_HandleRxFrame(&frame);
      Motor_ApplyTargetSpeed();
      Motor_PublishSnapshot();
    }

    nowTick = xTaskGetTickCount();
    if ((int32_t)(nowTick - nextControlTick) >= 0) {
      uint32_t nowMs = HAL_GetTick();

      Motor_UpdateRuntime(nowMs);
      Motor_UpdateSafety(nowMs);
      Motor_ApplyTargetSpeed();
      Motor_PublishSnapshot();

      do {
        nextControlTick += controlPeriod;
      } while ((int32_t)(nowTick - nextControlTick) >= 0);
    }
  }
}

static void CanTxTask(void *argument)
{
  CanTxItem_t item;

  (void)argument;

  for (;;) {
    if (xQueueReceive(canTxQueue, &item, portMAX_DELAY) != pdPASS) {
      continue;
    }

    dbgCanLastTxId = item.frame.stdId;

    if (CAN_EnsureStarted() && CAN_TransmitFrame(&item.frame)) {
      dbgCanTxOk++;
      continue;
    }

    dbgCanTxFail++;
    dbgCanLastError = HAL_CAN_GetError(&hcan);

    if (item.urgent && (item.retries < CAN_TX_RETRY_LIMIT)) {
      item.retries++;
      if (xQueueSendToFront(canTxQueue, &item, 0U) != pdPASS) {
        dbgCanTxDropped++;
      }
    }

    vTaskDelay(pdMS_TO_TICKS(CAN_RESTART_COOLDOWN_MS));
  }
}

static void TelemetryTask(void *argument)
{
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
  MotorContext_t snapshot;

  (void)argument;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    if (xQueuePeek(motorSnapshotQueue, &snapshot, 0U) == pdPASS) {
      CAN_SendBasicFrom(&snapshot);
      CAN_SendSpeedFrom(&snapshot, false);
      CAN_SendCountersFrom(&snapshot, false);
    }
  }
}

static uint16_t ADC_ReadCurrentRaw(void)
{
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0U;
  }

  if (HAL_ADC_PollForConversion(&hadc1, 1U) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }

  uint16_t value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return value;
}

static uint16_t Current_mA_FromRaw(uint16_t raw)
{
  uint32_t voltage_mV = ((uint32_t)raw * ADC_VREF_MV) / ADC_MAX_COUNTS;
  uint32_t denominator = RIPROPI_OHMS * AIPROPI_UA_PER_A;
  uint32_t current_mA = (voltage_mV * 1000000UL) / denominator;

  return (current_mA > UINT16_MAX) ? UINT16_MAX : (uint16_t)current_mA;
}

static uint8_t ClampPercent(uint8_t percent)
{
  return (percent > 100U) ? 100U : percent;
}

static void PWM_SetComparePercent(uint8_t percent)
{
  uint32_t compare;

  percent = ClampPercent(percent);
  compare = (PWM_TIMER_COUNTS * (uint32_t)percent) / 100U;

  if (compare > PWM_TIMER_PERIOD) {
    compare = PWM_TIMER_PERIOD;
  }

  motor.pwmDutyRaw = (uint16_t)compare;
  __HAL_TIM_SET_COMPARE(&htim3, PWM_TIMER_CHANNEL, compare);
}

static void Motor_DriveBrake(void)
{
  /* DRV8876 PH/EN mode: EN=0 applies low-side slow-decay brake. */
  PWM_SetComparePercent(0U);
  motor.pwmDutyPercent = 0U;
}

static void Motor_SelectDirection(MotorDirection_t direction)
{
  GPIO_PinState ph = (direction == DIR_REVERSE) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(DRV_PH_GPIO_Port, DRV_PH_Pin, ph);
}

static void Motor_Wake(void)
{
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_RESET);
  vTaskDelay(pdMS_TO_TICKS(5U));
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_SET);
  vTaskDelay(pdMS_TO_TICKS(2U));

  motor.driverFault = false;
  driverFaultLowStartMs = 0U;
  driverFaultIgnoreUntilMs = HAL_GetTick() + DRIVER_FAULT_IGNORE_AFTER_WAKE_MS;
}

static void Motor_Start(MotorDirection_t direction)
{
  bool wasStopped;

  if (motor.motorStoppedByFault) {
    return;
  }

  if (direction != DIR_REVERSE) {
    direction = DIR_FORWARD;
  }

  wasStopped = (motor.motorState == MOTOR_STOPPED);
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_SET);
  motor.direction = direction;
  motor.motorState = (direction == DIR_REVERSE) ? MOTOR_REVERSE : MOTOR_FORWARD;
  Motor_SelectDirection(direction);
  Motor_ApplyTargetSpeed();
  motor.lastRunTickMs = HAL_GetTick();

  if (wasStopped) {
    SAT_U16_INC(motor.startStopCount);
  }
}

static void Motor_StopBrake(void)
{
  bool wasRunning = (motor.motorState != MOTOR_STOPPED);

  Motor_DriveBrake();
  motor.speedPercent = 0U;
  motor.pwmDutyPercent = 0U;
  motor.motorState = MOTOR_STOPPED;
  motor.direction = DIR_STOPPED;

  if (wasRunning) {
    SAT_U16_INC(motor.startStopCount);
  }
}

static void Motor_SetSpeedPercent(uint8_t percent)
{
  motor.targetSpeedPercent = ClampPercent(percent);
  Motor_ApplyTargetSpeed();
}

static void Motor_AdjustSpeedPercent(int8_t delta)
{
  int16_t next = (int16_t)motor.targetSpeedPercent + (int16_t)delta;

  if (next < 0) {
    next = 0;
  } else if (next > 100) {
    next = 100;
  }

  Motor_SetSpeedPercent((uint8_t)next);
}

static void Motor_ApplyTargetSpeed(void)
{
  if ((motor.motorState == MOTOR_STOPPED) || motor.motorStoppedByFault) {
    Motor_DriveBrake();
    motor.speedPercent = 0U;
    return;
  }

  Motor_SelectDirection(motor.direction);
  PWM_SetComparePercent(motor.targetSpeedPercent);
  motor.pwmDutyPercent = motor.targetSpeedPercent;
  motor.speedPercent = motor.targetSpeedPercent;
}

static void Motor_LatchFault(FaultCode_t fault)
{
  if (motor.faultCode == FAULT_NONE) {
    motor.faultCode = fault;
    SAT_U16_INC(motor.faultCount);
    CAN_SendFault(fault);
  }

  motor.motorStoppedByFault = true;
  Motor_StopBrake();
}

static void Motor_ResetFaults(void)
{
  motor.faultCode = FAULT_NONE;
  motor.driverFault = false;
  motor.overcurrentActive = false;
  motor.motorStoppedByFault = false;
  motor.testMode = false;
  overcurrentStartMs = 0U;
  driverFaultLowStartMs = 0U;
}

static void Motor_UpdateSafety(uint32_t nowMs)
{
  motor.current_mA = Current_mA_FromRaw(ADC_ReadCurrentRaw());
  Motor_UpdateDriverFault(nowMs);
  Motor_UpdateCurrent(nowMs);
  Motor_UpdateTestFault(nowMs);
}

static void Motor_UpdateCurrent(uint32_t nowMs)
{
  if (motor.current_mA < CURRENT_LIMIT_MA) {
    overcurrentStartMs = 0U;
    motor.overcurrentActive = false;
    return;
  }

  motor.overcurrentActive = true;
  if (overcurrentStartMs == 0U) {
    overcurrentStartMs = nowMs;
  }

  if ((nowMs - overcurrentStartMs) >= OVERCURRENT_STOP_MS) {
    Motor_LatchFault(FAULT_OVERCURRENT);
  }
}

static void Motor_UpdateDriverFault(uint32_t nowMs)
{
  bool pinLow =
    (HAL_GPIO_ReadPin(DRV_NFAULT_GPIO_Port, DRV_NFAULT_Pin) == GPIO_PIN_RESET);

  motor.driverFault = pinLow;
  if (!pinLow) {
    driverFaultLowStartMs = 0U;
    return;
  }

  if ((int32_t)(nowMs - driverFaultIgnoreUntilMs) < 0) {
    driverFaultLowStartMs = 0U;
    return;
  }

  if (driverFaultLowStartMs == 0U) {
    driverFaultLowStartMs = nowMs;
  }

  if ((nowMs - driverFaultLowStartMs) >= DRIVER_FAULT_DEBOUNCE_MS) {
    Motor_LatchFault(FAULT_DRIVER);
  }
}

static void Motor_UpdateTestFault(uint32_t nowMs)
{
  if (motor.testMode && !motor.motorStoppedByFault &&
      ((nowMs - motor.testStartMs) >= OVERCURRENT_STOP_MS)) {
    motor.testMode = false;
    Motor_LatchFault(FAULT_TEST);
  }
}

static void Motor_UpdateRuntime(uint32_t nowMs)
{
  if (motor.motorState == MOTOR_STOPPED) {
    motor.lastRunTickMs = nowMs;
    return;
  }

  if (motor.lastRunTickMs == 0U) {
    motor.lastRunTickMs = nowMs;
    return;
  }

  motor.runTimeMs += nowMs - motor.lastRunTickMs;
  motor.lastRunTickMs = nowMs;
}

static void Motor_PublishSnapshot(void)
{
  (void)xQueueOverwrite(motorSnapshotQueue, &motor);
}

static char ToUpperAscii(char value)
{
  if ((value >= 'a') && (value <= 'z')) {
    return (char)(value - ('a' - 'A'));
  }
  return value;
}

static bool Payload_HasTextPrefix(const uint8_t *payload, uint8_t dlc,
                                  const char *prefix)
{
  uint8_t index = 0U;

  while (prefix[index] != '\0') {
    if ((index >= dlc) ||
        (ToUpperAscii((char)payload[index]) != ToUpperAscii(prefix[index]))) {
      return false;
    }
    index++;
  }
  return true;
}

static Command_t Command_FromPayload(const uint8_t *payload, uint8_t dlc)
{
  char value;

  if ((payload == NULL) || (dlc == 0U)) {
    return CMD_NONE;
  }

  if (Payload_HasTextPrefix(payload, dlc, "SPEED_U") ||
      Payload_HasTextPrefix(payload, dlc, "UP")) {
    return CMD_SPEED_UP;
  }
  if (Payload_HasTextPrefix(payload, dlc, "SPEED_D") ||
      Payload_HasTextPrefix(payload, dlc, "DOWN")) {
    return CMD_SPEED_DOWN;
  }

  value = ToUpperAscii((char)payload[0]);
  switch (value) {
    case 'A': return CMD_A;
    case 'B': return CMD_B;
    case 'X': return CMD_X;
    case 'D': return CMD_D;
    case 'R': return CMD_R;
    case 'T': return CMD_T;
    case 'F': return CMD_F;
    case 'I': return CMD_I;
    case 'S': return CMD_S;
    case 'P': return CMD_P;
    case '+':
    case 'U': return CMD_SPEED_UP;
    case '-':
    case 'N': return CMD_SPEED_DOWN;
    default: return CMD_NONE;
  }
}

static char Command_ToChar(Command_t command)
{
  switch (command) {
    case CMD_A: return 'A';
    case CMD_B: return 'B';
    case CMD_X: return 'X';
    case CMD_D: return 'D';
    case CMD_R: return 'R';
    case CMD_T: return 'T';
    case CMD_F: return 'F';
    case CMD_I: return 'I';
    case CMD_S: return 'S';
    case CMD_P: return 'P';
    case CMD_SPEED_UP: return 'U';
    case CMD_SPEED_DOWN: return 'N';
    default: return '?';
  }
}

static CommandEntry_t Command_Decode(const CanFrame_t *frame)
{
  CommandEntry_t entry = { CMD_NONE, 0U, false };

  entry.id = Command_FromPayload(frame->data, frame->dlc);
  if ((frame->dlc >= 2U) &&
      ((entry.id == CMD_P) || (entry.id == CMD_A) || (entry.id == CMD_B))) {
    entry.value = ClampPercent(frame->data[1]);
    entry.hasValue = (entry.id == CMD_P) || (entry.value > 0U);
  }
  return entry;
}

static void Command_Process(CommandEntry_t entry)
{
  switch (entry.id) {
    case CMD_A:
    case CMD_B:
      if (entry.hasValue) {
        Motor_SetSpeedPercent(entry.value);
      }
      if (motor.motorStoppedByFault) {
        if (motor.faultCode != FAULT_REMOTE) {
          CAN_SendAck(entry.id, false, (uint8_t)motor.faultCode);
          return;
        }
        Motor_ResetFaults();
      }
      Motor_Start((entry.id == CMD_B) ? DIR_REVERSE : DIR_FORWARD);
      CAN_SendAck(entry.id, true, 0U);
      break;

    case CMD_P:
      if (!entry.hasValue) {
        CAN_SendAck(entry.id, false, 0xFEU);
      } else {
        Motor_SetSpeedPercent(entry.value);
        CAN_SendAck(entry.id, true, 0U);
      }
      break;

    case CMD_SPEED_UP:
    case CMD_SPEED_DOWN:
      if (motor.motorStoppedByFault) {
        CAN_SendAck(entry.id, false, (uint8_t)motor.faultCode);
      } else {
        int8_t step = (entry.id == CMD_SPEED_UP) ?
                      (int8_t)MOTOR_SPEED_STEP_PERCENT :
                      -(int8_t)MOTOR_SPEED_STEP_PERCENT;
        Motor_AdjustSpeedPercent(step);
        CAN_SendAck(entry.id, true, 0U);
        CAN_SendSpeedFrom(&motor, true);
      }
      break;

    case CMD_X:
      Motor_StopBrake();
      CAN_SendAck(entry.id, true, 0U);
      break;

    case CMD_D:
      Motor_LatchFault(FAULT_REMOTE);
      CAN_SendAck(entry.id, true, 0U);
      break;

    case CMD_R:
      Motor_ResetFaults();
      Motor_StopBrake();
      motor.targetSpeedPercent = MOTOR_DEFAULT_SPEED_PERCENT;
      CAN_SendAck(entry.id, true, 0U);
      CAN_SendStatusFrom(&motor, true);
      CAN_SendSpeedFrom(&motor, true);
      break;

    case CMD_T:
      if (motor.motorStoppedByFault) {
        CAN_SendAck(entry.id, false, (uint8_t)motor.faultCode);
      } else {
        motor.testMode = true;
        motor.testStartMs = HAL_GetTick();
        CAN_SendAck(entry.id, true, 0U);
      }
      break;

    case CMD_F:
      CAN_SendAck(entry.id, true, 0U);
      break;

    case CMD_I:
    case CMD_S:
      CAN_SendStatusFrom(&motor, true);
      CAN_SendSpeedFrom(&motor, true);
      CAN_SendCountersFrom(&motor, true);
      CAN_SendAck(entry.id, true, 0U);
      break;

    default:
      break;
  }
}

static void Control_HandleRxFrame(const CanFrame_t *frame)
{
  if (frame->stdId == CAN_ID_CMD_RX) {
    CommandEntry_t entry = Command_Decode(frame);
    if (entry.id != CMD_NONE) {
      Command_Process(entry);
    }
    return;
  }

  if ((frame->stdId == SENSOR_TELEM_ID_RX) && (frame->dlc >= 3U)) {
    int16_t temp_x100 =
      (int16_t)((uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8U));

    if ((temp_x100 > SENSOR_TEMP_STOP_X100) && !motor.motorStoppedByFault) {
      Motor_LatchFault(FAULT_SENSOR_TEMPERATURE);
    } else if ((frame->data[2] >= SENSOR_VIB_STOP_PCT) &&
               !motor.motorStoppedByFault) {
      Motor_LatchFault(FAULT_SENSOR_VIBRATION);
    }
  }
}

static bool CAN_QueueFrame(uint16_t stdId, const uint8_t *data, uint8_t dlc,
                           bool urgent)
{
  CanTxItem_t item = {0};
  BaseType_t result;

  if ((stdId > 0x7FFU) || (dlc > 8U) || ((dlc > 0U) && (data == NULL))) {
    return false;
  }

  item.frame.stdId = stdId;
  item.frame.dlc = dlc;
  item.urgent = urgent;
  if (dlc > 0U) {
    memcpy(item.frame.data, data, dlc);
  }

  result = urgent ?
    xQueueSendToFront(canTxQueue, &item, 0U) :
    xQueueSendToBack(canTxQueue, &item, 0U);

  if (result == pdPASS) {
    dbgCanTxQueued++;
    return true;
  }

  dbgCanTxDropped++;
  return false;
}

static bool CAN_EnsureStarted(void)
{
  HAL_CAN_StateTypeDef state = HAL_CAN_GetState(&hcan);
  bool busOff = ((hcan.Instance->ESR & CAN_ESR_BOFF_FLAG) != 0U);
  uint32_t nowMs = HAL_GetTick();

  if ((state == HAL_CAN_STATE_LISTENING) && !busOff) {
    return true;
  }
  if ((nowMs - lastCanRestartMs) < CAN_RESTART_COOLDOWN_MS) {
    return false;
  }

  lastCanRestartMs = nowMs;
  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
  (void)HAL_CAN_Stop(&hcan);

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    return false;
  }

  if (HAL_CAN_ActivateNotification(
        &hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
    return false;
  }

  HAL_NVIC_ClearPendingIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_ClearPendingIRQ(USB_HP_CAN1_TX_IRQn);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
  return true;
}

static bool CAN_TransmitFrame(const CanFrame_t *frame)
{
  CAN_TxHeaderTypeDef header = {0};
  uint32_t mailbox;
  TickType_t start = xTaskGetTickCount();

  while (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0U) {
    if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(CAN_MAILBOX_WAIT_MS)) {
      return false;
    }
    vTaskDelay(1U);
  }

  header.StdId = frame->stdId;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = frame->dlc;
  header.TransmitGlobalTime = DISABLE;

  (void)ulTaskNotifyTake(pdTRUE, 0U);
  if (HAL_CAN_AddTxMessage(
        &hcan, &header, (uint8_t *)frame->data, &mailbox) != HAL_OK) {
    return false;
  }

  if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(CAN_TX_DONE_WAIT_MS)) == 0U) {
    if (HAL_CAN_IsTxMessagePending(&hcan, mailbox)) {
      (void)HAL_CAN_AbortTxRequest(&hcan, mailbox);
    }
    return false;
  }

  return !HAL_CAN_IsTxMessagePending(&hcan, mailbox);
}

static void CAN_SendAck(Command_t command, bool accepted, uint8_t reason)
{
  uint8_t payload[8] = {
    (uint8_t)Command_ToChar(command), accepted ? 1U : 0U, reason,
    (uint8_t)motor.motorState, (uint8_t)motor.faultCode,
    (uint8_t)(motor.current_mA & 0xFFU),
    (uint8_t)(motor.current_mA >> 8U), motor.speedPercent
  };
  (void)CAN_QueueFrame(CAN_ID_ACK_TX, payload, 8U, true);
}

static void CAN_SendFault(FaultCode_t fault)
{
  uint8_t payload[8] = {
    (uint8_t)fault, (uint8_t)motor.motorState,
    (uint8_t)(motor.current_mA & 0xFFU),
    (uint8_t)(motor.current_mA >> 8U),
    motor.driverFault ? 1U : 0U, motor.overcurrentActive ? 1U : 0U,
    motor.testMode ? 1U : 0U, motor.speedPercent
  };
  (void)CAN_QueueFrame(CAN_ID_FAULT_TX, payload, 8U, true);
}

static void CAN_SendStatusFrom(const MotorContext_t *s, bool urgent)
{
  uint8_t payload[8] = {
    (uint8_t)s->motorState, (uint8_t)s->faultCode,
    s->driverFault ? 1U : 0U, s->overcurrentActive ? 1U : 0U,
    s->motorStoppedByFault ? 1U : 0U,
    (uint8_t)(s->current_mA & 0xFFU),
    (uint8_t)(s->current_mA >> 8U), s->speedPercent
  };
  (void)CAN_QueueFrame(CAN_ID_STATUS_TX, payload, 8U, urgent);
}

static void CAN_SendBasicFrom(const MotorContext_t *s)
{
  uint8_t payload[8] = {
    (uint8_t)(s->current_mA & 0xFFU),
    (uint8_t)(s->current_mA >> 8U),
    (uint8_t)s->motorState, (uint8_t)s->faultCode,
    s->driverFault ? 1U : 0U, s->overcurrentActive ? 1U : 0U,
    s->testMode ? 1U : 0U, s->speedPercent
  };
  (void)CAN_QueueFrame(CAN_ID_TELEM_TX, payload, 8U, false);
}

static void CAN_SendSpeedFrom(const MotorContext_t *s, bool urgent)
{
  uint8_t payload[8] = {
    s->speedPercent, s->targetSpeedPercent, s->pwmDutyPercent,
    (uint8_t)s->direction,
    (uint8_t)(s->pwmDutyRaw & 0xFFU),
    (uint8_t)(s->pwmDutyRaw >> 8U),
    (uint8_t)s->motorState, 0U
  };
  (void)CAN_QueueFrame(CAN_ID_SPEED_TX, payload, 8U, urgent);
}

static void CAN_SendCountersFrom(const MotorContext_t *s, bool urgent)
{
  uint8_t payload[8] = {
    (uint8_t)(s->runTimeMs & 0xFFU),
    (uint8_t)((s->runTimeMs >> 8U) & 0xFFU),
    (uint8_t)((s->runTimeMs >> 16U) & 0xFFU),
    (uint8_t)((s->runTimeMs >> 24U) & 0xFFU),
    (uint8_t)(s->startStopCount & 0xFFU),
    (uint8_t)(s->startStopCount >> 8U),
    (uint8_t)(s->faultCount & 0xFFU),
    (uint8_t)(s->faultCount >> 8U)
  };
  (void)CAN_QueueFrame(CAN_ID_COUNTER_TX, payload, 8U, urgent);
}

static void CAN_NotifyTxTaskFromISR(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (canTxTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(canTxTaskHandle, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

static void CAN_ConfigFilters(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef config = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;

  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    Error_Handler();
  }

  config.Channel = ADC_CHANNEL_3;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &config) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef clockConfig = {0};
  TIM_MasterConfigTypeDef masterConfig = {0};
  TIM_OC_InitTypeDef outputConfig = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0U;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PWM_TIMER_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }
  clockConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &clockConfig) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
    Error_Handler();
  }

  masterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  masterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &masterConfig) != HAL_OK) {
    Error_Handler();
  }

  outputConfig.OCMode = TIM_OCMODE_PWM1;
  outputConfig.Pulse = 0U;
  outputConfig.OCPolarity = TIM_OCPOLARITY_HIGH;
  outputConfig.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(
        &htim3, &outputConfig, PWM_TIMER_CHANNEL) != HAL_OK) {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  HAL_GPIO_WritePin(GPIOB, DRV_PH_Pin | DRV_NSLEEP_Pin, GPIO_PIN_RESET);

  gpio.Pin = DRV_NFAULT_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DRV_NFAULT_GPIO_Port, &gpio);

  gpio.Pin = DRV_PH_Pin | DRV_NSLEEP_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef *can)
{
  GPIO_InitTypeDef gpio = {0};

  if (can->Instance != CAN1) {
    return;
  }

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_11;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 5U, 0U);
  HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(USB_HP_CAN1_TX_IRQn);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef *adc)
{
  GPIO_InitTypeDef gpio = {0};

  if (adc->Instance != ADC1) {
    return;
  }
  __HAL_RCC_ADC1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  gpio.Pin = GPIO_PIN_3;
  gpio.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  } else if (timer->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM3) {
    __HAL_RCC_TIM3_CLK_ENABLE();
  }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *timer)
{
  GPIO_InitTypeDef gpio = {0};

  if (timer->Instance != TIM3) {
    return;
  }
  __HAL_RCC_GPIOB_CLK_ENABLE();
  gpio.Pin = DRV_EN_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(DRV_EN_GPIO_Port, &gpio);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((can->Instance != CAN1) || (canRxQueue == NULL)) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) > 0U) {
    CAN_RxHeaderTypeDef header;
    CanFrame_t frame = {0};
    bool urgent = false;
    BaseType_t queued;

    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, frame.data) != HAL_OK) {
      break;
    }
    if ((header.IDE != CAN_ID_STD) || (header.RTR != CAN_RTR_DATA)) {
      continue;
    }

    frame.stdId = (uint16_t)header.StdId;
    frame.dlc = (header.DLC > 8U) ? 8U : (uint8_t)header.DLC;

    if ((frame.stdId == CAN_ID_CMD_RX) && (frame.dlc > 0U)) {
      char command = ToUpperAscii((char)frame.data[0]);
      urgent = (command == 'D') || (command == 'X');
    } else if ((frame.stdId == SENSOR_TELEM_ID_RX) && (frame.dlc >= 3U)) {
      int16_t temp =
        (int16_t)((uint16_t)frame.data[0] | ((uint16_t)frame.data[1] << 8U));
      urgent = (temp > SENSOR_TEMP_STOP_X100) ||
               (frame.data[2] >= SENSOR_VIB_STOP_PCT);
    }

    queued = urgent ?
      xQueueSendToFrontFromISR(canRxQueue, &frame, &higherPriorityTaskWoken) :
      xQueueSendToBackFromISR(canRxQueue, &frame, &higherPriorityTaskWoken);

    if ((queued != pdPASS) && urgent) {
      CanFrame_t discarded;
      (void)xQueueReceiveFromISR(canRxQueue, &discarded, &higherPriorityTaskWoken);
      queued = xQueueSendToFrontFromISR(
        canRxQueue, &frame, &higherPriorityTaskWoken);
    }

    if (queued == pdPASS) {
      dbgCanRxQueued++;
    } else {
      dbgCanRxDropped++;
    }
  }

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    CAN_NotifyTxTaskFromISR();
  }
}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    CAN_NotifyTxTaskFromISR();
  }
}

void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    CAN_NotifyTxTaskFromISR();
  }
}

void HAL_CAN_TxMailbox0AbortCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    CAN_NotifyTxTaskFromISR();
  }
}

void HAL_CAN_TxMailbox1AbortCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    CAN_NotifyTxTaskFromISR();
  }
}

void HAL_CAN_TxMailbox2AbortCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    CAN_NotifyTxTaskFromISR();
  }
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void USB_HP_CAN1_TX_IRQHandler(void)
{
  HAL_CAN_IRQHandler(&hcan);
}

void TIM2_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim2HalTick);
}

HAL_StatusTypeDef HAL_InitTick(uint32_t tickPriority)
{
  uint32_t pclk1;
  uint32_t timerClock;

  __HAL_RCC_TIM2_CLK_ENABLE();
  pclk1 = HAL_RCC_GetPCLK1Freq();
  timerClock = ((RCC->CFGR & RCC_CFGR_PPRE1) == 0U) ? pclk1 : (pclk1 * 2U);

  if (timerClock < 1000000U) {
    return HAL_ERROR;
  }

  if (htim2HalTick.Instance == TIM2) {
    (void)HAL_TIM_Base_Stop_IT(&htim2HalTick);
  }
  htim2HalTick.Instance = TIM2;
  htim2HalTick.Init.Prescaler = (timerClock / 1000000U) - 1U;
  htim2HalTick.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2HalTick.Init.Period = 999U;
  htim2HalTick.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2HalTick.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_Base_Init(&htim2HalTick) != HAL_OK) {
    return HAL_ERROR;
  }

  HAL_NVIC_SetPriority(TIM2_IRQn, tickPriority, 0U);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  uwTickPrio = tickPriority;
  return HAL_TIM_Base_Start_IT(&htim2HalTick);
}

void HAL_SuspendTick(void)
{
  __HAL_TIM_DISABLE_IT(&htim2HalTick, TIM_IT_UPDATE);
}

void HAL_ResumeTick(void)
{
  __HAL_TIM_ENABLE_IT(&htim2HalTick, TIM_IT_UPDATE);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM2) {
    HAL_IncTick();
  }
}

void vApplicationGetIdleTaskMemory(StaticTask_t **taskBuffer,
                                   StackType_t **stackBuffer,
                                   uint32_t *stackSize)
{
  *taskBuffer = &idleTaskControl;
  *stackBuffer = idleTaskStack;
  *stackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *taskName)
{
  (void)task;
  (void)taskName;
  Error_Handler();
}

void vApplicationMallocFailedHook(void)
{
  Error_Handler();
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  RCC_PeriphCLKInitTypeDef periph = {0};

  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.HSIState = RCC_HSI_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Error_Handler();
  }

  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }

  periph.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  periph.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  __HAL_TIM_SET_COMPARE(&htim3, PWM_TIMER_CHANNEL, 0U);
  HAL_GPIO_WritePin(DRV_NSLEEP_GPIO_Port, DRV_NSLEEP_Pin, GPIO_PIN_RESET);
  for (;;) {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
