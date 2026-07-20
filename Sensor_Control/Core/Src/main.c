/*
 * STM32F103C8T6 CAN sensor node - native FreeRTOS version
 *
 * Hardware:
 *   PA11 CAN_RX, PA12 CAN_TX, 500 kbit/s
 *   PA4  ADC1_IN4, piezo vibration input
 *   PA1  DS18B20 1-Wire, external 4.7 kohm pull-up to 3.3 V
 *
 * FreeRTOS ownership:
 *   SensorControlTask (priority 5): sole owner of sensor state and ADC
 *   CanTxTask        (priority 4): sole owner of CAN transmission/recovery
 *   TemperatureTask  (priority 3): sole owner of the 1-Wire bus
 *   TelemetryTask    (priority 2): fixed 500 ms telemetry trigger
 *
 * CAN IDs and payload layout are unchanged:
 *   RX 0x102 command
 *   TX 0x182 ACK, 0x202 telemetry, 0x212 status, 0x222 fault
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
static TIM_HandleTypeDef htim2HalTick;

#define REAL_TEMP_SENSOR                  1

#define PIEZO_ADC_CHANNEL                 ADC_CHANNEL_4
#define PIEZO_ACTIVE_RAW                  50U
#define PIEZO_MAX_RAW                     1500U
#define PIEZO_SAMPLE_PERIOD_MS            1U

#define OW_PORT                           GPIOA
#define OW_PIN                            GPIO_PIN_1
#define DS_CONVERSION_MS                  800U
#define DS_RETRY_MS                       100U
#define SENSOR_FAIL_LIMIT                 10U
#define TEMP_MIN_X100                     (-5500)
#define TEMP_MAX_X100                     12500

#define TEST_FAULT_MS                     5000U
#define TELEMETRY_PERIOD_MS               500U

#define CAN_ID_CMD_RX                     0x102U
#define CAN_ID_ACK_TX                     0x182U
#define CAN_ID_TELEM_TX                   0x202U
#define CAN_ID_STATUS_TX                  0x212U
#define CAN_ID_FAULT_TX                   0x222U

#define SENSOR_EVENT_QUEUE_LENGTH         20U
#define CAN_TX_QUEUE_LENGTH               16U
#define CAN_TX_RETRY_LIMIT                2U
#define CAN_MAILBOX_WAIT_MS               3U
#define CAN_TX_DONE_WAIT_MS               10U
#define CAN_RESTART_COOLDOWN_MS            100U
#define CAN_ESR_BOFF_FLAG                 0x00000004U

#define SENSOR_TASK_PRIORITY              5U
#define CAN_TX_TASK_PRIORITY              4U
#define TEMPERATURE_TASK_PRIORITY         3U
#define TELEMETRY_TASK_PRIORITY           2U

#define SENSOR_TASK_STACK_WORDS           320U
#define CAN_TX_TASK_STACK_WORDS           256U
#define TEMPERATURE_TASK_STACK_WORDS      320U
#define TELEMETRY_TASK_STACK_WORDS        160U

typedef enum {
  SYS_NORMAL = 0,
  SYS_WARNING = 1,
  SYS_ERROR = 2
} SystemState_t;

typedef enum {
  FAULT_NONE = 0,
  FAULT_REMOTE = 1,
  FAULT_TEST = 2,
  FAULT_TEMP_SENSOR = 3
} FaultCode_t;

typedef enum {
  CMD_NONE = 0,
  CMD_A,
  CMD_X,
  CMD_D,
  CMD_R,
  CMD_T,
  CMD_F,
  CMD_I,
  CMD_S
} Command_t;

typedef enum {
  SENSOR_EVENT_COMMAND = 0,
  SENSOR_EVENT_TEMPERATURE,
  SENSOR_EVENT_TELEMETRY
} SensorEventType_t;

typedef struct {
  int32_t temperatureX100;
  uint32_t vibrationPercent;
  uint32_t vibrationPeakRaw;
  bool temperatureValid;
  bool temperatureSensorOk;
  bool streamEnabled;
  bool testMode;
  uint32_t testStartMs;
  SystemState_t state;
  FaultCode_t fault;
} SensorContext_t;

typedef struct {
  SensorEventType_t type;
  union {
    uint8_t commandByte;
    struct {
      int32_t measuredX100;
      bool valid;
    } temperature;
  } value;
} SensorEvent_t;

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

static SensorContext_t sensor = {
  .temperatureX100 = 0,
  .vibrationPercent = 0U,
  .vibrationPeakRaw = 0U,
  .temperatureValid = false,
  .temperatureSensorOk = false,
  .streamEnabled = true,
  .testMode = false,
  .testStartMs = 0U,
  .state = SYS_NORMAL,
  .fault = FAULT_NONE
};

static uint8_t sensorFailCount;
static uint32_t lastCanRestartMs;

static volatile uint32_t dbgEventQueued;
static volatile uint32_t dbgEventDropped;
static volatile uint32_t dbgCanTxQueued;
static volatile uint32_t dbgCanTxDropped;
static volatile uint32_t dbgCanTxOk;
static volatile uint32_t dbgCanTxFail;
static volatile uint32_t dbgCanLastError;
static volatile uint32_t dbgCanLastTxId;

static QueueHandle_t sensorEventQueue;
static QueueHandle_t canTxQueue;
static StaticQueue_t sensorEventQueueControl;
static StaticQueue_t canTxQueueControl;
static uint8_t sensorEventQueueStorage[
  SENSOR_EVENT_QUEUE_LENGTH * sizeof(SensorEvent_t)];
static uint8_t canTxQueueStorage[CAN_TX_QUEUE_LENGTH * sizeof(CanTxItem_t)];

static TaskHandle_t sensorTaskHandle;
static TaskHandle_t canTxTaskHandle;
static TaskHandle_t temperatureTaskHandle;
static TaskHandle_t telemetryTaskHandle;
static StaticTask_t sensorTaskControl;
static StaticTask_t canTxTaskControl;
static StaticTask_t temperatureTaskControl;
static StaticTask_t telemetryTaskControl;
static StackType_t sensorTaskStack[SENSOR_TASK_STACK_WORDS];
static StackType_t canTxTaskStack[CAN_TX_TASK_STACK_WORDS];
static StackType_t temperatureTaskStack[TEMPERATURE_TASK_STACK_WORDS];
static StackType_t telemetryTaskStack[TELEMETRY_TASK_STACK_WORDS];

static StaticTask_t idleTaskControl;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN_Init(void);
static void CAN_ConfigFilters(void);
static void RTOS_CreateObjects(void);

static void SensorControlTask(void *argument);
static void CanTxTask(void *argument);
static void TemperatureTask(void *argument);
static void TelemetryTask(void *argument);

static uint16_t ADC_ReadPiezoRaw(void);
static uint32_t Piezo_RawToPercent(uint32_t raw);
static void Sensor_ProcessEvent(const SensorEvent_t *event);
static void Sensor_ProcessCommand(Command_t command);
static void Sensor_UpdateTemperature(int32_t measuredX100, bool valid);
static void Sensor_UpdateTestFault(uint32_t nowMs);
static void Sensor_LatchFault(FaultCode_t fault);
static void Sensor_ResetFaults(void);
static Command_t Command_FromChar(char value);
static char Command_ToChar(Command_t command);

static bool CAN_QueueFrame(uint16_t stdId, const uint8_t *data, uint8_t dlc,
                           bool urgent);
static bool CAN_EnsureStarted(void);
static bool CAN_TransmitFrame(const CanFrame_t *frame);
static void CAN_SendAck(Command_t command, bool accepted, uint8_t reason);
static void CAN_SendTelemetry(void);
static void CAN_SendStatus(bool urgent);
static void CAN_SendFault(FaultCode_t fault);
static void CAN_NotifyTxTaskFromISR(void);

#if REAL_TEMP_SENSOR
static void DWT_Init(void);
static void OW_DelayUs(uint32_t us);
static void OW_SetOutput(void);
static void OW_SetInput(void);
static bool OW_Reset(void);
static void OW_WriteByte(uint8_t value);
static uint8_t OW_ReadByte(void);
static uint8_t OW_Crc8(const uint8_t *data, uint8_t length);
static bool DS18B20_StartConversion(void);
static bool DS18B20_ReadTemperature(int32_t *measuredX100);
#endif

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_CAN_Init();

  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
    Error_Handler();
  }

#if REAL_TEMP_SENSOR
  DWT_Init();
#endif

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
  sensorEventQueue = xQueueCreateStatic(
    SENSOR_EVENT_QUEUE_LENGTH,
	sizeof(SensorEvent_t),
    sensorEventQueueStorage,
	&sensorEventQueueControl);
  canTxQueue = xQueueCreateStatic(
    CAN_TX_QUEUE_LENGTH, sizeof(CanTxItem_t),
    canTxQueueStorage, &canTxQueueControl);

  configASSERT(sensorEventQueue != NULL);
  configASSERT(canTxQueue != NULL);

  vQueueAddToRegistry(sensorEventQueue, "SensorEvent");
  vQueueAddToRegistry(canTxQueue, "CanTx");

  sensorTaskHandle = xTaskCreateStatic(
    SensorControlTask, "SensorCtrl", SENSOR_TASK_STACK_WORDS, NULL,
    SENSOR_TASK_PRIORITY, sensorTaskStack, &sensorTaskControl);
  canTxTaskHandle = xTaskCreateStatic(
    CanTxTask, "CanTx", CAN_TX_TASK_STACK_WORDS, NULL,
    CAN_TX_TASK_PRIORITY, canTxTaskStack, &canTxTaskControl);
  temperatureTaskHandle = xTaskCreateStatic(
    TemperatureTask, "Temperature", TEMPERATURE_TASK_STACK_WORDS, NULL,
    TEMPERATURE_TASK_PRIORITY, temperatureTaskStack, &temperatureTaskControl);
  telemetryTaskHandle = xTaskCreateStatic(
    TelemetryTask, "Telemetry", TELEMETRY_TASK_STACK_WORDS, NULL,
    TELEMETRY_TASK_PRIORITY, telemetryTaskStack, &telemetryTaskControl);

  configASSERT(sensorTaskHandle != NULL);
  configASSERT(canTxTaskHandle != NULL);
  configASSERT(temperatureTaskHandle != NULL);
  configASSERT(telemetryTaskHandle != NULL);
}

static void SensorControlTask(void *argument)
{
  TickType_t nextSample = xTaskGetTickCount();
  const TickType_t samplePeriod = pdMS_TO_TICKS(PIEZO_SAMPLE_PERIOD_MS);
  SensorEvent_t event;

  (void)argument;
  CAN_SendStatus(true);

  for (;;) {
    TickType_t now = xTaskGetTickCount();
    TickType_t waitTicks = 0U;

    if ((int32_t)(nextSample - now) > 0) {
      waitTicks = nextSample - now;
    }

    if (xQueueReceive(sensorEventQueue, &event, waitTicks) == pdPASS) {
      Sensor_ProcessEvent(&event);

      for (uint8_t count = 0U; count < 7U; count++) {
        if (xQueueReceive(sensorEventQueue, &event, 0U) != pdPASS) {
          break;
        }
        Sensor_ProcessEvent(&event);
      }
    }

    now = xTaskGetTickCount();
    if ((int32_t)(now - nextSample) >= 0) {
      uint32_t raw = ADC_ReadPiezoRaw();

      if (raw > sensor.vibrationPeakRaw) {
        sensor.vibrationPeakRaw = raw;
      }
      Sensor_UpdateTestFault(HAL_GetTick());

      do {
        nextSample += samplePeriod;
      } while ((int32_t)(now - nextSample) >= 0);
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

static void TemperatureTask(void *argument)
{
  SensorEvent_t event = {0};

  (void)argument;
  event.type = SENSOR_EVENT_TEMPERATURE;

  for (;;) {
#if REAL_TEMP_SENSOR
    int32_t measuredX100 = 0;

    if (!DS18B20_StartConversion()) {
      event.value.temperature.measuredX100 = 0;
      event.value.temperature.valid = false;
      if (xQueueSendToBack(sensorEventQueue, &event, 0U) == pdPASS) {
        dbgEventQueued++;
      } else {
        dbgEventDropped++;
      }
      vTaskDelay(pdMS_TO_TICKS(DS_RETRY_MS));
      continue;
    }

    vTaskDelay(pdMS_TO_TICKS(DS_CONVERSION_MS));
    event.value.temperature.valid =
      DS18B20_ReadTemperature(&measuredX100);
    event.value.temperature.measuredX100 = measuredX100;
#else
    event.value.temperature.valid = true;
    event.value.temperature.measuredX100 = 2500;
    vTaskDelay(pdMS_TO_TICKS(DS_CONVERSION_MS));
#endif

    if (xQueueSendToBack(sensorEventQueue, &event, 0U) == pdPASS) {
      dbgEventQueued++;
    } else {
      dbgEventDropped++;
    }
  }
}

static void TelemetryTask(void *argument)
{
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
  const SensorEvent_t event = { .type = SENSOR_EVENT_TELEMETRY };

  (void)argument;

  for (;;) {
    vTaskDelayUntil(&lastWake, period);

    if (xQueueSendToBack(sensorEventQueue, &event, 0U) == pdPASS) {
      dbgEventQueued++;
    } else {
      dbgEventDropped++;
    }
  }
}

static uint16_t ADC_ReadPiezoRaw(void)
{
  uint16_t value;

  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return 0U;
  }
  if (HAL_ADC_PollForConversion(&hadc1, 1U) != HAL_OK) {
    (void)HAL_ADC_Stop(&hadc1);
    return 0U;
  }

  value = (uint16_t)HAL_ADC_GetValue(&hadc1);
  (void)HAL_ADC_Stop(&hadc1);
  return value;
}

static uint32_t Piezo_RawToPercent(uint32_t raw)
{
  uint32_t percent;

  if (raw <= PIEZO_ACTIVE_RAW) {
    return 0U;
  }
  if (raw >= PIEZO_MAX_RAW) {
    return 100U;
  }

  percent = ((raw - PIEZO_ACTIVE_RAW) * 100U) /
            (PIEZO_MAX_RAW - PIEZO_ACTIVE_RAW);
  return (percent > 100U) ? 100U : percent;
}

static void Sensor_ProcessEvent(const SensorEvent_t *event)
{
  switch (event->type) {
    case SENSOR_EVENT_COMMAND: {
      Command_t command = Command_FromChar((char)event->value.commandByte);
      if (command != CMD_NONE) {
        Sensor_ProcessCommand(command);
      }
      break;
    }

    case SENSOR_EVENT_TEMPERATURE:
      Sensor_UpdateTemperature(
        event->value.temperature.measuredX100,
        event->value.temperature.valid);
      break;

    case SENSOR_EVENT_TELEMETRY:
      sensor.vibrationPercent =
        Piezo_RawToPercent(sensor.vibrationPeakRaw);
      sensor.vibrationPeakRaw = 0U;

      if (sensor.streamEnabled || (sensor.state == SYS_ERROR)) {
        CAN_SendTelemetry();
      }
      break;

    default:
      break;
  }
}

static void Sensor_ProcessCommand(Command_t command)
{
  switch (command) {
    case CMD_A:
      if (sensor.state == SYS_ERROR) {
        CAN_SendAck(command, false, (uint8_t)sensor.fault);
      } else {
        sensor.streamEnabled = true;
        CAN_SendAck(command, true, 0U);
      }
      break;

    case CMD_X:
      sensor.streamEnabled = false;
      CAN_SendAck(command, true, 0U);
      break;

    case CMD_D:
      Sensor_LatchFault(FAULT_REMOTE);
      CAN_SendAck(command, true, 0U);
      break;

    case CMD_R:
      Sensor_ResetFaults();
      CAN_SendAck(command, true, 0U);
      break;

    case CMD_T:
      if (sensor.state == SYS_ERROR) {
        CAN_SendAck(command, false, (uint8_t)sensor.fault);
      } else {
        sensor.testMode = true;
        sensor.testStartMs = HAL_GetTick();
        CAN_SendAck(command, true, 0U);
      }
      break;

    case CMD_F:
      CAN_SendAck(command, true, 0U);
      break;

    case CMD_I:
    case CMD_S:
      CAN_SendStatus(true);
      CAN_SendAck(command, true, 0U);
      break;

    default:
      break;
  }
}

static void Sensor_UpdateTemperature(int32_t measuredX100, bool valid)
{
  if (valid) {
    /* Safety consumers need the latest sample, not a delayed moving average. */
    sensor.temperatureX100 = measuredX100;
    sensor.temperatureValid = true;

    if (sensor.temperatureX100 < TEMP_MIN_X100) {
      sensor.temperatureX100 = TEMP_MIN_X100;
    } else if (sensor.temperatureX100 > TEMP_MAX_X100) {
      sensor.temperatureX100 = TEMP_MAX_X100;
    }

    sensor.temperatureSensorOk = true;
    sensorFailCount = 0U;
    return;
  }

  sensor.temperatureSensorOk = false;
  if (sensorFailCount < UINT8_MAX) {
    sensorFailCount++;
  }

  if ((sensorFailCount >= SENSOR_FAIL_LIMIT) &&
      (sensor.fault == FAULT_NONE)) {
    Sensor_LatchFault(FAULT_TEMP_SENSOR);
  }
}

static void Sensor_UpdateTestFault(uint32_t nowMs)
{
  if (sensor.testMode && (sensor.state != SYS_ERROR) &&
      ((nowMs - sensor.testStartMs) >= TEST_FAULT_MS)) {
    Sensor_LatchFault(FAULT_TEST);
  }
}

static void Sensor_LatchFault(FaultCode_t fault)
{
  bool firstFault = (sensor.fault == FAULT_NONE);

  if (firstFault) {
    sensor.fault = fault;
  }
  sensor.state = SYS_ERROR;
  sensor.testMode = false;

  if (firstFault) {
    CAN_SendFault(fault);
  }
}

static void Sensor_ResetFaults(void)
{
  sensor.state = SYS_NORMAL;
  sensor.fault = FAULT_NONE;
  sensor.streamEnabled = true;
  sensor.testMode = false;
  sensor.vibrationPercent = 0U;
  sensor.vibrationPeakRaw = 0U;
  sensorFailCount = 0U;
}

static Command_t Command_FromChar(char value)
{
  if ((value >= 'a') && (value <= 'z')) {
    value = (char)(value - ('a' - 'A'));
  }

  switch (value) {
    case 'A': return CMD_A;
    case 'X': return CMD_X;
    case 'D': return CMD_D;
    case 'R': return CMD_R;
    case 'T': return CMD_T;
    case 'F': return CMD_F;
    case 'I': return CMD_I;
    case 'S': return CMD_S;
    default:  return CMD_NONE;
  }
}

static char Command_ToChar(Command_t command)
{
  switch (command) {
    case CMD_A: return 'A';
    case CMD_X: return 'X';
    case CMD_D: return 'D';
    case CMD_R: return 'R';
    case CMD_T: return 'T';
    case CMD_F: return 'F';
    case CMD_I: return 'I';
    case CMD_S: return 'S';
    default:    return '?';
  }
}

static bool CAN_QueueFrame(uint16_t stdId, const uint8_t *data, uint8_t dlc,
                           bool urgent)
{
  CanTxItem_t item = {0};
  BaseType_t result;

  if ((stdId > 0x7FFU) || (dlc > 8U) ||
      ((dlc > 0U) && (data == NULL))) {
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
  uint8_t sensorOk =
    ((sensor.state == SYS_NORMAL) && sensor.temperatureSensorOk) ? 1U : 0U;
  uint8_t vibration = (sensor.vibrationPercent > 100U) ?
                      100U : (uint8_t)sensor.vibrationPercent;
  uint8_t payload[8] = {
    (uint8_t)Command_ToChar(command), accepted ? 1U : 0U, reason,
    (uint8_t)sensor.state, sensor.streamEnabled ? 1U : 0U,
    sensorOk, vibration, (uint8_t)sensor.fault
  };

  (void)CAN_QueueFrame(CAN_ID_ACK_TX, payload, 8U, true);
}

static void CAN_SendTelemetry(void)
{
  int16_t temperature = (int16_t)sensor.temperatureX100;
  uint8_t vibration = (sensor.vibrationPercent > 100U) ?
                      100U : (uint8_t)sensor.vibrationPercent;
  uint8_t sensorOk =
    ((sensor.state == SYS_NORMAL) && sensor.temperatureSensorOk) ? 1U : 0U;
  uint8_t payload[8] = {
    (uint8_t)((uint16_t)temperature & 0xFFU),
    (uint8_t)(((uint16_t)temperature >> 8U) & 0xFFU),
    vibration, (uint8_t)sensor.state, sensorOk,
    sensor.streamEnabled ? 1U : 0U,
    sensor.testMode ? 1U : 0U,
    (uint8_t)sensor.fault
  };

  (void)CAN_QueueFrame(CAN_ID_TELEM_TX, payload, 8U, false);
}

static void CAN_SendStatus(bool urgent)
{
  int16_t temperature = (int16_t)sensor.temperatureX100;
  uint8_t vibration = (sensor.vibrationPercent > 100U) ?
                      100U : (uint8_t)sensor.vibrationPercent;
  uint8_t payload[8] = {
    (uint8_t)sensor.state, (uint8_t)sensor.fault,
    sensor.temperatureSensorOk ? 1U : 0U,
    sensor.streamEnabled ? 1U : 0U,
    vibration,
    (uint8_t)((uint16_t)temperature & 0xFFU),
    (uint8_t)(((uint16_t)temperature >> 8U) & 0xFFU),
    0U
  };

  (void)CAN_QueueFrame(CAN_ID_STATUS_TX, payload, 8U, urgent);
}

static void CAN_SendFault(FaultCode_t fault)
{
  int16_t temperature = (int16_t)sensor.temperatureX100;
  uint8_t vibration = (sensor.vibrationPercent > 100U) ?
                      100U : (uint8_t)sensor.vibrationPercent;
  uint8_t payload[8] = {
    (uint8_t)fault, (uint8_t)sensor.state,
    (uint8_t)((uint16_t)temperature & 0xFFU),
    (uint8_t)(((uint16_t)temperature >> 8U) & 0xFFU),
    vibration,
    sensor.temperatureSensorOk ? 1U : 0U,
    sensor.testMode ? 1U : 0U,
    0U
  };

  (void)CAN_QueueFrame(CAN_ID_FAULT_TX, payload, 8U, true);
}

static void CAN_NotifyTxTaskFromISR(void)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (canTxTaskHandle != NULL) {
    vTaskNotifyGiveFromISR(canTxTaskHandle, &higherPriorityTaskWoken);
    portYIELD_FROM_ISR(higherPriorityTaskWoken);
  }
}

#if REAL_TEMP_SENSOR
static void DWT_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static void OW_DelayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t ticks = us * (SystemCoreClock / 1000000UL);

  while ((DWT->CYCCNT - start) < ticks) {
  }
}

static void OW_SetOutput(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = OW_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OW_PORT, &gpio);
}

static void OW_SetInput(void)
{
  GPIO_InitTypeDef gpio = {0};

  gpio.Pin = OW_PIN;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OW_PORT, &gpio);
}

static bool OW_Reset(void)
{
  bool present;
  uint32_t primask;

  OW_SetOutput();
  HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);
  OW_DelayUs(480U);

  primask = __get_PRIMASK();
  __disable_irq();
  OW_SetInput();
  OW_DelayUs(70U);
  present = (HAL_GPIO_ReadPin(OW_PORT, OW_PIN) == GPIO_PIN_RESET);
  __set_PRIMASK(primask);

  OW_DelayUs(410U);
  return present;
}

static void OW_WriteByte(uint8_t value)
{
  for (uint8_t bit = 0U; bit < 8U; bit++) {
    uint32_t primask;

    OW_SetOutput();
    primask = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);

    if ((value & (1U << bit)) != 0U) {
      OW_DelayUs(2U);
      HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_SET);
      OW_DelayUs(58U);
    } else {
      OW_DelayUs(60U);
      HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_SET);
    }

    __set_PRIMASK(primask);
    OW_DelayUs(2U);
  }
}

static uint8_t OW_ReadByte(void)
{
  uint8_t value = 0U;

  for (uint8_t bit = 0U; bit < 8U; bit++) {
    uint32_t primask;

    OW_SetOutput();
    primask = __get_PRIMASK();
    __disable_irq();
    HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_RESET);
    OW_DelayUs(2U);
    OW_SetInput();
    OW_DelayUs(10U);

    if (HAL_GPIO_ReadPin(OW_PORT, OW_PIN) == GPIO_PIN_SET) {
      value |= (uint8_t)(1U << bit);
    }

    OW_DelayUs(50U);
    __set_PRIMASK(primask);
    OW_DelayUs(2U);
  }

  return value;
}

static uint8_t OW_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;

  for (uint8_t index = 0U; index < length; index++) {
    uint8_t value = data[index];

    for (uint8_t bit = 0U; bit < 8U; bit++) {
      uint8_t mix = (uint8_t)((crc ^ value) & 0x01U);
      crc >>= 1U;
      if (mix != 0U) {
        crc ^= 0x8CU;
      }
      value >>= 1U;
    }
  }

  return crc;
}

static bool DS18B20_StartConversion(void)
{
  if (!OW_Reset()) {
    return false;
  }

  OW_WriteByte(0xCCU);
  OW_WriteByte(0x44U);
  return true;
}

static bool DS18B20_ReadTemperature(int32_t *measuredX100)
{
  uint8_t scratchpad[9];
  int16_t raw;
  int32_t value;

  if ((measuredX100 == NULL) || !OW_Reset()) {
    return false;
  }

  OW_WriteByte(0xCCU);
  OW_WriteByte(0xBEU);
  for (uint8_t index = 0U; index < 9U; index++) {
    scratchpad[index] = OW_ReadByte();
  }

  (void)OW_Reset();

  if (OW_Crc8(scratchpad, 8U) != scratchpad[8]) {
    return false;
  }

  raw = (int16_t)((uint16_t)scratchpad[0] |
                  ((uint16_t)scratchpad[1] << 8U));
  if (raw == (int16_t)0x0550) {
    return false;
  }

  value = ((int32_t)raw * 25L) / 4L;
  if ((value < TEMP_MIN_X100) || (value > TEMP_MAX_X100)) {
    return false;
  }

  *measuredX100 = value;
  return true;
}
#endif

static void CAN_ConfigFilters(void)
{
  CAN_FilterTypeDef filter = {0};

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = (uint16_t)(CAN_ID_CMD_RX << 5U);
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5U);
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

  config.Channel = PIEZO_ADC_CHANNEL;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
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

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &gpio);

#if REAL_TEMP_SENSOR
  gpio.Pin = OW_PIN;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(OW_PORT, &gpio);
  HAL_GPIO_WritePin(OW_PORT, OW_PIN, GPIO_PIN_SET);
#endif
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

  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &gpio);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((can->Instance != CAN1) || (sensorEventQueue == NULL)) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) > 0U) {
    CAN_RxHeaderTypeDef header;
    uint8_t payload[8] = {0};
    SensorEvent_t event = {0};
    BaseType_t queued;
    bool urgent;

    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, payload) != HAL_OK) {
      break;
    }

    if ((header.IDE != CAN_ID_STD) ||
        (header.RTR != CAN_RTR_DATA) ||
        (header.StdId != CAN_ID_CMD_RX) ||
        (header.DLC < 1U)) {
      continue;
    }

    event.type = SENSOR_EVENT_COMMAND;
    event.value.commandByte = payload[0];
    urgent = (payload[0] == 'D') || (payload[0] == 'd') ||
             (payload[0] == 'R') || (payload[0] == 'r') ||
             (payload[0] == 'X') || (payload[0] == 'x');

    queued = urgent ?
      xQueueSendToFrontFromISR(
        sensorEventQueue, &event, &higherPriorityTaskWoken) :
      xQueueSendToBackFromISR(
        sensorEventQueue, &event, &higherPriorityTaskWoken);

    if ((queued != pdPASS) && urgent) {
      SensorEvent_t discarded;
      (void)xQueueReceiveFromISR(
        sensorEventQueue, &discarded, &higherPriorityTaskWoken);
      queued = xQueueSendToFrontFromISR(
        sensorEventQueue, &event, &higherPriorityTaskWoken);
    }

    if (queued == pdPASS) {
      dbgEventQueued++;
    } else {
      dbgEventDropped++;
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
