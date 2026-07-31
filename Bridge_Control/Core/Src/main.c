/*
 * STM32F103C8T6 UART <-> CAN bridge - native FreeRTOS version
 *
 * Hardware:
 *   PA9  USART1_TX, PA10 USART1_RX, 115200 8N1
 *   PA11 CAN1_RX, PA12 CAN1_TX, 500 kbit/s
 *   PB12 green status LED, active high
 *   PB14 red alarm LED, active high
 *   PA8  active buzzer, active high
 *   PC13 Blue Pill activity LED, active low
 *
 * FreeRTOS ownership:
 *   BridgeControlTask (priority 5): bridge/alarm state and CAN RX processing
 *   CanTxTask         (priority 4): sole owner of CAN transmission/recovery
 *   UartParserTask    (priority 3): UART frame parser
 *   UartTxTask        (priority 2): sole owner of UART transmission
 *   HeartbeatTask     (priority 1): fixed 1000 ms heartbeat event producer
 *
 * UART protocol and CAN identifiers are unchanged from the bare-metal bridge.
 */

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

CAN_HandleTypeDef hcan;
UART_HandleTypeDef huart1;
static TIM_HandleTypeDef htim2HalTick;

#define ACTIVITY_LED_Pin                   GPIO_PIN_13
#define ACTIVITY_LED_GPIO_Port             GPIOC
#define GREEN_LED_Pin                      GPIO_PIN_12
#define GREEN_LED_GPIO_Port                GPIOB
#define RED_LED_Pin                        GPIO_PIN_14
#define RED_LED_GPIO_Port                  GPIOB
#define BUZZER_Pin                         GPIO_PIN_8
#define BUZZER_GPIO_Port                   GPIOA

#define UART_RX_STREAM_SIZE                512U
#define UART_RX_CHUNK_SIZE                 32U
#define UART_CMD_BUFFER_SIZE               96U
#define UART_TX_MESSAGE_SIZE               192U
#define UART_TX_QUEUE_LENGTH               12U

#define BRIDGE_EVENT_QUEUE_LENGTH          20U
#define CAN_TX_QUEUE_LENGTH                16U

#define HEARTBEAT_PERIOD_MS                1000U
#define CAN_MAILBOX_WAIT_MS                3U
#define CAN_TX_DONE_WAIT_MS                10U
#define CAN_TX_RETRY_LIMIT                 2U
#define CAN_RESTART_COOLDOWN_MS            100U
#define CAN_ESR_BOFF_FLAG                  0x00000004U

#define CAN_ID_MOTOR_CMD                   0x101U
#define CAN_ID_SENSOR_CMD                  0x102U
#define CAN_ID_MOTOR_ACK                   0x181U
#define CAN_ID_MOTOR_TELEM                 0x201U
#define CAN_ID_MOTOR_STATUS                0x211U
#define CAN_ID_MOTOR_FAULT                 0x221U
#define CAN_ID_SENSOR_TELEM                0x202U
#define CAN_ID_SENSOR_FAULT                0x222U

#define SENSOR_VIB_ALARM_PCT               80U
#define SENSOR_DEBUG_TEXT_ENABLE           0U

#define ALARM_REASON_NONE                  0U
#define ALARM_REASON_ESTOP_CMD             (1U << 0U)
#define ALARM_REASON_SENSOR_VIB            (1U << 1U)
#define ALARM_REASON_MOTOR_FAULT           (1U << 2U)
#define ALARM_REASON_SENSOR_FAULT          (1U << 3U)

#define BRIDGE_TASK_PRIORITY               5U
#define CAN_TX_TASK_PRIORITY               4U
#define UART_PARSER_TASK_PRIORITY          3U
#define UART_TX_TASK_PRIORITY              2U
#define HEARTBEAT_TASK_PRIORITY            1U

#define BRIDGE_TASK_STACK_WORDS            384U
#define CAN_TX_TASK_STACK_WORDS            256U
#define UART_PARSER_TASK_STACK_WORDS       384U
#define UART_TX_TASK_STACK_WORDS           256U
#define HEARTBEAT_TASK_STACK_WORDS         160U

typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
  bool extended;
} CanFrame_t;

typedef struct {
  CanFrame_t frame;
  uint8_t retries;
  bool urgent;
} CanTxItem_t;

typedef struct {
  uint16_t length;
  char data[UART_TX_MESSAGE_SIZE];
} UartTxItem_t;

typedef enum {
  BRIDGE_EVENT_NONE = 0,
  BRIDGE_EVENT_CAN_RX,
  BRIDGE_EVENT_UART_CAN_TX,
  BRIDGE_EVENT_SET_DEBUG,
  BRIDGE_EVENT_PING,
  BRIDGE_EVENT_STAT,
  BRIDGE_EVENT_HEARTBEAT
} BridgeEventType_t;

typedef struct {
  BridgeEventType_t type;
  CanFrame_t frame;
  uint8_t value;
} BridgeEvent_t;

typedef struct {
  int16_t lastSensorTempX100;
  uint8_t lastSensorVibrationPct;
  uint8_t lastSensorOk;
  uint8_t sensorDebugTextEnabled;
  uint8_t alarmActive;
  uint8_t alarmReasons;
  uint8_t canUartOutputEnabled;
} BridgeContext_t;

static BridgeContext_t bridge = {
  .lastSensorTempX100 = 0,
  .lastSensorVibrationPct = 0U,
  .lastSensorOk = 0U,
  .sensorDebugTextEnabled = SENSOR_DEBUG_TEXT_ENABLE,
  .alarmActive = 0U,
  .alarmReasons = ALARM_REASON_NONE,
  .canUartOutputEnabled = 1U
};

static uint8_t uartRxByte;
static volatile uint8_t canStarted;

static volatile uint32_t uartToCanCount;
static volatile uint32_t canToUartCount;
static volatile uint32_t uartParseErrorCount;
static volatile uint32_t uartRxDroppedCount;
static volatile uint32_t uartRxErrorCount;
static volatile uint32_t uartTxDroppedCount;
static volatile uint32_t canTxErrorCount;
static volatile uint32_t canRxQueuedCount;
static volatile uint32_t canRxDroppedCount;
static volatile uint32_t canTxQueuedCount;
static volatile uint32_t canTxDroppedCount;
static volatile uint32_t canTxOkCount;
static volatile uint32_t canLastError;
static volatile uint32_t canLastTxId;
static uint32_t lastCanRestartMs;

static QueueHandle_t bridgeEventQueue;
static QueueHandle_t canTxQueue;
static QueueHandle_t uartTxQueue;
static StreamBufferHandle_t uartRxStream;

static StaticQueue_t bridgeEventQueueControl;
static StaticQueue_t canTxQueueControl;
static StaticQueue_t uartTxQueueControl;
static StaticStreamBuffer_t uartRxStreamControl;

static uint8_t bridgeEventQueueStorage[
  BRIDGE_EVENT_QUEUE_LENGTH * sizeof(BridgeEvent_t)];
static uint8_t canTxQueueStorage[
  CAN_TX_QUEUE_LENGTH * sizeof(CanTxItem_t)];
static uint8_t uartTxQueueStorage[
  UART_TX_QUEUE_LENGTH * sizeof(UartTxItem_t)];
static uint8_t uartRxStreamStorage[UART_RX_STREAM_SIZE + 1U];

static TaskHandle_t bridgeTaskHandle;
static TaskHandle_t canTxTaskHandle;
static TaskHandle_t uartParserTaskHandle;
static TaskHandle_t uartTxTaskHandle;
static TaskHandle_t heartbeatTaskHandle;

static StaticTask_t bridgeTaskControl;
static StaticTask_t canTxTaskControl;
static StaticTask_t uartParserTaskControl;
static StaticTask_t uartTxTaskControl;
static StaticTask_t heartbeatTaskControl;

static StackType_t bridgeTaskStack[BRIDGE_TASK_STACK_WORDS];
static StackType_t canTxTaskStack[CAN_TX_TASK_STACK_WORDS];
static StackType_t uartParserTaskStack[UART_PARSER_TASK_STACK_WORDS];
static StackType_t uartTxTaskStack[UART_TX_TASK_STACK_WORDS];
static StackType_t heartbeatTaskStack[HEARTBEAT_TASK_STACK_WORDS];

static StaticTask_t idleTaskControl;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

void SystemClock_Config(void);
void Error_Handler(void);

static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
static void CAN_ConfigFilters(void);
static void RTOS_CreateObjects(void);

static void BridgeControlTask(void *argument);
static void CanTxTask(void *argument);
static void UartParserTask(void *argument);
static void UartTxTask(void *argument);
static void HeartbeatTask(void *argument);

static bool Bridge_PostEvent(const BridgeEvent_t *event, bool urgent);
static void Bridge_HandleEvent(const BridgeEvent_t *event);
static void Bridge_HandleCanRx(const CanFrame_t *frame);
static void Bridge_HandleUartCanTx(const CanFrame_t *frame);
static void Bridge_HandlePing(void);
static void Bridge_SendStatusLine(const char *tag);
static void Bridge_SendHeartbeat(void);

static void Alarm_Trigger(uint8_t reason);
static void Alarm_ClearReason(uint8_t reason);
static void Alarm_UpdateOutputs(void);
static void Alarm_HandleCanFrame(const CanFrame_t *frame);
static void SensorTelemetry_Update(const CanFrame_t *frame);

static bool UART_QueueText(const char *text, bool urgent);
static void UART_QueueError(const char *error);
static void UART_ProcessByte(uint8_t byte, char *buffer, uint16_t *index,
                             bool *inFrame);
static void UART_ProcessCommand(char *command);
static size_t String_LengthBounded(const char *text, size_t limit);
static char ToUpperAscii(char value);

static bool HexCharToNibble(char value, uint8_t *nibble);
static bool ParseHexU32(const char *text, uint32_t *value);
static bool ParseDecU8(const char *text, uint8_t *value);
static bool HexStringToBytes(const char *hex, uint8_t expectedLength,
                             uint8_t *output);

static bool CAN_QueueFrame(const CanFrame_t *frame, bool urgent);
static bool CAN_IsActive(void);
static bool CAN_EnsureStarted(void);
static bool CAN_TransmitFrame(const CanFrame_t *frame);
static void CAN_NotifyTxTaskFromISR(void);

int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  CAN_ConfigFilters();
  RTOS_CreateObjects();

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
    Error_Handler();
  }
  canStarted = 1U;

  if (HAL_CAN_ActivateNotification(
        &hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK) {
    Error_Handler();
  }

  if (HAL_UART_Receive_IT(&huart1, &uartRxByte, 1U) != HAL_OK) {
    Error_Handler();
  }

  vTaskStartScheduler();
  Error_Handler();
  return 0;
}

static void RTOS_CreateObjects(void)
{
  bridgeEventQueue = xQueueCreateStatic(
    BRIDGE_EVENT_QUEUE_LENGTH, sizeof(BridgeEvent_t),
    bridgeEventQueueStorage, &bridgeEventQueueControl);
  canTxQueue = xQueueCreateStatic(
    CAN_TX_QUEUE_LENGTH, sizeof(CanTxItem_t),
    canTxQueueStorage, &canTxQueueControl);
  uartTxQueue = xQueueCreateStatic(
    UART_TX_QUEUE_LENGTH, sizeof(UartTxItem_t),
    uartTxQueueStorage, &uartTxQueueControl);
  uartRxStream = xStreamBufferCreateStatic(
    UART_RX_STREAM_SIZE, 1U,
    uartRxStreamStorage, &uartRxStreamControl);

  configASSERT(bridgeEventQueue != NULL);
  configASSERT(canTxQueue != NULL);
  configASSERT(uartTxQueue != NULL);
  configASSERT(uartRxStream != NULL);

  vQueueAddToRegistry(bridgeEventQueue, "BridgeEvt");
  vQueueAddToRegistry(canTxQueue, "CanTx");
  vQueueAddToRegistry(uartTxQueue, "UartTx");

  bridgeTaskHandle = xTaskCreateStatic(
    BridgeControlTask, "BridgeCtrl", BRIDGE_TASK_STACK_WORDS, NULL,
    BRIDGE_TASK_PRIORITY, bridgeTaskStack, &bridgeTaskControl);
  canTxTaskHandle = xTaskCreateStatic(
    CanTxTask, "CanTx", CAN_TX_TASK_STACK_WORDS, NULL,
    CAN_TX_TASK_PRIORITY, canTxTaskStack, &canTxTaskControl);
  uartParserTaskHandle = xTaskCreateStatic(
    UartParserTask, "UartParse", UART_PARSER_TASK_STACK_WORDS, NULL,
    UART_PARSER_TASK_PRIORITY, uartParserTaskStack, &uartParserTaskControl);
  uartTxTaskHandle = xTaskCreateStatic(
    UartTxTask, "UartTx", UART_TX_TASK_STACK_WORDS, NULL,
    UART_TX_TASK_PRIORITY, uartTxTaskStack, &uartTxTaskControl);
  heartbeatTaskHandle = xTaskCreateStatic(
    HeartbeatTask, "Heartbeat", HEARTBEAT_TASK_STACK_WORDS, NULL,
    HEARTBEAT_TASK_PRIORITY, heartbeatTaskStack, &heartbeatTaskControl);

  configASSERT(bridgeTaskHandle != NULL);
  configASSERT(canTxTaskHandle != NULL);
  configASSERT(uartParserTaskHandle != NULL);
  configASSERT(uartTxTaskHandle != NULL);
  configASSERT(heartbeatTaskHandle != NULL);
}

static void BridgeControlTask(void *argument)
{
  BridgeEvent_t event;

  (void)argument;
  Alarm_UpdateOutputs();

  for (;;) {
    if (xQueueReceive(bridgeEventQueue, &event, portMAX_DELAY) == pdPASS) {
      Bridge_HandleEvent(&event);
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

    canLastTxId = item.frame.id;

    if (CAN_EnsureStarted() && CAN_TransmitFrame(&item.frame)) {
      uartToCanCount++;
      canTxOkCount++;
      continue;
    }

    canTxErrorCount++;
    canLastError = HAL_CAN_GetError(&hcan);

    if (item.urgent && (item.retries < CAN_TX_RETRY_LIMIT)) {
      item.retries++;
      if (xQueueSendToFront(canTxQueue, &item, 0U) != pdPASS) {
        canTxDroppedCount++;
        UART_QueueError("BUSY");
      }
    } else {
      UART_QueueError("CANTX");
    }

    vTaskDelay(pdMS_TO_TICKS(CAN_RESTART_COOLDOWN_MS));
  }
}

static void UartParserTask(void *argument)
{
  uint8_t bytes[UART_RX_CHUNK_SIZE];
  char commandBuffer[UART_CMD_BUFFER_SIZE] = {0};
  uint16_t commandIndex = 0U;
  bool inFrame = false;

  (void)argument;

  for (;;) {
    size_t received = xStreamBufferReceive(
      uartRxStream, bytes, sizeof(bytes), portMAX_DELAY);

    for (size_t i = 0U; i < received; i++) {
      UART_ProcessByte(bytes[i], commandBuffer, &commandIndex, &inFrame);
    }
  }
}

static void UartTxTask(void *argument)
{
  UartTxItem_t item;

  (void)argument;

  for (;;) {
    if (xQueueReceive(uartTxQueue, &item, portMAX_DELAY) != pdPASS) {
      continue;
    }

    if (HAL_UART_Transmit(
          &huart1, (uint8_t *)item.data, item.length, 100U) != HAL_OK) {
      uartTxDroppedCount++;
    }
  }
}

static void HeartbeatTask(void *argument)
{
  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = pdMS_TO_TICKS(HEARTBEAT_PERIOD_MS);

  (void)argument;

  for (;;) {
    BridgeEvent_t event = {0};

    vTaskDelayUntil(&lastWake, period);
    event.type = BRIDGE_EVENT_HEARTBEAT;
    (void)Bridge_PostEvent(&event, false);
  }
}

static bool Bridge_PostEvent(const BridgeEvent_t *event, bool urgent)
{
  BaseType_t result;

  if ((event == NULL) || (bridgeEventQueue == NULL)) {
    return false;
  }

  result = urgent ?
    xQueueSendToFront(bridgeEventQueue, event, 0U) :
    xQueueSendToBack(bridgeEventQueue, event, 0U);

  return result == pdPASS;
}

static void Bridge_HandleEvent(const BridgeEvent_t *event)
{
  switch (event->type) {
    case BRIDGE_EVENT_CAN_RX:
      Bridge_HandleCanRx(&event->frame);
      break;

    case BRIDGE_EVENT_UART_CAN_TX:
      Bridge_HandleUartCanTx(&event->frame);
      break;

    case BRIDGE_EVENT_SET_DEBUG:
      bridge.sensorDebugTextEnabled = event->value ? 1U : 0U;
      UART_QueueText(
        bridge.sensorDebugTextEnabled ?
        "<OK,DBG,1>\r\n" : "<OK,DBG,0>\r\n",
        true);
      break;

    case BRIDGE_EVENT_PING:
      Bridge_HandlePing();
      break;

    case BRIDGE_EVENT_STAT:
      Bridge_SendStatusLine("STAT");
      break;

    case BRIDGE_EVENT_HEARTBEAT:
      Bridge_SendHeartbeat();
      break;

    default:
      break;
  }
}

static void Bridge_HandleCanRx(const CanFrame_t *frame)
{
  char message[96];
  int position;

  if (!frame->extended) {
    SensorTelemetry_Update(frame);
    Alarm_HandleCanFrame(frame);
  }

  if (!bridge.canUartOutputEnabled) {
    return;
  }

  position = snprintf(
    message, sizeof(message), "<CAN,%lX,%u,",
    (unsigned long)frame->id, frame->dlc);

  if ((position < 0) || ((size_t)position >= sizeof(message))) {
    return;
  }

  for (uint8_t i = 0U; i < frame->dlc; i++) {
    int written = snprintf(
      &message[position], sizeof(message) - (size_t)position,
      "%02X", frame->data[i]);

    if ((written < 0) ||
        ((size_t)written >= (sizeof(message) - (size_t)position))) {
      return;
    }
    position += written;
  }

  if (snprintf(
        &message[position], sizeof(message) - (size_t)position,
        ">\r\n") < 0) {
    return;
  }

  if (UART_QueueText(message, false)) {
    canToUartCount++;
    HAL_GPIO_TogglePin(ACTIVITY_LED_GPIO_Port, ACTIVITY_LED_Pin);
  }
}

static void Bridge_HandleUartCanTx(const CanFrame_t *frame)
{
  bool urgent = false;
  char command = '\0';
  bool motorCommand = false;

  if (!frame->extended &&
      ((frame->id == CAN_ID_MOTOR_CMD) ||
       (frame->id == CAN_ID_SENSOR_CMD)) &&
      (frame->dlc >= 1U)) {
    command = ToUpperAscii((char)frame->data[0]);
    motorCommand = (frame->id == CAN_ID_MOTOR_CMD);

    if ((command == 'D') || (command == 'X')) {
      urgent = true;
    }
  }

  if (CAN_QueueFrame(frame, urgent)) {
    if (motorCommand && (command == 'D')) {
      bridge.canUartOutputEnabled = 0U;
      Alarm_Trigger(ALARM_REASON_ESTOP_CMD);
    } else if (motorCommand &&
               ((command == 'A') || (command == 'B') || (command == 'R'))) {
      bridge.canUartOutputEnabled = 1U;
      Alarm_ClearReason(ALARM_REASON_ESTOP_CMD);
    }
    HAL_GPIO_TogglePin(ACTIVITY_LED_GPIO_Port, ACTIVITY_LED_Pin);
  } else {
    UART_QueueError("BUSY");
  }
}

static void Bridge_HandlePing(void)
{
  CanFrame_t motorRequest = {
    .id = CAN_ID_MOTOR_CMD,
    .dlc = 1U,
    .data = { (uint8_t)'I' },
    .extended = false
  };
  CanFrame_t sensorRequest = {
    .id = CAN_ID_SENSOR_CMD,
    .dlc = 1U,
    .data = { (uint8_t)'I' },
    .extended = false
  };
  bool motorQueued = CAN_QueueFrame(&motorRequest, false);
  bool sensorQueued = CAN_QueueFrame(&sensorRequest, false);

  /* Queue status first because urgent UART messages are inserted at the front. */
  Bridge_SendStatusLine("PINGSTAT");

  if (motorQueued || sensorQueued) {
    UART_QueueText("<OK,PING>\r\n", true);
  } else {
    UART_QueueError("PINGTX");
  }
}

static void Bridge_SendStatusLine(const char *tag)
{
  char message[UART_TX_MESSAGE_SIZE];
  uint32_t esr = hcan.Instance->ESR;
  uint32_t tec = (esr >> 16U) & 0xFFU;
  uint32_t rec = (esr >> 24U) & 0xFFU;
  uint32_t lec = (esr >> 4U) & 0x07U;

  snprintf(
    message, sizeof(message),
    "<%s,OK=%u,STATE=%lu,ERR=%lX,ESR=%08lX,MSR=%08lX,TSR=%08lX,RF0R=%08lX,TEC=%lu,REC=%lu,LEC=%lu,PCLK1=%lu>\r\n",
    tag,
    CAN_IsActive() ? 1U : 0U,
    (unsigned long)HAL_CAN_GetState(&hcan),
    (unsigned long)HAL_CAN_GetError(&hcan),
    (unsigned long)esr,
    (unsigned long)hcan.Instance->MSR,
    (unsigned long)hcan.Instance->TSR,
    (unsigned long)hcan.Instance->RF0R,
    (unsigned long)tec,
    (unsigned long)rec,
    (unsigned long)lec,
    (unsigned long)HAL_RCC_GetPCLK1Freq());

  UART_QueueText(message, true);
}

static void Bridge_SendHeartbeat(void)
{
  char message[128];
  uint32_t esr = hcan.Instance->ESR;
  uint32_t tec = (esr >> 16U) & 0xFFU;
  uint32_t rec = (esr >> 24U) & 0xFFU;
  uint32_t lec = (esr >> 4U) & 0x07U;

  snprintf(
    message, sizeof(message),
    "<HB,%lu,%u,%lu,%lu,%lX,%lu,%lu,%lu>\r\n",
    (unsigned long)HAL_GetTick(),
    CAN_IsActive() ? 1U : 0U,
    (unsigned long)canToUartCount,
    (unsigned long)uartToCanCount,
    (unsigned long)HAL_CAN_GetError(&hcan),
    (unsigned long)tec,
    (unsigned long)rec,
    (unsigned long)lec);

  UART_QueueText(message, false);
}

static void Alarm_Trigger(uint8_t reason)
{
  bridge.alarmReasons |= reason;
  bridge.alarmActive = (bridge.alarmReasons != ALARM_REASON_NONE) ? 1U : 0U;
  Alarm_UpdateOutputs();
}

static void Alarm_ClearReason(uint8_t reason)
{
  bridge.alarmReasons &= (uint8_t)~reason;
  bridge.alarmActive = (bridge.alarmReasons != ALARM_REASON_NONE) ? 1U : 0U;
  Alarm_UpdateOutputs();
}

static void Alarm_UpdateOutputs(void)
{
  GPIO_PinState alarmState =
    bridge.alarmActive ? GPIO_PIN_SET : GPIO_PIN_RESET;
  GPIO_PinState normalState =
    bridge.alarmActive ? GPIO_PIN_RESET : GPIO_PIN_SET;

  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, alarmState);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, alarmState);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, normalState);
}

static void Alarm_HandleCanFrame(const CanFrame_t *frame)
{
  if ((frame->id == CAN_ID_MOTOR_ACK) && (frame->dlc >= 5U)) {
    if ((frame->data[1] != 0U) && (frame->data[4] == 0U)) {
      Alarm_ClearReason(ALARM_REASON_MOTOR_FAULT);
    }
    return;
  }

  if ((frame->id == CAN_ID_SENSOR_TELEM) && (frame->dlc >= 8U)) {
    bool vibrationAlarm = (frame->data[2] >= SENSOR_VIB_ALARM_PCT);
    bool sensorFault = (frame->data[3] == 2U) ||
                       (frame->data[4] == 0U) ||
                       (frame->data[7] != 0U);

    if (vibrationAlarm) {
      Alarm_Trigger(ALARM_REASON_SENSOR_VIB);
    } else {
      Alarm_ClearReason(ALARM_REASON_SENSOR_VIB);
    }

    if (sensorFault) {
      Alarm_Trigger(ALARM_REASON_SENSOR_FAULT);
    } else {
      Alarm_ClearReason(ALARM_REASON_SENSOR_FAULT);
    }
    return;
  }

  if ((frame->id == CAN_ID_MOTOR_TELEM) && (frame->dlc >= 4U)) {
    if (frame->data[3] == 0U) {
      Alarm_ClearReason(ALARM_REASON_MOTOR_FAULT);
    } else {
      Alarm_Trigger(ALARM_REASON_MOTOR_FAULT);
    }
    return;
  }

  if ((frame->id == CAN_ID_MOTOR_STATUS) && (frame->dlc >= 5U)) {
    if ((frame->data[1] == 0U) && (frame->data[4] == 0U)) {
      Alarm_ClearReason(ALARM_REASON_MOTOR_FAULT);
    } else {
      Alarm_Trigger(ALARM_REASON_MOTOR_FAULT);
    }
    return;
  }

  if ((frame->id == CAN_ID_MOTOR_FAULT) && (frame->dlc >= 1U)) {
    Alarm_Trigger(ALARM_REASON_MOTOR_FAULT);
    return;
  }

  if ((frame->id == CAN_ID_SENSOR_FAULT) && (frame->dlc >= 1U)) {
    Alarm_Trigger(ALARM_REASON_SENSOR_FAULT);
  }
}

static void SensorTelemetry_Update(const CanFrame_t *frame)
{
  char line[64];
  int32_t absoluteTemperature;

  if ((frame->id != CAN_ID_SENSOR_TELEM) || (frame->dlc < 5U)) {
    return;
  }

  bridge.lastSensorTempX100 =
    (int16_t)((uint16_t)frame->data[0] |
              ((uint16_t)frame->data[1] << 8U));
  bridge.lastSensorVibrationPct = frame->data[2];
  bridge.lastSensorOk = frame->data[4];

  if (!bridge.sensorDebugTextEnabled) {
    return;
  }

  absoluteTemperature = bridge.lastSensorTempX100;
  if (absoluteTemperature < 0) {
    absoluteTemperature = -absoluteTemperature;
  }

  snprintf(
    line, sizeof(line),
    "SENSOR,T=%s%ld.%02ldC,V=%u%%,OK=%u\r\n",
    (bridge.lastSensorTempX100 < 0) ? "-" : "",
    (long)(absoluteTemperature / 100L),
    (long)(absoluteTemperature % 100L),
    bridge.lastSensorVibrationPct,
    bridge.lastSensorOk);

  UART_QueueText(line, false);
}

static bool UART_QueueText(const char *text, bool urgent)
{
  UartTxItem_t item = {0};
  BaseType_t result;
  size_t length;

  if ((text == NULL) || (uartTxQueue == NULL)) {
    return false;
  }

  length = String_LengthBounded(text, UART_TX_MESSAGE_SIZE);
  if ((length == 0U) || (length >= UART_TX_MESSAGE_SIZE)) {
    return false;
  }

  memcpy(item.data, text, length);
  item.length = (uint16_t)length;

  result = urgent ?
    xQueueSendToFront(uartTxQueue, &item, 0U) :
    xQueueSendToBack(uartTxQueue, &item, 0U);

  if ((result != pdPASS) && urgent) {
    UartTxItem_t discarded;
    (void)xQueueReceive(uartTxQueue, &discarded, 0U);
    result = xQueueSendToFront(uartTxQueue, &item, 0U);
  }

  if (result != pdPASS) {
    uartTxDroppedCount++;
    return false;
  }

  return true;
}

static void UART_QueueError(const char *error)
{
  char message[48] = "<ERR,";
  size_t position = 5U;
  size_t errorLength = String_LengthBounded(error, 32U);

  if ((error == NULL) || (errorLength >= 32U)) {
    return;
  }

  memcpy(&message[position], error, errorLength);
  position += errorLength;
  message[position++] = '>';
  message[position++] = '\r';
  message[position++] = '\n';
  message[position] = '\0';

  (void)UART_QueueText(message, true);
}

static void UART_ProcessByte(uint8_t byte, char *buffer, uint16_t *index,
                             bool *inFrame)
{
  if (byte == (uint8_t)'<') {
    *inFrame = true;
    *index = 0U;
    memset(buffer, 0, UART_CMD_BUFFER_SIZE);
    return;
  }

  if (!*inFrame) {
    return;
  }

  if (byte == (uint8_t)'>') {
    buffer[*index] = '\0';
    *inFrame = false;
    *index = 0U;
    UART_ProcessCommand(buffer);
    return;
  }

  if (*index < (UART_CMD_BUFFER_SIZE - 1U)) {
    buffer[(*index)++] = (char)byte;
  } else {
    *inFrame = false;
    *index = 0U;
    uartParseErrorCount++;
    UART_QueueError("LONG");
  }
}

static void UART_ProcessCommand(char *command)
{
  char *tokenCommand;
  char *tokenId;
  char *tokenDlc;
  char *tokenData;
  BridgeEvent_t event = {0};

  tokenCommand = strtok(command, ",");
  tokenId = strtok(NULL, ",");
  tokenDlc = strtok(NULL, ",");
  tokenData = strtok(NULL, ",");

  if (tokenCommand == NULL) {
    uartParseErrorCount++;
    UART_QueueError("EMPTY");
    return;
  }

  if (strcmp(tokenCommand, "DBG") == 0) {
    if ((tokenId == NULL) ||
        ((tokenId[0] != '0') && (tokenId[0] != '1')) ||
        (tokenId[1] != '\0')) {
      uartParseErrorCount++;
      UART_QueueError("DBG");
      return;
    }

    event.type = BRIDGE_EVENT_SET_DEBUG;
    event.value = (tokenId[0] == '1') ? 1U : 0U;
    if (!Bridge_PostEvent(&event, false)) {
      UART_QueueError("BUSY");
    }
    return;
  }

  if (strcmp(tokenCommand, "PING") == 0) {
    event.type = BRIDGE_EVENT_PING;
    if (!Bridge_PostEvent(&event, false)) {
      UART_QueueError("BUSY");
    }
    return;
  }

  if (strcmp(tokenCommand, "STAT") == 0) {
    event.type = BRIDGE_EVENT_STAT;
    if (!Bridge_PostEvent(&event, false)) {
      UART_QueueError("BUSY");
    }
    return;
  }

  if (strcmp(tokenCommand, "TX") != 0) {
    uartParseErrorCount++;
    UART_QueueError("CMD");
    return;
  }

  if ((tokenId == NULL) || (tokenDlc == NULL) || (tokenData == NULL)) {
    uartParseErrorCount++;
    UART_QueueError("FORMAT");
    return;
  }

  if (!ParseHexU32(tokenId, &event.frame.id)) {
    uartParseErrorCount++;
    UART_QueueError("ID");
    return;
  }

  if (event.frame.id > 0x7FFU) {
    uartParseErrorCount++;
    UART_QueueError("STDID");
    return;
  }

  if (!ParseDecU8(tokenDlc, &event.frame.dlc)) {
    uartParseErrorCount++;
    UART_QueueError("DLC");
    return;
  }

  if (event.frame.dlc > 8U) {
    uartParseErrorCount++;
    UART_QueueError("DLC8");
    return;
  }

  if (!HexStringToBytes(
        tokenData, event.frame.dlc, event.frame.data)) {
    uartParseErrorCount++;
    UART_QueueError("DATA");
    return;
  }

  event.type = BRIDGE_EVENT_UART_CAN_TX;
  event.frame.extended = false;

  {
    bool urgent = false;

    if (((event.frame.id == CAN_ID_MOTOR_CMD) ||
         (event.frame.id == CAN_ID_SENSOR_CMD)) &&
        (event.frame.dlc >= 1U)) {
      char value = ToUpperAscii((char)event.frame.data[0]);
      urgent = (value == 'D') || (value == 'X');
    }

    if (!Bridge_PostEvent(&event, urgent)) {
      UART_QueueError("BUSY");
    }
  }
}

static size_t String_LengthBounded(const char *text, size_t limit)
{
  size_t length = 0U;

  if (text == NULL) {
    return 0U;
  }

  while ((length < limit) && (text[length] != '\0')) {
    length++;
  }
  return length;
}

static char ToUpperAscii(char value)
{
  if ((value >= 'a') && (value <= 'z')) {
    return (char)(value - ('a' - 'A'));
  }
  return value;
}

static bool HexCharToNibble(char value, uint8_t *nibble)
{
  if ((value >= '0') && (value <= '9')) {
    *nibble = (uint8_t)(value - '0');
    return true;
  }
  if ((value >= 'A') && (value <= 'F')) {
    *nibble = (uint8_t)(value - 'A' + 10);
    return true;
  }
  if ((value >= 'a') && (value <= 'f')) {
    *nibble = (uint8_t)(value - 'a' + 10);
    return true;
  }
  return false;
}

static bool ParseHexU32(const char *text, uint32_t *value)
{
  uint32_t result = 0U;
  uint8_t digits = 0U;

  if ((text == NULL) || (value == NULL) || (*text == '\0')) {
    return false;
  }

  if ((text[0] == '0') && ((text[1] == 'x') || (text[1] == 'X'))) {
    text += 2;
  }

  if (*text == '\0') {
    return false;
  }

  while (*text != '\0') {
    uint8_t nibble;

    if ((digits >= 8U) || !HexCharToNibble(*text, &nibble)) {
      return false;
    }
    result = (result << 4U) | nibble;
    digits++;
    text++;
  }

  *value = result;
  return true;
}

static bool ParseDecU8(const char *text, uint8_t *value)
{
  uint32_t result = 0U;

  if ((text == NULL) || (value == NULL) || (*text == '\0')) {
    return false;
  }

  while (*text != '\0') {
    if ((*text < '0') || (*text > '9')) {
      return false;
    }

    result = (result * 10U) + (uint32_t)(*text - '0');
    if (result > 255U) {
      return false;
    }
    text++;
  }

  *value = (uint8_t)result;
  return true;
}

static bool HexStringToBytes(const char *hex, uint8_t expectedLength,
                             uint8_t *output)
{
  if ((hex == NULL) || (output == NULL) ||
      (strlen(hex) != ((size_t)expectedLength * 2U))) {
    return false;
  }

  for (uint8_t i = 0U; i < expectedLength; i++) {
    uint8_t high;
    uint8_t low;

    if (!HexCharToNibble(hex[i * 2U], &high) ||
        !HexCharToNibble(hex[(i * 2U) + 1U], &low)) {
      return false;
    }
    output[i] = (uint8_t)((high << 4U) | low);
  }
  return true;
}

static bool CAN_QueueFrame(const CanFrame_t *frame, bool urgent)
{
  CanTxItem_t item = {0};
  BaseType_t result;

  if ((frame == NULL) || frame->extended ||
      (frame->id > 0x7FFU) || (frame->dlc > 8U)) {
    return false;
  }

  item.frame = *frame;
  item.urgent = urgent;

  result = urgent ?
    xQueueSendToFront(canTxQueue, &item, 0U) :
    xQueueSendToBack(canTxQueue, &item, 0U);

  if (result == pdPASS) {
    canTxQueuedCount++;
    return true;
  }

  canTxDroppedCount++;
  return false;
}

static bool CAN_IsActive(void)
{
  HAL_CAN_StateTypeDef state = HAL_CAN_GetState(&hcan);
  bool busOff = ((hcan.Instance->ESR & CAN_ESR_BOFF_FLAG) != 0U);

  return (canStarted != 0U) &&
         (state == HAL_CAN_STATE_LISTENING) &&
         !busOff;
}

static bool CAN_EnsureStarted(void)
{
  uint32_t nowMs = HAL_GetTick();

  if (CAN_IsActive()) {
    return true;
  }

  if ((nowMs - lastCanRestartMs) < CAN_RESTART_COOLDOWN_MS) {
    return false;
  }
  lastCanRestartMs = nowMs;

  HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  HAL_NVIC_DisableIRQ(USB_HP_CAN1_TX_IRQn);
  canStarted = 0U;
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

  canStarted = 1U;
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
    if ((xTaskGetTickCount() - start) >=
        pdMS_TO_TICKS(CAN_MAILBOX_WAIT_MS)) {
      return false;
    }
    vTaskDelay(1U);
  }

  header.StdId = frame->id;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = frame->dlc;
  header.TransmitGlobalTime = DISABLE;

  (void)ulTaskNotifyTake(pdTRUE, 0U);
  if (HAL_CAN_AddTxMessage(
        &hcan, &header, (uint8_t *)frame->data, &mailbox) != HAL_OK) {
    return false;
  }

  if (ulTaskNotifyTake(
        pdTRUE, pdMS_TO_TICKS(CAN_TX_DONE_WAIT_MS)) == 0U) {
    if (HAL_CAN_IsTxMessagePending(&hcan, mailbox)) {
      (void)HAL_CAN_AbortTxRequest(&hcan, mailbox);
    }
    return false;
  }

  return !HAL_CAN_IsTxMessagePending(&hcan, mailbox);
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

  filter.FilterBank = 0U;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000U;
  filter.FilterIdLow = 0x0000U;
  filter.FilterMaskIdHigh = 0x0000U;
  filter.FilterMaskIdLow = 0x0000U;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14U;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
    Error_Handler();
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if ((can->Instance != CAN1) || (bridgeEventQueue == NULL)) {
    return;
  }

  while (HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0) > 0U) {
    CAN_RxHeaderTypeDef header;
    BridgeEvent_t event = {0};
    bool urgent = false;
    BaseType_t result;

    if (HAL_CAN_GetRxMessage(
          can, CAN_RX_FIFO0, &header, event.frame.data) != HAL_OK) {
      break;
    }

    if (header.RTR != CAN_RTR_DATA) {
      continue;
    }

    event.type = BRIDGE_EVENT_CAN_RX;
    event.frame.extended = (header.IDE == CAN_ID_EXT);
    event.frame.id = event.frame.extended ? header.ExtId : header.StdId;
    event.frame.dlc = (header.DLC > 8U) ? 8U : (uint8_t)header.DLC;

    if (!event.frame.extended) {
      if ((event.frame.id == CAN_ID_MOTOR_FAULT) ||
          (event.frame.id == CAN_ID_SENSOR_FAULT)) {
        urgent = true;
      } else if ((event.frame.id == CAN_ID_SENSOR_TELEM) &&
                 (event.frame.dlc >= 3U)) {
        urgent = (event.frame.data[2] >= SENSOR_VIB_ALARM_PCT) ||
                 ((event.frame.dlc >= 8U) &&
                  ((event.frame.data[3] == 2U) ||
                   (event.frame.data[4] == 0U) ||
                   (event.frame.data[7] != 0U)));
      }
    }

    result = urgent ?
      xQueueSendToFrontFromISR(
        bridgeEventQueue, &event, &higherPriorityTaskWoken) :
      xQueueSendToBackFromISR(
        bridgeEventQueue, &event, &higherPriorityTaskWoken);

    if (result == pdPASS) {
      canRxQueuedCount++;
    } else {
      canRxDroppedCount++;
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

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *can)
{
  if (can->Instance == CAN1) {
    canLastError = HAL_CAN_GetError(can);
    CAN_NotifyTxTaskFromISR();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
  BaseType_t higherPriorityTaskWoken = pdFALSE;

  if (uart->Instance != USART1) {
    return;
  }

  if ((uartRxStream == NULL) ||
      (xStreamBufferSendFromISR(
         uartRxStream, &uartRxByte, 1U,
         &higherPriorityTaskWoken) != 1U)) {
    uartRxDroppedCount++;
  }

  if (HAL_UART_Receive_IT(&huart1, &uartRxByte, 1U) != HAL_OK) {
    uartRxErrorCount++;
  }

  portYIELD_FROM_ISR(higherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
  if (uart->Instance == USART1) {
    uartRxErrorCount++;
    __HAL_UART_CLEAR_OREFLAG(uart);
    (void)HAL_UART_Receive_IT(&huart1, &uartRxByte, 1U);
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

void USART1_IRQHandler(void)
{
  HAL_UART_IRQHandler(&huart1);
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
  timerClock = ((RCC->CFGR & RCC_CFGR_PPRE1) == 0U) ?
               pclk1 : (pclk1 * 2U);

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
  RCC_OscInitTypeDef oscillator = {0};
  RCC_ClkInitTypeDef clocks = {0};

  oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  oscillator.HSEState = RCC_HSE_ON;
  oscillator.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  oscillator.HSIState = RCC_HSI_ON;
  oscillator.PLL.PLLState = RCC_PLL_ON;
  oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  oscillator.PLL.PLLMUL = RCC_PLL_MUL9;

  if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
    Error_Handler();
  }

  clocks.ClockType = RCC_CLOCKTYPE_HCLK |
                     RCC_CLOCKTYPE_SYSCLK |
                     RCC_CLOCKTYPE_PCLK1 |
                     RCC_CLOCKTYPE_PCLK2;
  clocks.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clocks.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clocks.APB1CLKDivider = RCC_HCLK_DIV2;
  clocks.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&clocks, FLASH_LATENCY_2) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_CAN_Init(void)
{
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4U;
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

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200U;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&huart1) != HAL_OK) {
    Error_Handler();
  }
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();

  HAL_GPIO_WritePin(
    ACTIVITY_LED_GPIO_Port, ACTIVITY_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_SET);

  gpio.Pin = ACTIVITY_LED_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACTIVITY_LED_GPIO_Port, &gpio);

  gpio.Pin = BUZZER_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUZZER_GPIO_Port, &gpio);

  gpio.Pin = GREEN_LED_Pin | RED_LED_Pin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  Alarm_UpdateOutputs();
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

void HAL_UART_MspInit(UART_HandleTypeDef *uart)
{
  GPIO_InitTypeDef gpio = {0};

  if (uart->Instance != USART1) {
    return;
  }

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Pin = GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_10;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_NVIC_SetPriority(USART1_IRQn, 6U, 0U);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *timer)
{
  if (timer->Instance == TIM2) {
    __HAL_RCC_TIM2_CLK_ENABLE();
  }
}

void Error_Handler(void)
{
  __disable_irq();

  HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);

  for (;;) {
    volatile uint32_t delay;

    HAL_GPIO_TogglePin(ACTIVITY_LED_GPIO_Port, ACTIVITY_LED_Pin);
    for (delay = 0U; delay < 500000U; delay++) {
      __NOP();
    }
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
