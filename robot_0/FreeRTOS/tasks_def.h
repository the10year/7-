#ifndef TASKS_DEF_H
#define TASKS_DEF_H

#include "stm32f10x.h"
#include "scheduler.h"
#include "../Hardware/Input.h"

#define TASK_STACK_SIZE_SMALL    128
#define TASK_STACK_SIZE_MEDIUM   256
#define TASK_STACK_SIZE_LARGE    512

#define PRIORITY_HIGHEST         4
#define PRIORITY_HIGH            3
#define PRIORITY_MEDIUM          2
#define PRIORITY_LOW             1
#define PRIORITY_LOWEST          0

typedef struct {
    float lux;
    float distance;
    float noise_db;
    uint8_t noise_level;
} SensorData_t;

typedef struct {
    uint8_t cmd;
    float param1;
    float param2;
    float param3;
    float param4;
} CommandData_t;

typedef struct {
    float angle[8];
    uint32_t timestamp;
} ServoData_t;

typedef enum {
    CMD_NONE = 0,
    CMD_MOVE_FORWARD,
    CMD_MOVE_BACKWARD,
    CMD_TURN_LEFT,
    CMD_TURN_RIGHT,
    CMD_STOP,
    CMD_SWING,
    CMD_WAVE,
    CMD_LIFT_LEG,
    CMD_STAND,
    CMD_SIT,
    CMD_AUTO_MODE,
    CMD_MANUAL_MODE,
    CMD_RESET
} CommandType_t;

typedef enum {
    SENSOR_OK = 0,
    SENSOR_ERROR,
    SENSOR_TIMEOUT
} SensorStatus_t;

typedef enum {
    SYSTEM_NORMAL = 0,
    SYSTEM_WARNING,
    SYSTEM_ERROR,
    SYSTEM_FATAL
} SystemStatus_t;

#define QUEUE_SIZE 10

typedef struct {
    SensorData_t data[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} SensorQueue_t;

typedef struct {
    CommandData_t data[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} CommandQueue_t;

typedef struct {
    ServoData_t data[QUEUE_SIZE];
    uint8_t head;
    uint8_t tail;
    uint8_t count;
} ServoQueue_t;

extern SensorQueue_t sensor_queue;
extern CommandQueue_t command_queue;
extern ServoQueue_t servo_queue;

extern volatile uint8_t servo_mutex_locked;
extern volatile uint8_t uart_mutex_locked;
extern volatile uint8_t oled_mutex_locked;

void vTask_ServoControl(void);
void vTask_SensorRead(void);
void vTask_Communication(void);
void vTask_Display(void);
void vTask_Decision(void);

void System_Init(void);
void System_ErrorHandler(SystemStatus_t status, const char *msg);
void System_Recovery(void);

uint8_t Queue_SendSensor(SensorData_t *data);
uint8_t Queue_ReceiveSensor(SensorData_t *data);
uint8_t Queue_SendCommand(CommandData_t *data);
uint8_t Queue_ReceiveCommand(CommandData_t *data);
uint8_t Queue_SendServo(ServoData_t *data);
uint8_t Queue_ReceiveServo(ServoData_t *data);

#endif
