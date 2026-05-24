#include "tasks_def.h"
#include "Servo.h"
#include "LED.h"
#include "OLED.h"
#include "Serial.h"
#include "bh1750.h"
#include "hcsr04.h"
#include "noise_sensor.h"
#include "Input.h"
#include "Delay.h"
#include <string.h>

SensorQueue_t sensor_queue;
CommandQueue_t command_queue;
ServoQueue_t servo_queue;

volatile uint8_t servo_mutex_locked = 0;
volatile uint8_t uart_mutex_locked = 0;
volatile uint8_t oled_mutex_locked = 0;

static volatile SystemStatus_t system_status = SYSTEM_NORMAL;
static volatile uint32_t error_count = 0;

static volatile uint8_t auto_mode = 0;
static volatile uint8_t action_state = 0;
static volatile uint16_t step_counter = 0;

uint8_t Queue_SendSensor(SensorData_t *data) {
    if (sensor_queue.count >= QUEUE_SIZE) {
        return 0;
    }
    sensor_queue.data[sensor_queue.head] = *data;
    sensor_queue.head = (sensor_queue.head + 1) % QUEUE_SIZE;
    sensor_queue.count++;
    return 1;
}

uint8_t Queue_ReceiveSensor(SensorData_t *data) {
    if (sensor_queue.count == 0) {
        return 0;
    }
    *data = sensor_queue.data[sensor_queue.tail];
    sensor_queue.tail = (sensor_queue.tail + 1) % QUEUE_SIZE;
    sensor_queue.count--;
    return 1;
}

uint8_t Queue_SendCommand(CommandData_t *data) {
    if (command_queue.count >= QUEUE_SIZE) {
        return 0;
    }
    command_queue.data[command_queue.head] = *data;
    command_queue.head = (command_queue.head + 1) % QUEUE_SIZE;
    command_queue.count++;
    return 1;
}

uint8_t Queue_ReceiveCommand(CommandData_t *data) {
    if (command_queue.count == 0) {
        return 0;
    }
    *data = command_queue.data[command_queue.tail];
    command_queue.tail = (command_queue.tail + 1) % QUEUE_SIZE;
    command_queue.count--;
    return 1;
}

uint8_t Queue_SendServo(ServoData_t *data) {
    if (servo_queue.count >= QUEUE_SIZE) {
        return 0;
    }
    servo_queue.data[servo_queue.head] = *data;
    servo_queue.head = (servo_queue.head + 1) % QUEUE_SIZE;
    servo_queue.count++;
    return 1;
}

uint8_t Queue_ReceiveServo(ServoData_t *data) {
    if (servo_queue.count == 0) {
        return 0;
    }
    *data = servo_queue.data[servo_queue.tail];
    servo_queue.tail = (servo_queue.tail + 1) % QUEUE_SIZE;
    servo_queue.count--;
    return 1;
}

void vTask_ServoControl(void) {
    static ServoData_t servo_data;
    
    if (!servo_mutex_locked) {
        servo_mutex_locked = 1;
        if (Queue_ReceiveServo(&servo_data)) {
            Servo_SetAngle1(servo_data.angle[0]);
            Servo_SetAngle2(servo_data.angle[1]);
            Servo_SetAngle3(servo_data.angle[2]);
            Servo_SetAngle4(servo_data.angle[3]);
            Servo_SetAngle5(servo_data.angle[4]);
            Servo_SetAngle6(servo_data.angle[5]);
            Servo_SetAngle7(servo_data.angle[6]);
            Servo_SetAngle8(servo_data.angle[7]);
        }
        servo_mutex_locked = 0;
    }
}

void vTask_SensorRead(void) {
    static SensorData_t sensor_data;
    static NoiseData_TypeDef noise_data;
    
    sensor_data.lux = bh1750_get_lux();
    sensor_data.distance = hcsr04_get_distance();
    Noise_Sensor_GetData(&noise_data);
    sensor_data.noise_db = noise_data.db_value;
    sensor_data.noise_level = noise_data.level;
    
    Queue_SendSensor(&sensor_data);
}

void vTask_Communication(void) {
    static CommandData_t cmd_data;
    static uint8_t rx_buffer[64];
    uint8_t i;
    
    if (Serial_RxFlag == 1) {
        if (!uart_mutex_locked) {
            uart_mutex_locked = 1;
            for (i = 0; i < 63 && Serial_RxPacket[i] != '\0'; i++) {
                rx_buffer[i] = Serial_RxPacket[i];
            }
            rx_buffer[i] = '\0';
            Serial_RxFlag = 0;
            uart_mutex_locked = 0;
            
            cmd_data.cmd = CMD_NONE;
            if (rx_buffer[0] == 'U' && rx_buffer[1] == 'P') {
                cmd_data.cmd = CMD_MOVE_FORWARD;
            } else if (rx_buffer[0] == 'B' && rx_buffer[1] == 'A' && rx_buffer[2] == 'C' && rx_buffer[3] == 'K') {
                cmd_data.cmd = CMD_MOVE_BACKWARD;
            } else if (rx_buffer[0] == 'L' && rx_buffer[1] == 'E' && rx_buffer[2] == 'F' && rx_buffer[3] == 'T') {
                cmd_data.cmd = CMD_TURN_LEFT;
            } else if (rx_buffer[0] == 'R' && rx_buffer[1] == 'I' && rx_buffer[2] == 'G' && rx_buffer[3] == 'H' && rx_buffer[4] == 'T') {
                cmd_data.cmd = CMD_TURN_RIGHT;
            } else if (rx_buffer[0] == 'Y' && rx_buffer[1] == 'B') {
                cmd_data.cmd = CMD_SWING;
            } else if (rx_buffer[0] == 'Z' && rx_buffer[1] == 'S') {
                cmd_data.cmd = CMD_WAVE;
            } else if (rx_buffer[0] == 'T' && rx_buffer[1] == 'J') {
                cmd_data.cmd = CMD_LIFT_LEG;
            } else if (rx_buffer[0] == 'Z' && rx_buffer[1] == 'K') {
                cmd_data.cmd = CMD_STAND;
            } else if (rx_buffer[0] == 'Z' && rx_buffer[1] == 'X') {
                cmd_data.cmd = CMD_SIT;
            } else if (rx_buffer[0] == 'A' && rx_buffer[1] == 'U' && rx_buffer[2] == 'T' && rx_buffer[3] == 'O') {
                cmd_data.cmd = CMD_AUTO_MODE;
            } else if (rx_buffer[0] == 'M' && rx_buffer[1] == 'A' && rx_buffer[2] == 'N' && rx_buffer[3] == 'U') {
                cmd_data.cmd = CMD_MANUAL_MODE;
            } else if (rx_buffer[0] == 'R' && rx_buffer[1] == 'E' && rx_buffer[2] == 'S' && rx_buffer[3] == 'T') {
                cmd_data.cmd = CMD_RESET;
            }
            
            if (cmd_data.cmd != CMD_NONE) {
                Queue_SendCommand(&cmd_data);
            }
        }
    }
}

void vTask_Display(void) {
    static SensorData_t sensor_data;
    
    if (!oled_mutex_locked) {
        oled_mutex_locked = 1;
        if (Queue_ReceiveSensor(&sensor_data)) {
            OLED_Clear();
            OLED_Printf(0, 0, 8, "Lux:%.0f D:%.0fcm", 
                sensor_data.lux, 
                sensor_data.distance > 0 ? sensor_data.distance : 0);
            OLED_Printf(0, 16, 8, "dB:%.1f", sensor_data.noise_db);
            
            switch (system_status) {
                case SYSTEM_NORMAL:
                    OLED_Printf(0, 32, 8, "Status: Normal");
                    break;
                case SYSTEM_WARNING:
                    OLED_Printf(0, 32, 8, "Status: Warning");
                    break;
                case SYSTEM_ERROR:
                    OLED_Printf(0, 32, 8, "Status: Error");
                    break;
                case SYSTEM_FATAL:
                    OLED_Printf(0, 32, 8, "Status: Fatal");
                    break;
            }
            OLED_Update();
        }
        oled_mutex_locked = 0;
    }
}

void vTask_Decision(void) {
    static SensorData_t sensor_data;
    static CommandData_t cmd_data;
    static ServoData_t servo_data;
    
    step_counter++;
    
    if (Queue_ReceiveCommand(&cmd_data)) {
        switch (cmd_data.cmd) {
            case CMD_MOVE_FORWARD:
                action_state = 1;
                step_counter = 0;
                auto_mode = 0;
                break;
            case CMD_MOVE_BACKWARD:
                action_state = 2;
                step_counter = 0;
                auto_mode = 0;
                break;
            case CMD_TURN_LEFT:
                action_state = 3;
                step_counter = 0;
                auto_mode = 0;
                break;
            case CMD_TURN_RIGHT:
                action_state = 4;
                step_counter = 0;
                auto_mode = 0;
                break;
            case CMD_SWING:
                action_state = 5;
                step_counter = 0;
                auto_mode = 0;
                break;
            case CMD_AUTO_MODE:
                auto_mode = 1;
                action_state = 1;
                step_counter = 0;
                break;
            case CMD_MANUAL_MODE:
                auto_mode = 0;
                break;
            case CMD_RESET:
                auto_mode = 0;
                action_state = 0;
                servo_data.angle[0] = 135; servo_data.angle[1] = 45;
                servo_data.angle[2] = 45;  servo_data.angle[3] = 135;
                servo_data.angle[4] = 0;   servo_data.angle[5] = 180;
                servo_data.angle[6] = 0;   servo_data.angle[7] = 180;
                Queue_SendServo(&servo_data);
                break;
            default:
                break;
        }
    }
    
    if (Queue_ReceiveSensor(&sensor_data)) {
        if (auto_mode && sensor_data.distance > 0 && sensor_data.distance < 10.0f) {
            action_state = 3;
        } else if (auto_mode && sensor_data.distance >= 10.0f) {
            action_state = 1;
        }
    }
    
    switch (action_state) {
        case 1:
            servo_data.angle[0] = 135 + (step_counter % 20 < 10 ? 10 : -10);
            servo_data.angle[1] = 45 + (step_counter % 20 < 10 ? -10 : 10);
            servo_data.angle[2] = 45 + (step_counter % 20 < 10 ? -10 : 10);
            servo_data.angle[3] = 135 + (step_counter % 20 < 10 ? 10 : -10);
            servo_data.angle[4] = 0;
            servo_data.angle[5] = 180;
            servo_data.angle[6] = 0;
            servo_data.angle[7] = 180;
            Queue_SendServo(&servo_data);
            break;
        case 5:
            servo_data.angle[0] = 135;
            servo_data.angle[1] = 45;
            servo_data.angle[2] = 45;
            servo_data.angle[3] = 135;
            servo_data.angle[4] = (step_counter % 40 < 20 ? (step_counter % 20) * 2 : (40 - step_counter % 40) * 2);
            servo_data.angle[5] = 180 - servo_data.angle[4];
            servo_data.angle[6] = servo_data.angle[4];
            servo_data.angle[7] = 180 - servo_data.angle[4];
            Queue_SendServo(&servo_data);
            break;
        default:
            break;
    }
}

void System_Init(void) {
    sensor_queue.head = 0;
    sensor_queue.tail = 0;
    sensor_queue.count = 0;
    
    command_queue.head = 0;
    command_queue.tail = 0;
    command_queue.count = 0;
    
    servo_queue.head = 0;
    servo_queue.tail = 0;
    servo_queue.count = 0;
    
    Scheduler_Init();
    
    Scheduler_CreateTask(vTask_Communication, "Comm", PRIORITY_HIGHEST, TASK_STACK_SIZE_MEDIUM, 5);
    Scheduler_CreateTask(vTask_ServoControl, "Servo", PRIORITY_HIGH, TASK_STACK_SIZE_MEDIUM, 5);
    Scheduler_CreateTask(vTask_Decision, "Decision", PRIORITY_HIGH, TASK_STACK_SIZE_LARGE, 25);
    Scheduler_CreateTask(vTask_SensorRead, "Sensor", PRIORITY_MEDIUM, TASK_STACK_SIZE_MEDIUM, 50);
    Scheduler_CreateTask(vTask_Display, "Display", PRIORITY_LOW, TASK_STACK_SIZE_SMALL, 100);
    
    Servo_Init();
    Serial_Init();
    Input_Init();
    OLED_Init();
    LED_Init();
    bh1750_init();
    Noise_Sensor_Init();
    hcsr04_init();
    
    LED_StartBlink();
}

void System_ErrorHandler(SystemStatus_t status, const char *msg) {
    system_status = status;
    
    if (status >= SYSTEM_ERROR) {
        LED_StopBlink();
        LED1_ON();
        LED2_ON();
    }
    
    if (!uart_mutex_locked) {
        uart_mutex_locked = 1;
        Serial_Printf("Error: %s\r\n", msg);
        uart_mutex_locked = 0;
    }
}

void System_Recovery(void) {
    system_status = SYSTEM_NORMAL;
    error_count = 0;
    LED_StartBlink();
}
