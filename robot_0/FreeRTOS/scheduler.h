#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "stm32f10x.h"
#include <stdint.h>

#ifndef NULL
#define NULL ((void*)0)
#endif

#define MAX_TASKS            8
#define TASK_NAME_LEN        16

typedef void (*TaskFunction_t)(void);

typedef enum {
    TASK_STATE_READY = 0,
    TASK_STATE_RUNNING,
    TASK_STATE_SUSPENDED,
    TASK_STATE_DELAYED
} TaskState_t;

typedef struct {
    TaskFunction_t function;
    char name[TASK_NAME_LEN];
    uint32_t period_ms;
    uint32_t last_run_tick;
    uint32_t priority;
    TaskState_t state;
    uint32_t stack_size;
} TaskControlBlock_t;

extern volatile uint32_t system_tick_count;

void Scheduler_Init(void);
void Scheduler_CreateTask(TaskFunction_t func, const char *name, 
                         uint32_t priority, uint32_t stack_size, uint32_t period_ms);
void Scheduler_Delay(uint32_t ms);
void Scheduler_Start(void);
uint32_t Scheduler_GetTickCount(void);
void Scheduler_IncTick(void);

#endif
