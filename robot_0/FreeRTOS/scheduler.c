#include "scheduler.h"
#include "Delay.h"

#ifndef NULL
#define NULL ((void*)0)
#endif

static TaskControlBlock_t tasks[MAX_TASKS];
static volatile uint8_t task_count = 0;
volatile uint32_t system_tick_count = 0;

void Scheduler_Init(void) {
    uint8_t i;
    for (i = 0; i < MAX_TASKS; i++) {
        tasks[i].function = NULL;
        tasks[i].state = TASK_STATE_SUSPENDED;
        tasks[i].period_ms = 0;
        tasks[i].last_run_tick = 0;
        tasks[i].priority = 0;
        tasks[i].name[0] = '\0';
    }
    task_count = 0;
    system_tick_count = 0;
}

void Scheduler_CreateTask(TaskFunction_t func, const char *name, 
                         uint32_t priority, uint32_t stack_size, uint32_t period_ms) {
    uint8_t i;
    
    if (task_count >= MAX_TASKS || func == NULL) {
        return;
    }
    
    tasks[task_count].function = func;
    tasks[task_count].state = TASK_STATE_READY;
    tasks[task_count].priority = priority;
    tasks[task_count].stack_size = stack_size;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].last_run_tick = 0;
    
    i = 0;
    while (name[i] != '\0' && i < TASK_NAME_LEN - 1) {
        tasks[task_count].name[i] = name[i];
        i++;
    }
    tasks[task_count].name[i] = '\0';
    
    task_count++;
}

void Scheduler_Delay(uint32_t ms) {
    Delay_ms(ms);
}

uint32_t Scheduler_GetTickCount(void) {
    return system_tick_count;
}

void Scheduler_IncTick(void) {
    system_tick_count++;
}

void Scheduler_Start(void) {
    uint8_t i;
    uint32_t current_tick;
    
    while (1) {
        current_tick = system_tick_count;
        
        for (i = 0; i < task_count; i++) {
            if (tasks[i].state == TASK_STATE_READY && tasks[i].function != NULL) {
                if (current_tick - tasks[i].last_run_tick >= tasks[i].period_ms) {
                    tasks[i].function();
                    tasks[i].last_run_tick = current_tick;
                }
            }
        }
    }
}
