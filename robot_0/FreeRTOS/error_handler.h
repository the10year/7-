#ifndef ERROR_HANDLER_H
#define ERROR_HANDLER_H

#include "stm32f10x.h"
#include <stdint.h>

#define MAX_ERROR_LOG_ENTRIES    10

typedef struct {
    uint32_t timestamp;
    uint8_t error_code;
    uint8_t error_source;
    char message[32];
} ErrorLogEntry_t;

typedef enum {
    ERR_SOURCE_SERVO = 0,
    ERR_SOURCE_SENSOR,
    ERR_SOURCE_COMM,
    ERR_SOURCE_DISPLAY,
    ERR_SOURCE_SYSTEM,
    ERR_SOURCE_MEMORY
} ErrorSource_t;

typedef enum {
    ERR_NONE = 0,
    ERR_TIMEOUT,
    ERR_INVALID_DATA,
    ERR_QUEUE_FULL,
    ERR_SEMAPHORE_TIMEOUT,
    ERR_MEMORY_ALLOC_FAILED,
    ERR_INIT_FAILED,
    ERR_OVERFLOW,
    ERR_CRC_MISMATCH,
    ERR_UNKNOWN
} ErrorCode_t;

extern ErrorLogEntry_t error_log[MAX_ERROR_LOG_ENTRIES];
extern volatile uint32_t error_log_head;
extern volatile uint32_t error_log_tail;
extern volatile uint32_t total_errors;

void ErrorHandler_Init(void);
void ErrorHandler_Log(ErrorSource_t source, ErrorCode_t code, const char *msg);
void ErrorHandler_PrintLog(void);
void ErrorHandler_ClearLog(void);
uint32_t ErrorHandler_GetErrorCount(void);
ErrorCode_t ErrorHandler_GetLastError(ErrorSource_t *source);

#define LOG_ERROR(source, code, msg) ErrorHandler_Log((source), (code), (msg))

#endif
