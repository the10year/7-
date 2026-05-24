#include "error_handler.h"
#include "Serial.h"
#include <string.h>

ErrorLogEntry_t error_log[MAX_ERROR_LOG_ENTRIES];
volatile uint32_t error_log_head = 0;
volatile uint32_t error_log_tail = 0;
volatile uint32_t total_errors = 0;

static const char *error_source_names[] = {
    "SERVO", "SENSOR", "COMM", "DISPLAY", "SYSTEM", "MEMORY"
};

static const char *error_code_names[] = {
    "NONE", "TIMEOUT", "INVALID_DATA", "QUEUE_FULL", 
    "SEMAPHORE_TIMEOUT", "MEM_ALLOC_FAILED", "INIT_FAILED", 
    "OVERFLOW", "CRC_MISMATCH", "UNKNOWN"
};

void ErrorHandler_Init(void) {
    for (uint32_t i = 0; i < MAX_ERROR_LOG_ENTRIES; i++) {
        error_log[i].timestamp = 0;
        error_log[i].error_code = ERR_NONE;
        error_log[i].error_source = 0;
        error_log[i].message[0] = '\0';
    }
    error_log_head = 0;
    error_log_tail = 0;
    total_errors = 0;
}

void ErrorHandler_Log(ErrorSource_t source, ErrorCode_t code, const char *msg) {
    uint32_t next_head = (error_log_head + 1) % MAX_ERROR_LOG_ENTRIES;
    
    if (next_head == error_log_tail) {
        error_log_tail = (error_log_tail + 1) % MAX_ERROR_LOG_ENTRIES;
    }
    
    error_log[error_log_head].timestamp = 0;
    error_log[error_log_head].error_source = source;
    error_log[error_log_head].error_code = code;
    strncpy((char*)error_log[error_log_head].message, msg, 31);
    error_log[error_log_head].message[31] = '\0';
    
    error_log_head = next_head;
    total_errors++;
}

void ErrorHandler_PrintLog(void) {
    Serial_SendString("\r\n=== Error Log ===\r\n");
    
    if (error_log_head == error_log_tail) {
        Serial_SendString("No errors logged.\r\n");
        return;
    }
    
    uint32_t i = error_log_tail;
    while (i != error_log_head) {
        Serial_Printf("[%lu] %s: %s - %s\r\n",
            error_log[i].timestamp,
            error_source_names[error_log[i].error_source],
            error_code_names[error_log[i].error_code],
            error_log[i].message);
        i = (i + 1) % MAX_ERROR_LOG_ENTRIES;
    }
    
    Serial_SendString("=== End of Log ===\r\n");
}

void ErrorHandler_ClearLog(void) {
    for (uint32_t i = 0; i < MAX_ERROR_LOG_ENTRIES; i++) {
        error_log[i].timestamp = 0;
        error_log[i].error_code = ERR_NONE;
        error_log[i].error_source = 0;
        error_log[i].message[0] = '\0';
    }
    error_log_head = 0;
    error_log_tail = 0;
    total_errors = 0;
}

uint32_t ErrorHandler_GetErrorCount(void) {
    return total_errors;
}

ErrorCode_t ErrorHandler_GetLastError(ErrorSource_t *source) {
    if (error_log_head == error_log_tail) {
        if (source != NULL) {
            *source = ERR_SOURCE_SYSTEM;
        }
        return ERR_NONE;
    }
    
    uint32_t last_entry = (error_log_head == 0) ? MAX_ERROR_LOG_ENTRIES - 1 : error_log_head - 1;
    
    if (source != NULL) {
        *source = (ErrorSource_t)error_log[last_entry].error_source;
    }
    
    return (ErrorCode_t)error_log[last_entry].error_code;
}