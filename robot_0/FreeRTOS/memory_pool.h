#ifndef MEMORY_POOL_H
#define MEMORY_POOL_H

#include "stm32f10x.h"
#include <stdint.h>

#ifndef NULL
#define NULL ((void*)0)
#endif

#define MEM_POOL_BLOCK_SIZE    64
#define MEM_POOL_BLOCK_COUNT   16

typedef struct MemBlock {
    uint8_t buffer[MEM_POOL_BLOCK_SIZE];
    struct MemBlock *next;
} MemBlock_t;

typedef struct {
    MemBlock_t blocks[MEM_POOL_BLOCK_COUNT];
    MemBlock_t *free_list;
    uint8_t initialized;
} MemPool_t;

extern MemPool_t system_mem_pool;

void MemPool_Init(MemPool_t *pool);
void* MemPool_Alloc(MemPool_t *pool);
void MemPool_Free(MemPool_t *pool, void *ptr);
uint32_t MemPool_GetFreeBlocks(MemPool_t *pool);
uint32_t MemPool_GetUsedBlocks(MemPool_t *pool);

#endif
